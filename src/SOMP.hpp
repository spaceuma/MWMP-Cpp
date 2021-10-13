// MIT License
// -----------
//
// Copyright (c) 2021 University of Malaga
// Permission is hereby granted, free of charge, to any person
// obtaining a copy of this software and associated documentation
// files (the "Software"), to deal in the Software without
// restriction, including without limitation the rights to use,
// copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following
// conditions:
//
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
// OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
// HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
// WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
// OTHER DEALINGS IN THE SOFTWARE.
//
// Author: Gonzalo J. Paz Delgado
// Supervisor: Carlos J. Pérez del Pulgar
// Affiliation: Department of Systems Engineering and Automation
// Space Robotics Lab (www.uma.es/space-robotics)

#ifndef __MOTION_PLANNER_SOMP__
#define __MOTION_PLANNER_SOMP__

#include <exception>
#include <opencv2/opencv.hpp>
#include <vector>

#include "FileManager.hpp"
#include "StateSpaceModels.hpp"

#define pi 3.14159265359
#define inf 1000000007

#define reset "\033[0m"
#define black "\033[1;30m"
#define red "\033[1;31m"
#define green "\033[1;32m"
#define yellow "\033[1;33m"
#define blue "\033[1;34m"
#define magenta "\033[1;35m"
#define cyan "\033[1;36m"
#define white "\033[1;37m"

namespace SOMP
{
/*********************************************************************************************
 *
 *  SOMP: Stepped Optimal Motion Planner, based on unconstrained and constrained SLQr.
 *  Given the system modelled by "state_space_model",
 *  and the configurations given in "config", the SLQR motion planning
 *  problem is solved, trying to reach the objective given by "x0" and "u0"
 *  from the current state "x" and control "u".
 *  The results can be obtained in the matrices "x" and "u".
 *  Info about the map can be provided through "map_info" parameter.
 *
 * CONSTRUCTORS:
 * SOMP::MotionPlanner * somp_planner = new SOMP::MotionPlanner(Config config,
 *                                                              SSModel state_space_model);
 * SOMP::MotionPlanner * somp_planner = new SOMP::MotionPlanner(Config config,
 *                                                              SSModel state_space_model,
 *                                                              MapInfo map_info);
 *
 * "Config config" is a struct containing:
 *     - Planner horizon time "double time_horizon". Default: 160s.
 *     - Time step "double time_step". Default: 0.8s.
 *     - Maximum number of iterations "uint max_iterations". Default: 200.
 *     - Acceptable control actuation step percentage to consider
 *       convergence is reached, "double control_threshold". Default: 1e-3.
 *     - Step of the linear search procedure, "double line_search_step".
 *       Default: 0.30.
 *     - Yes/no about check distance to goal, "bool check_distance".
 *     - Yes/no about check final orientation, "bool check_orientation".
 *
 * "SSModel state_space_model" should be an instance of the state space model to be used.
 *
 * "MapInfo map_info" is a struct only needed if checking safety, should contain:
 *     - Map resolution "double map_resolution".
 *     - Obstacles map "std::vector<std::vector<uint>> obstacles_map".
 *     - Repulsive cost from obstacles "double obstacles_cost".
 *
 *********************************************************************************************/

struct Config
{
    double time_horizon;
    double time_step;

    uint max_iterations;
    double control_threshold;
    double line_search_step;

    bool check_distance;

    bool check_orientation;
};

struct MapInfo
{
    double map_resolution;
    std::vector<std::vector<uint>> obstacles_map;
    double obstacles_cost;
};

class MotionPlanner
{
private:
    //********************//
    // Dependency classes //
    //********************//

    // State space model of the robot
    StateSpaceModels::MobileManipulator * robot_ss_model;

    //*************************//
    // Configurable Parameters //
    //*************************//

    // Horizon for the planning task, default 160 seconds
    double time_horizon = 160;

    // Discretization step for the time horizon, default 0.8 seconds
    double time_step = 0.8;

    // Max number of iterations of the motion planner, default 200 iterations
    uint max_iterations = 200;

    // Max control actuation step percentage to consider the planner
    // has converged, default 1e-3
    double control_threshold = 1e-3;

    // Step of the line search procedure, bigger steps reduce computational cost
    // but also reduce convergence ratio, default 0.30
    double line_search_step = 0.30;

    // Check distance to goal to ensure successfull motion plans, default true
    bool check_distance = true;

    // Parameters that depend on the model, defining the index in the state vector
    // where to check for the distance to the goal, and which is the minimum to
    // consider convergence is reached
    std::vector<uint> distance_indexes;
    double distance_threshold;

    // Check goal orientation to ensure successfull motion plans, default true
    bool check_orientation = true;

    // Parameters that depend on the model, defining the index in the state vector
    // where to check for the orientation goal, and which is the minimum to
    // consider convergence is reached
    std::vector<uint> orientation_indexes;
    double orientation_threshold;

    // Characteristics of the scenario
    // Decide whether to check obstacle collisions or not, default false
    bool check_safety = false;

    // Parameters that depend on the model, defining the index in the state vector
    // where to check for the pose of the robot to confirm safety
    std::vector<uint> pose_indexes;

    // Map resolution in meters, default 0.05m
    double map_resolution = 0.05;

    // Boolean map with obstacles as 0 and safe areas as 1, default 5x5m non obstacle map
    std::vector<std::vector<uint>> obstacles_map;

    // Auxiliar maps that ensure safety of the rover
    std::vector<std::vector<uint>> dilated_obstacles_map;
    std::vector<std::vector<uint>> safety_obstacles_map;

    // Gradient of the obstacles map in X and Y directions, used for computing the cost
    // of the robot pose with respect to nearby obstacles
    std::vector<std::vector<double>> gradient_obstacles_map_x;
    std::vector<std::vector<double>> gradient_obstacles_map_y;

    //*********************//
    // Motion Plan Results //
    //*********************//

    // Resulting state vector. Size: number_states x number_time_steps
    std::vector<std::vector<double>> planned_state;

    // Resulting state vector. Size: number_inputs x number_time_steps
    std::vector<std::vector<double>> planned_control;

    //**********************//
    // Supporting variables //
    //**********************//

    // Flag to check if there is already a planned motion
    bool is_motion_planned = false;

    // Number of time steps, default 200 (160/0.8)
    uint number_time_steps = 200;

    // State space model properties
    uint number_states;
    uint number_inputs;
    uint number_si_constraints;
    uint number_ps_constraints;

    //**********************//
    // Supporting functions //
    //**********************//

    // Get the complete time-horizon state, ss linearized matrixes and cost matrixes
    bool updateHorizon(std::vector<std::vector<double>> & x,
                       const std::vector<std::vector<double>> & u,
                       std::vector<std::vector<std::vector<double>>> & Ah,
                       std::vector<std::vector<std::vector<double>>> & Bh,
                       std::vector<std::vector<std::vector<double>>> & Qh,
                       std::vector<std::vector<std::vector<double>>> & Rh,
                       std::vector<std::vector<std::vector<double>>> & Kh);

    // Get the complete time-horizon state, ss linearized matrixes and cost matrixes
    bool updateHorizonConstraints(std::vector<std::vector<std::vector<double>>> & Ch,
                                  std::vector<std::vector<std::vector<double>>> & Dh,
                                  std::vector<std::vector<double>> & rh,
                                  std::vector<std::vector<std::vector<double>>> & Gh,
                                  std::vector<std::vector<double>> & hh);

    // Get the gradient of the obstacles map
    bool computeObstaclesGradient(const std::vector<std::vector<uint>> & obst_map);

    // Dilate a binary obstacles map to ensure safety
    bool dilateObstaclesMap(const std::vector<std::vector<uint>> & obst_map,
                            double dilatation_distance,
                            std::vector<std::vector<uint>> & dilated_map);

    // Compute the line search procedure, trying to find the best way to apply
    // the computed state and control steps (xh and uh),if convergence is not
    // reached yet, decreasing the intensity of the actuation step
    bool computeLineSearch(std::vector<std::vector<double>> & x,
                           const std::vector<std::vector<double>> & x0,
                           std::vector<std::vector<double>> & u,
                           const std::vector<std::vector<double>> & u0,
                           const std::vector<std::vector<double>> & uh,
                           const std::vector<std::vector<std::vector<double>>> & Qh,
                           const std::vector<std::vector<std::vector<double>>> & Rh);

public:
    //*******************//
    // Class Constructor //
    //*******************//
    MotionPlanner(StateSpaceModels::MobileManipulator * _robot_ss_model);
    MotionPlanner(StateSpaceModels::MobileManipulator * _robot_ss_model, Config config);
    MotionPlanner(StateSpaceModels::MobileManipulator * _robot_ss_model,
                  Config config,
                  MapInfo map_info);

    //***********************//
    // Configuring Functions //
    //***********************//
    bool setTimeHorizon(double new_time_horizon);
    bool setTimeStep(double new_time_step);

    //********************//
    // Planning Functions //
    //********************//
    /******************************************************************************************
     *
     * USAGE:
     *
     * "std::vector<double> x" are the initial
     *  states. Size number_states.
     *
     * "std::vector<std::vector<double>> x0" are the reference
     *  states for the whole time horizon (usually filled with zeros except
     *  the last index which represents the goal).
     *  Size number_time_steps x number_states.
     *
     * "std::vector<double> u" are the initial control inputs.
     * Size number_inputs.
     *
     * "std::vector<std::vector<double>> u0" are the reference
     *  control inputs for the whole time horizon (usually filled with zeros).
     *  Size number_time_steps x number_inputs.
     *
     * If convergence is reached, this functions will return "1", if not:
     *     " 0" --> something unexpected happened.
     *     "-1" --> the goal is still far away.
     *     "-2" --> the algorithm is still hardly updating the control.
     *     "-3" --> the generated state is not safe.
     *     "-4" --> something unexpected happened.
     *
     ******************************************************************************************/

    int generateUnconstrainedMotionPlan(std::vector<double> x,
                                        std::vector<std::vector<double>> x0,
                                        std::vector<double> u,
                                        std::vector<std::vector<double>> u0,
                                        uint max_iter);

    int generateConstrainedMotionPlan(std::vector<double> x,
                                      std::vector<std::vector<double>> x0,
                                      std::vector<double> u,
                                      std::vector<std::vector<double>> u0,
                                      uint max_iter);

    int generateSteppedMotionPlan(std::vector<double> x,
                                  std::vector<std::vector<double>> x0,
                                  std::vector<double> u,
                                  std::vector<std::vector<double>> u0);
    //**********//
    // Get Data //
    //**********//
    bool getPlannedState(std::vector<std::vector<double>> & x);

    bool getPlannedControl(std::vector<std::vector<double>> & u);
};
}    // namespace SOMP
#endif
