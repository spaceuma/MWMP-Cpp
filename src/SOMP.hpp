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
#include <vector>

#include "FileManager.hpp"
#include "StateSpaceModels.hpp"

#define pi 3.14159265359

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
 *     - Maximum number of iterations "int max_iterations". Default: 200.
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

    // Map resolution in meters, default 0.05m
    double map_resolution = 0.05;

    // Boolean map with obstacles as 0 and safe areas as 1, default 5x5m non obstacle map
    std::vector<std::vector<uint>> obstacles_map;

    // TODO what to do with obstacles_cost?

    //*******//
    // Flags //
    //*******//

    //*********************//
    // Motion Plan Results //
    //*********************//

    //**********************//
    // Supporting Functions //
    //**********************//

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
     * "std::vector<std::vector<double>> x, x0" are the initial
     * and goal states respectively. Size number_states x number_time_steps.
     *
     * "std::vector<std::vector<double>> u, u0" are the initial and goal
     * control inputs respectively (u0 is usually filled with zeros).
     * Size number_inputs x number_time_steps.
     *
     * If convergence is reached, this functions will return "1", if not:
     *     " 0" --> something unexpected happened.
     *     "-1" --> the goal is still far away.
     *     "-2" --> the algorithm is still hardly updating the control.
     *     "-3" --> the generated state is not safe.
     *     "-4" --> something unexpected happened.
     *
     ******************************************************************************************/

    int generateUnconstrainedMotionPlan(std::vector<std::vector<double>> x,
                                        std::vector<std::vector<double>> x0,
                                        std::vector<std::vector<double>> u,
                                        std::vector<std::vector<double>> u0);

    int generateConstrainedMotionPlan(std::vector<std::vector<double>> x,
                                      std::vector<std::vector<double>> x0,
                                      std::vector<std::vector<double>> u,
                                      std::vector<std::vector<double>> u0);

    int generateSteppedMotionPlan(std::vector<std::vector<double>> x,
                                  std::vector<std::vector<double>> x0,
                                  std::vector<std::vector<double>> u,
                                  std::vector<std::vector<double>> u0);
    //**********//
    // Get Data //
    //**********//
    std::vector<std::vector<double>> getPlannedState();
    std::vector<std::vector<double>> getPlannedControl();

    //********************//
    // Checking Functions //
    //********************//
};
}    // namespace SOMP
#endif
