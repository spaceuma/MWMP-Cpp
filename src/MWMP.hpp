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
// Supervisor: Carlos J. Pe©rez del Pulgar
// Affiliation: Department of Systems Engineering and Automation
// Space Robotics Lab (www.uma.es/space-robotics)

#ifndef __MOTION_PLANNER_MWMP__
#define __MOTION_PLANNER_MWMP__

#include <Eigen/Dense>
#include <exception>
#include <opencv2/opencv.hpp>
#include <vector>

#include "FastMarching.hpp"
#include "StateSpaceModels.hpp"

#define inf 1000000007

#define NOCOLOR "\033[0m"
#define BLACK "\033[1;30m"
#define RED "\033[1;31m"
#define GREEN "\033[1;32m"
#define YELLOW "\033[1;33m"
#define BLUE "\033[1;34m"
#define MAGENTA "\033[1;35m"
#define CYAN "\033[1;36m"
#define WHITE "\033[1;37m"

namespace MWMP
{
/*********************************************************************************************
 *
 *  MWMP: Stepped Optimal Motion Planner, based on unconstrained and constrained SLQr.
 *  Given the system modelled by "state_space_model",
 *  and the configurations given in "config", the SLQR motion planning
 *  problem is solved, trying to reach the objective given by "x0" and "u0"
 *  from the current state "x" and control "u".
 *  The results can be obtained in the matrices "x" and "u".
 *  Info about the map can be provided through "map_info" parameter.
 *
 * CONSTRUCTORS:
 * MWMP::MotionPlanner * somp_planner = new MWMP::MotionPlanner(Config config,
 *                                                              SSModel state_space_model);
 * MWMP::MotionPlanner * somp_planner = new MWMP::MotionPlanner(Config config,
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
 *     - Yes/no about tracking a reference trajectory, "bool track_reference_trajectory".
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

    bool track_reference_trajectory;
};

struct MapInfo
{
    double map_resolution;
    std::vector<std::vector<uint>> obstacles_map;
    std::vector<double> goal_pose;
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

    // Discretization step for the time horizon, default 1.0063 seconds
    double time_step = 1.006289308;

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

    // Decide whether to track a reference trajectory for the robot base or not, default false
    bool track_reference_trajectory = false;

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

    // Cost map of the scenario, to be used to compute reference trajectories
    std::vector<std::vector<double>> cost_map;

    // Gradient of the obstacles map in X and Y directions, used for computing the cost
    // of the robot pose with respect to nearby obstacles
    std::vector<std::vector<double>> gradient_obstacles_map_x;
    std::vector<std::vector<double>> gradient_obstacles_map_y;

    //*********************//
    // Motion Plan Results //
    //*********************//

    // Resulting state vector (x). Size: number_states x number_time_steps
    std::vector<Eigen::VectorXd> planned_state;

    // Resulting input vector. Size: number_inputs x number_time_steps
    std::vector<Eigen::VectorXd> planned_control;

    //**********************//
    // Supporting variables //
    //**********************//

    // Flag to check if there is already a planned motion
    bool is_motion_planned = false;

    // Number of time steps, default 160 (160/1.006289308 + 1)
    uint number_time_steps = 160;

    // State space model properties
    uint number_states;
    uint number_inputs;
    uint number_si_constraints;
    uint number_ps_constraints;

    // Constraint time horizon matrixes
    std::vector<Eigen::MatrixXd> Q_hor;
    std::vector<Eigen::MatrixXd> R_hor;
    std::vector<Eigen::MatrixXd> K_hor;

    // Constraint time horizon matrixes
    std::vector<Eigen::MatrixXd> C_hor;
    std::vector<Eigen::MatrixXd> D_hor;
    std::vector<Eigen::VectorXd> r_hor;
    std::vector<Eigen::MatrixXd> G_hor;
    std::vector<Eigen::VectorXd> h_hor;

    //**********************//
    // Supporting functions //
    //**********************//

    // Get the complete time-horizon state and ss linearized matrixes
    bool generateHorizonLinearization(std::vector<Eigen::VectorXd> & x,
                                      const std::vector<Eigen::VectorXd> & u,
                                      std::vector<Eigen::MatrixXd> & A_hor_output,
                                      std::vector<Eigen::MatrixXd> & B_hor_output);

    bool generateHorizonLinearization(std::vector<std::vector<double>> & x,
                                      const std::vector<std::vector<double>> & u,
                                      std::vector<std::vector<std::vector<double>>> & A_hor_output,
                                      std::vector<std::vector<std::vector<double>>> & B_hor_output);

    // Get the complete time-horizon cost matrixes
    bool generateHorizonCosts(std::vector<Eigen::MatrixXd> & Q_hor_output,
                              std::vector<Eigen::MatrixXd> & R_hor_output,
                              std::vector<Eigen::MatrixXd> & K_hor_output);

    bool generateHorizonCosts(std::vector<std::vector<std::vector<double>>> & Q_hor_output,
                              std::vector<std::vector<std::vector<double>>> & R_hor_output,
                              std::vector<std::vector<std::vector<double>>> & K_hor_output);

    // Update the linearized matrixes A and B throughout the whole time horizon
    bool updateLinearModel(const std::vector<Eigen::VectorXd> & x,
                           const std::vector<Eigen::VectorXd> & u,
                           std::vector<Eigen::MatrixXd> & A_hor_output,
                           std::vector<Eigen::MatrixXd> & B_hor_output);

    bool updateLinearModel(const std::vector<std::vector<double>> & x,
                           const std::vector<std::vector<double>> & u,
                           std::vector<std::vector<std::vector<double>>> & A_hor_output,
                           std::vector<std::vector<std::vector<double>>> & B_hor_output);

    // Get the complete time-horizon constraint matrixes
    bool generateHorizonConstraints(std::vector<Eigen::MatrixXd> & C_hor_output,
                                    std::vector<Eigen::MatrixXd> & D_hor_output,
                                    std::vector<Eigen::VectorXd> & r_hor_output,
                                    std::vector<Eigen::MatrixXd> & G_hor_output,
                                    std::vector<Eigen::VectorXd> & h_hor_output);

    bool generateHorizonConstraints(std::vector<std::vector<std::vector<double>>> & C_hor_output,
                                    std::vector<std::vector<std::vector<double>>> & D_hor_output,
                                    std::vector<std::vector<double>> & r_hor_output,
                                    std::vector<std::vector<std::vector<double>>> & G_hor_output,
                                    std::vector<std::vector<double>> & h_hor_output);

    // Get the gradient of the obstacles map
    bool computeObstaclesGradient(const std::vector<std::vector<uint>> & obst_map);

    // Get the cost map for the path planner
    bool computeCostMap(const std::vector<std::vector<uint>> & obst_map,
                        const std::vector<std::vector<uint>> & safe_obst_map);

    // Dilate a binary obstacles map to ensure safety
    bool dilateObstaclesMap(const std::vector<std::vector<uint>> & obst_map,
                            double dilatation_distance,
                            std::vector<std::vector<uint>> & dilated_map);

    // Compute the line search procedure, trying to find the best way to apply
    // the computed state and control steps (xh and uh),if convergence is not
    // reached yet, decreasing the intensity of the actuation step
    bool computeLineSearch(std::vector<Eigen::VectorXd> & x,
                           const std::vector<Eigen::VectorXd> & x0,
                           std::vector<Eigen::VectorXd> & u,
                           const std::vector<Eigen::VectorXd> & u0,
                           const std::vector<Eigen::VectorXd> & uh);

    // Line search with a specified maximum actuation percentage
    bool computeLineSearch(std::vector<Eigen::VectorXd> & x,
                           const std::vector<Eigen::VectorXd> & x0,
                           std::vector<Eigen::VectorXd> & u,
                           const std::vector<Eigen::VectorXd> & u0,
                           const std::vector<Eigen::VectorXd> & uh,
                           double max_alfa,
                           double & final_alfa);

    // Compute the line search procedure, trying to find the best way to apply
    // the computed state and control steps (xh and uh),if convergence is not
    // reached yet, decreasing the intensity of the actuation step
    bool computeLineSearch(std::vector<std::vector<double>> & x,
                           const std::vector<std::vector<double>> & x0,
                           std::vector<std::vector<double>> & u,
                           const std::vector<std::vector<double>> & u0,
                           const std::vector<std::vector<double>> & uh,
                           const std::vector<std::vector<std::vector<double>>> & Q_hor,
                           const std::vector<std::vector<std::vector<double>>> & R_hor);

    bool checkConstraints(bool & constraints_satisfied);

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

    int generateUnconstrainedMotionPlan(const Eigen::VectorXd & x_t0,
                                        const std::vector<Eigen::VectorXd> & x0,
                                        const std::vector<Eigen::VectorXd> & u_ini,
                                        const std::vector<Eigen::VectorXd> & u0,
                                        uint max_iter);

    int generateConstrainedMotionPlan(const Eigen::VectorXd & x_t0,
                                      const std::vector<Eigen::VectorXd> & x0,
                                      const std::vector<Eigen::VectorXd> & xs,
                                      const std::vector<Eigen::VectorXd> & u_ini,
                                      const std::vector<Eigen::VectorXd> & u0,
                                      const std::vector<Eigen::VectorXd> & us,
                                      uint max_iter);

    int generateSteppedMotionPlan(const Eigen::VectorXd & x_t0,
                                  const std::vector<Eigen::VectorXd> & x0,
                                  const std::vector<Eigen::VectorXd> & u_ini,
                                  const std::vector<Eigen::VectorXd> & u0);
    //**********//
    // Get Data //
    //**********//
    bool getPlannedState(std::vector<Eigen::VectorXd> & x);

    bool getPlannedControl(std::vector<Eigen::VectorXd> & u);

    bool getPlannedTimestamps(std::vector<double> & t);

    double getTimeHorizon();

    double getTimeStep();

    uint getNumberTimeSteps();
};
}    // namespace MWMP
#endif
