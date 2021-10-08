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

#include "StateSpaceModels.hpp"

#define pi 3.14159265359

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
 *     - If checking distance to goal, indexes of state vector where
 *       to check for the distance, "std::vector<uint> dist_indexes".
 *     - If checking distance to goal, acceptable distance to goal
 *       to consider convergence is reached, "double dist_threshold".
 *       Default: 0.005.
 *     - Yes/no about check final orientation, "bool check_orientation".
 *     - If checking final orientation, indexes of state vector where
 *       to check for the orientation, "std::vector<uint> orientation_indexes".
 *     - If checking final orientation, acceptable euclidean distance to
 *       the final orientation to consider convergence is reached,
 *       "double orientation_threshold". Default: 0.005.
 *
 * "SSModel state_space_model" should be an instance of the state space model to be used.
 *
 * "MapInfo map_info" is a struct only needed if checking safety, should contain:
 *     - Map resolution "double map_resolution".
 *     - Digital elevation map "std::vector<std::vector<uint>> dem".
 *     - Repulsive cost from obstacles "double obstacles_cost".
 *
 *********************************************************************************************/

class MotionPlanner
{
private:
    //********************//
    // Dependency classes //
    //********************//

    //*************************//
    // Configurable Parameters //
    //*************************//
    bool checkSafety;

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
    MotionPlannerSLQ();

    //***********************//
    // Configuring Functions //
    //***********************//
    bool updateTimeHorizon(double new_time_horizon);
    bool updateTimeStep(double new_time_step);

    //********************//
    // Planning Functions //
    //********************//
    /*********************************************************************************************
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
     * If convergence is reached, "int status" will be "1", if not:
     *     " 0" --> something unexpected happened.
     *     "-1" --> the goal is still far away.
     *     "-2" --> the algorithm is still hardly updating the control.
     *     "-3" --> the generated state is not safe.
     *     "-4" --> something unexpected happened.
     *
     *********************************************************************************************/

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
