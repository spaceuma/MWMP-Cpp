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

#include "SOMP.hpp"

using namespace SOMP;
using namespace StateSpaceModels;

bool MotionPlanner::updateHorizon(std::vector<std::vector<double>> & x,
                                  const std::vector<std::vector<double>> & u,
                                  std::vector<std::vector<std::vector<double>>> & Ah,
                                  std::vector<std::vector<std::vector<double>>> & Bh,
                                  std::vector<std::vector<std::vector<double>>> & Qh,
                                  std::vector<std::vector<std::vector<double>>> & Rh,
                                  std::vector<std::vector<std::vector<double>>> & Kh)
{
    if(!robot_ss_model->getLinearizedMatrixA(x[0], time_step, Ah[0]) ||
       !robot_ss_model->getLinearizedMatrixB(x[0], u[0], time_step, Bh[0]))
    {
        std::cout
            << red
            << "ERROR [MotionPlanner::updateHorizon]: Unable to linearize the state space model"
            << reset << std::endl;
        return false;
    }

    for(uint i = 1; i < number_time_steps; i++)
    {
        if(!robot_ss_model->forwardIntegrateModel(x[i - 1], u[i - 1], time_step, x[i]))
        {
            std::cout
                << red
                << "ERROR [MotionPlanner::updateHorizon]: Unable to forward integrate the model"
                << reset << std::endl;
            return false;
        }

        if(!robot_ss_model->getLinearizedMatrixA(x[i - 1], time_step, Ah[i]) ||
           !robot_ss_model->getLinearizedMatrixB(x[i - 1], u[i - 1], time_step, Bh[i]))
        {
            std::cout
                << red
                << "ERROR [MotionPlanner::updateHorizon]: Unable to linearize the state space model"
                << reset << std::endl;
            return false;
        }

        double percentage_horizon = 100 * (i + 1) / number_time_steps;
        if(!robot_ss_model->getStateCostMatrix(percentage_horizon, Qh[i]) ||
           !robot_ss_model->getInputCostMatrix(Rh[i]) ||
           !robot_ss_model->getStateInputCostMatrix(Kh[i]))
        {
            std::cout
                << red
                << "ERROR [MotionPlanner::updateHorizon]: Unable to compute the quadratized costs"
                << reset << std::endl;
            return false;
        }
    }

    return true;
}

bool MotionPlanner::updateHorizonConstraints(std::vector<std::vector<std::vector<double>>> & Ch,
                                             std::vector<std::vector<std::vector<double>>> & Dh,
                                             std::vector<std::vector<double>> & rh,
                                             std::vector<std::vector<std::vector<double>>> & Gh,
                                             std::vector<std::vector<double>> & hh)
{
    for(uint i = 0; i < number_time_steps; i++)
    {
        if(!robot_ss_model->getConstraintsMatrixC(Ch[i]) ||
           !robot_ss_model->getConstraintsMatrixD(Dh[i]) ||
           !robot_ss_model->getConstraintsMatrixR(rh[i]))
        {
            std::cout << red
                      << "ERROR [MotionPlanner::updateHorizonConstraints]: Unable to compute the "
                         "state input constraints"
                      << reset << std::endl;
            return false;
        }
        if(!robot_ss_model->getConstraintsMatrixG(Gh[i]) ||
           !robot_ss_model->getConstraintsMatrixH(hh[i]))
        {
            std::cout << red
                      << "ERROR [MotionPlanner::updateHorizonConstraints]: Unable to compute the "
                         "pure state constraints"
                      << reset << std::endl;
            return false;
        }
    }

    return true;
}

MotionPlanner::MotionPlanner(MobileManipulator * _robot_ss_model)
{
    robot_ss_model = _robot_ss_model;

    distance_indexes = robot_ss_model->getIndexesGoalDistance();
    distance_threshold = robot_ss_model->getThresholdGoalDistance();

    orientation_indexes = robot_ss_model->getIndexesGoalOrientation();
    orientation_threshold = robot_ss_model->getThresholdGoalOrientation();

    check_safety = false;
    map_resolution = 0.05;

    FileManager::readMatrixFile("dummy_obstacles_map.txt", obstacles_map);

    number_states = robot_ss_model->getNumberStates();
    number_inputs = robot_ss_model->getNumberInputs();
    number_si_constraints = robot_ss_model->getNumberStateInputConstraints();
    number_ps_constraints = robot_ss_model->getNumberPureStateConstraints();
}

MotionPlanner::MotionPlanner(MobileManipulator * _robot_ss_model, Config config)
{
    robot_ss_model = _robot_ss_model;

    time_horizon = config.time_horizon;
    time_step = config.time_step;

    number_time_steps = (uint)time_horizon / time_step;

    max_iterations = config.max_iterations;
    control_threshold = config.control_threshold;
    line_search_step = config.line_search_step;

    check_distance = config.check_distance;
    distance_indexes = robot_ss_model->getIndexesGoalDistance();
    distance_threshold = robot_ss_model->getThresholdGoalDistance();

    check_orientation = config.check_orientation;
    orientation_indexes = robot_ss_model->getIndexesGoalOrientation();
    orientation_threshold = robot_ss_model->getThresholdGoalOrientation();

    check_safety = false;
    map_resolution = 0.05;

    FileManager::readMatrixFile("dummy_obstacles_map.txt", obstacles_map);

    number_states = robot_ss_model->getNumberStates();
    number_inputs = robot_ss_model->getNumberInputs();
    number_si_constraints = robot_ss_model->getNumberStateInputConstraints();
    number_ps_constraints = robot_ss_model->getNumberPureStateConstraints();
}

MotionPlanner::MotionPlanner(MobileManipulator * _robot_ss_model, Config config, MapInfo map_info)
{
    robot_ss_model = _robot_ss_model;

    time_horizon = config.time_horizon;
    time_step = config.time_step;

    number_time_steps = (uint)time_horizon / time_step;

    max_iterations = config.max_iterations;
    control_threshold = config.control_threshold;
    line_search_step = config.line_search_step;

    check_distance = config.check_distance;
    distance_indexes = robot_ss_model->getIndexesGoalDistance();
    distance_threshold = robot_ss_model->getThresholdGoalDistance();

    check_orientation = config.check_orientation;
    orientation_indexes = robot_ss_model->getIndexesGoalOrientation();
    orientation_threshold = robot_ss_model->getThresholdGoalOrientation();

    check_safety = true;
    map_resolution = map_info.map_resolution;
    obstacles_map = map_info.obstacles_map;

    number_states = robot_ss_model->getNumberStates();
    number_inputs = robot_ss_model->getNumberInputs();
    number_si_constraints = robot_ss_model->getNumberStateInputConstraints();
    number_ps_constraints = robot_ss_model->getNumberPureStateConstraints();
}

bool MotionPlanner::setTimeHorizon(double new_time_horizon)
{
    if(new_time_horizon > 0 && new_time_horizon > time_step)
        time_horizon = new_time_horizon;
    else
    {
        std::cout << magenta
                  << "WARNING [MotionPlanner::setTimeHorizon]: The new requested time horizon is "
                     "not valid, the previous one is maintained"
                  << reset << std::endl;
        return false;
    }
    return true;
}

bool MotionPlanner::setTimeStep(double new_time_step)
{
    if(new_time_step > 0 && new_time_step < time_horizon)
        time_step = new_time_step;
    else
    {
        std::cout << magenta
                  << "WARNING [MotionPlanner::setTimeStep]: The new requested time step is not "
                     "valid, the previous one is maintained"
                  << reset << std::endl;
        return false;
    }
    return true;
}

int MotionPlanner::generateUnconstrainedMotionPlan(std::vector<double> x,
                                                   std::vector<double> x0,
                                                   std::vector<double> u,
                                                   std::vector<double> u0)
{
    std::vector<std::vector<double>> xh(number_time_steps, std::vector<double>(number_states, 0));
    std::vector<std::vector<double>> uh(number_time_steps, std::vector<double>(number_inputs, 0));

    xh[0] = x;
    uh[0] = u;

    std::vector<std::vector<std::vector<double>>> Ah(
        number_time_steps,
        std::vector<std::vector<double>>(number_states, std::vector<double>(number_states, 0)));
    std::vector<std::vector<std::vector<double>>> Bh(
        number_time_steps,
        std::vector<std::vector<double>>(number_states, std::vector<double>(number_inputs, 0)));

    std::vector<std::vector<std::vector<double>>> Qh(
        number_time_steps,
        std::vector<std::vector<double>>(number_states, std::vector<double>(number_states, 0)));
    std::vector<std::vector<std::vector<double>>> Rh(
        number_time_steps,
        std::vector<std::vector<double>>(number_inputs, std::vector<double>(number_inputs, 0)));
    std::vector<std::vector<std::vector<double>>> Kh(
        number_time_steps,
        std::vector<std::vector<double>>(number_states, std::vector<double>(number_inputs, 0)));

    return 1;
}

int MotionPlanner::generateConstrainedMotionPlan(std::vector<double> x,
                                                 std::vector<double> x0,
                                                 std::vector<double> u,
                                                 std::vector<double> u0)
{
    std::vector<std::vector<double>> xh(number_time_steps, std::vector<double>(number_states, 0));
    std::vector<std::vector<double>> uh(number_time_steps, std::vector<double>(number_inputs, 0));

    xh[0] = x;
    uh[0] = u;

    std::vector<std::vector<std::vector<double>>> Ah(
        number_time_steps,
        std::vector<std::vector<double>>(number_states, std::vector<double>(number_states, 0)));
    std::vector<std::vector<std::vector<double>>> Bh(
        number_time_steps,
        std::vector<std::vector<double>>(number_states, std::vector<double>(number_inputs, 0)));

    std::vector<std::vector<std::vector<double>>> Qh(
        number_time_steps,
        std::vector<std::vector<double>>(number_states, std::vector<double>(number_states, 0)));
    std::vector<std::vector<std::vector<double>>> Rh(
        number_time_steps,
        std::vector<std::vector<double>>(number_inputs, std::vector<double>(number_inputs, 0)));
    std::vector<std::vector<std::vector<double>>> Kh(
        number_time_steps,
        std::vector<std::vector<double>>(number_states, std::vector<double>(number_inputs, 0)));

    std::vector<std::vector<std::vector<double>>> Ch(
        number_time_steps,
        std::vector<std::vector<double>>(number_si_constraints,
                                         std::vector<double>(number_states, 0)));
    std::vector<std::vector<std::vector<double>>> Dh(
        number_time_steps,
        std::vector<std::vector<double>>(number_si_constraints,
                                         std::vector<double>(number_inputs, 0)));
    std::vector<std::vector<double>> rh(number_time_steps,
                                        std::vector<double>(number_si_constraints, 0));

    std::vector<std::vector<std::vector<double>>> Gh(
        number_time_steps,
        std::vector<std::vector<double>>(number_ps_constraints,
                                         std::vector<double>(number_states, 0)));
    std::vector<std::vector<double>> hh(number_time_steps,
                                        std::vector<double>(number_ps_constraints, 0));

    if(!updateHorizonConstraints(Ch, Dh, rh, Gh, hh))
    {
        std::cout << red
                  << "ERROR [MotionPlanner::generateUnconstrainedMotionPlan]: Unable to initialize "
                     "the constraints"
                  << reset << std::endl;
        return false;
    }

    return 1;
}

int MotionPlanner::generateSteppedMotionPlan(std::vector<double> x,
                                             std::vector<double> x0,
                                             std::vector<double> u,
                                             std::vector<double> u0)
{
    return 1;
}

bool MotionPlanner::getPlannedState(std::vector<std::vector<double>> & x)
{
    if(!is_motion_planned)
    {
        std::cout << red
                  << "ERROR [MotionPlanner::getPlannedState]: No motion plan has yet been generated"
                  << reset << std::endl;
        return false;
    }
    else
        x = planned_state;

    return true;
}

bool MotionPlanner::getPlannedControl(std::vector<std::vector<double>> & u)
{
    if(!is_motion_planned)
    {
        std::cout
            << red
            << "ERROR [MotionPlanner::getPlannedControl]: No motion plan has yet been generated"
            << reset << std::endl;
        return false;
    }
    else
        u = planned_control;

    return true;
}
