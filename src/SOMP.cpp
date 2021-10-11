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
}

MotionPlanner::MotionPlanner(MobileManipulator * _robot_ss_model, Config config)
{
    robot_ss_model = _robot_ss_model;

    time_horizon = config.time_horizon;
    time_step = config.time_step;

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
}

MotionPlanner::MotionPlanner(MobileManipulator * _robot_ss_model, Config config, MapInfo map_info)
{
    robot_ss_model = _robot_ss_model;

    time_horizon = config.time_horizon;
    time_step = config.time_step;

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
