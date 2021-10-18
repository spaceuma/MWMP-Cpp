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

#include "FileManager.hpp"
#include "SOMP.hpp"
#include <Eigen/Dense>
#include <ctime>
#include <gtest/gtest.h>

#define nocolor "\033[0m"
#define black "\033[1;30m"
#define red "\033[1;31m"
#define green "\033[1;32m"
#define yellow "\033[1;33m"
#define blue "\033[1;34m"
#define magenta "\033[1;35m"
#define cyan "\033[1;36m"
#define white "\033[1;37m"

using namespace SOMP;

TEST(SOMP, constructors_test)
{
    StateSpaceModels::MobileManipulator * exoter_model =
        new StateSpaceModels::MobileManipulator("exoter");

    MotionPlanner * exoter_mp1 = new MotionPlanner(exoter_model);

    EXPECT_EQ(true, exoter_mp1->setTimeStep(1));

    SOMP::Config mp_config;
    mp_config.time_horizon = 160;
    mp_config.time_step = 1.0063;
    mp_config.max_iterations = 200;
    mp_config.control_threshold = 1e-3;
    mp_config.line_search_step = 0.30;
    mp_config.check_distance = true;
    mp_config.check_orientation = true;
    uint number_time_steps = 159;

    MotionPlanner * exoter_mp2 = new MotionPlanner(exoter_model, mp_config);

    EXPECT_EQ(true, exoter_mp2->setTimeHorizon(200));

    SOMP::MapInfo mp_map;
    mp_map.map_resolution = 0.05;
    FileManager::readMatrixFile("inputs/dummy_obstacles_map.txt", mp_map.obstacles_map);

    MotionPlanner * exoter_mp3 = new MotionPlanner(exoter_model, mp_config, mp_map);
}
