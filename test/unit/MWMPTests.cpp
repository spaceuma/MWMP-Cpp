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
#include "MWMP.hpp"
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

using namespace MWMP;

TEST(MWMP, constructors_test)
{
    StateSpaceModels::MobileManipulator * robot_model =
        new StateSpaceModels::MobileManipulator("exoter_ack");

    MotionPlanner * robot_mp1 = new MotionPlanner(robot_model);

    EXPECT_EQ(true, robot_mp1->setTimeStep(1));

    MWMP::Config mp_config;
    mp_config.time_horizon = 160;
    mp_config.time_step = 1.0063;
    mp_config.max_iterations = 200;
    mp_config.control_threshold = 1e-3;
    mp_config.line_search_step = 0.32;
    mp_config.check_distance = true;
    mp_config.check_orientation = true;
    mp_config.track_reference_trajectory = true;
    uint number_time_steps = 159;

    MotionPlanner * robot_mp2 = new MotionPlanner(robot_model, mp_config);

    EXPECT_EQ(true, robot_mp2->setTimeHorizon(200));

    MWMP::MapInfo mp_map;
    mp_map.map_resolution = 0.05;
    std::vector<double> goal_pose = {3.1, 2.8};
    mp_map.goal_pose = goal_pose;
    FileManager::readMatrixFile("inputs/dummy_obstacles_map.txt", mp_map.obstacles_map);

    MotionPlanner * robot_mp3 = new MotionPlanner(robot_model, mp_config, mp_map);

    delete(robot_model);
    delete(robot_mp1);
    delete(robot_mp2);
    delete(robot_mp3);
}

TEST(MWMP, unconstrained_mp_test)
{
    StateSpaceModels::MobileManipulator * robot_model =
        new StateSpaceModels::MobileManipulator("exoter_ack");

    MWMP::Config mp_config;
    mp_config.time_horizon = 160;
    mp_config.time_step = 1.006289308;
    mp_config.max_iterations = 100;
    mp_config.control_threshold = 5e-2;
    mp_config.line_search_step = 0.32;
    mp_config.check_distance = true;
    mp_config.check_orientation = true;
    mp_config.track_reference_trajectory = false;
    uint number_time_steps = (uint)(mp_config.time_horizon / mp_config.time_step) + 1;

    MWMP::MapInfo mp_map;
    mp_map.map_resolution = 0.05;

    std::vector<double> goal_pose;
    std::vector<double> goal_ee_pose;
    FileManager::readVectorFile("inputs/goal_ee_pose.txt", goal_ee_pose);
    goal_pose.push_back(goal_ee_pose[0]);
    goal_pose.push_back(goal_ee_pose[1]);
    mp_map.goal_pose = goal_pose;

    FileManager::readMatrixFile("inputs/dummy_obstacles_map.txt", mp_map.obstacles_map);

    MotionPlanner * robot_mp = new MotionPlanner(robot_model, mp_config, mp_map);

    std::vector<double> ini_rover_pose;
    FileManager::readVectorFile("inputs/ini_rover_pose.txt", ini_rover_pose);
    std::vector<double> ini_arm_positions;
    FileManager::readVectorFile("inputs/ini_arm_positions.txt", ini_arm_positions);
    std::vector<double> ini_arm_speeds;
    FileManager::readVectorFile("inputs/ini_arm_speeds.txt", ini_arm_speeds);
    std::vector<double> ini_wheels_speed;
    FileManager::readVectorFile("inputs/ini_wheels_speed.txt", ini_wheels_speed);
    std::vector<double> ini_wheels_steering = {0};

    Eigen::VectorXd x_ini =
        robot_model->getInitialStateVectorEigen(ini_rover_pose, ini_arm_positions);
    std::vector<Eigen::VectorXd> u_ini{number_time_steps,
                                       Eigen::VectorXd::Zero(robot_model->getNumberInputs())};
    u_ini[0] =
        robot_model->getInputVectorEigen(ini_arm_speeds, ini_wheels_speed, ini_wheels_steering);

    std::vector<Eigen::VectorXd> x0(number_time_steps,
                                    Eigen::VectorXd::Zero(robot_model->getNumberStates()));
    std::vector<Eigen::VectorXd> u0(number_time_steps,
                                    Eigen::VectorXd::Zero(robot_model->getNumberInputs()));


    x0[number_time_steps - 1] = robot_model->getGoalStateVectorEigen(goal_ee_pose);

    double ini_time = clock();
    EXPECT_EQ(1, robot_mp->generateUnconstrainedMotionPlan(x_ini, x0, u_ini, u0, 100));
    std::cout << cyan << "[MWMP::unconstrained_mp_test] Elapsed time: "
              << (double)(clock() - ini_time) / CLOCKS_PER_SEC << " s" << nocolor << std::endl;

    std::vector<Eigen::VectorXd> x;
    std::vector<Eigen::VectorXd> u;

    robot_mp->getPlannedState(x);
    robot_mp->getPlannedControl(u);

    FileManager::writeMatrixFile("results/unconstrained_planned_state.txt", x);
    FileManager::writeMatrixFile("results/unconstrained_planned_control.txt", u);

    delete(robot_model);
    delete(robot_mp);
}

TEST(MWMP, ws_unconstrained_mp_test)
{
    StateSpaceModels::MobileManipulator * robot_model =
        new StateSpaceModels::MobileManipulator("exoter_ack");

    MWMP::Config mp_config;
    mp_config.time_horizon = 160;
    mp_config.time_step = 1.006289308;
    mp_config.max_iterations = 100;
    mp_config.control_threshold = 5e-2;
    mp_config.line_search_step = 0.32;
    mp_config.check_distance = true;
    mp_config.check_orientation = true;
    mp_config.track_reference_trajectory = true;
    uint number_time_steps = (uint)(mp_config.time_horizon / mp_config.time_step) + 1;

    MWMP::MapInfo mp_map;
    mp_map.map_resolution = 0.05;

    std::vector<double> goal_pose;
    std::vector<double> goal_ee_pose;
    FileManager::readVectorFile("inputs/goal_ee_pose.txt", goal_ee_pose);
    goal_pose.push_back(goal_ee_pose[0]);
    goal_pose.push_back(goal_ee_pose[1]);
    mp_map.goal_pose = goal_pose;

    FileManager::readMatrixFile("inputs/dummy_obstacles_map.txt", mp_map.obstacles_map);

    MotionPlanner * robot_mp = new MotionPlanner(robot_model, mp_config, mp_map);

    std::vector<double> ini_rover_pose;
    FileManager::readVectorFile("inputs/ini_rover_pose.txt", ini_rover_pose);
    std::vector<double> ini_arm_positions;
    FileManager::readVectorFile("inputs/ini_arm_positions.txt", ini_arm_positions);
    std::vector<double> ini_arm_speeds;
    FileManager::readVectorFile("inputs/ini_arm_speeds.txt", ini_arm_speeds);
    std::vector<double> ini_wheels_speed;
    FileManager::readVectorFile("inputs/ini_wheels_speed.txt", ini_wheels_speed);
    std::vector<double> ini_wheels_steering = {0};

    Eigen::VectorXd x_ini =
        robot_model->getInitialStateVectorEigen(ini_rover_pose, ini_arm_positions);
    std::vector<Eigen::VectorXd> u_ini{number_time_steps,
                                       Eigen::VectorXd::Zero(robot_model->getNumberInputs())};
    u_ini[0] =
        robot_model->getInputVectorEigen(ini_arm_speeds, ini_wheels_speed, ini_wheels_steering);

    std::vector<Eigen::VectorXd> x0(number_time_steps,
                                    Eigen::VectorXd::Zero(robot_model->getNumberStates()));
    std::vector<Eigen::VectorXd> u0(number_time_steps,
                                    Eigen::VectorXd::Zero(robot_model->getNumberInputs()));


    x0[number_time_steps - 1] = robot_model->getGoalStateVectorEigen(goal_ee_pose);

    double ini_time = clock();
    EXPECT_EQ(1, robot_mp->generateUnconstrainedMotionPlan(x_ini, x0, u_ini, u0, 100));
    std::cout << cyan << "[MWMP::ws_unconstrained_mp_test] Elapsed time: "
              << (double)(clock() - ini_time) / CLOCKS_PER_SEC << " s" << nocolor << std::endl;

    std::vector<Eigen::VectorXd> x;
    std::vector<Eigen::VectorXd> u;

    robot_mp->getPlannedState(x);
    robot_mp->getPlannedControl(u);

    FileManager::writeMatrixFile("results/ws_unconstrained_planned_state.txt", x);
    FileManager::writeMatrixFile("results/ws_unconstrained_planned_control.txt", u);

    delete(robot_model);
    delete(robot_mp);
}

TEST(MWMP, constrained_mp_test)
{
    StateSpaceModels::MobileManipulator * robot_model =
        new StateSpaceModels::MobileManipulator("exoter_ack");

    MWMP::Config mp_config;
    mp_config.time_horizon = 160;
    mp_config.time_step = 1.006289308;
    mp_config.max_iterations = 200;
    mp_config.control_threshold = 5e-2;
    mp_config.line_search_step = 0.32;
    mp_config.check_distance = true;
    mp_config.check_orientation = true;
    mp_config.track_reference_trajectory = false;
    uint number_time_steps = (uint)(mp_config.time_horizon / mp_config.time_step) + 1;

    MWMP::MapInfo mp_map;
    mp_map.map_resolution = 0.05;

    std::vector<double> goal_pose;
    std::vector<double> goal_ee_pose;
    FileManager::readVectorFile("inputs/goal_ee_pose.txt", goal_ee_pose);
    goal_pose.push_back(goal_ee_pose[0]);
    goal_pose.push_back(goal_ee_pose[1]);
    mp_map.goal_pose = goal_pose;

    FileManager::readMatrixFile("inputs/dummy_obstacles_map.txt", mp_map.obstacles_map);

    MotionPlanner * robot_mp = new MotionPlanner(robot_model, mp_config, mp_map);

    std::vector<double> ini_rover_pose;
    FileManager::readVectorFile("inputs/ini_rover_pose.txt", ini_rover_pose);
    std::vector<double> ini_arm_positions;
    FileManager::readVectorFile("inputs/ini_arm_positions.txt", ini_arm_positions);
    std::vector<double> ini_arm_speeds;
    FileManager::readVectorFile("inputs/ini_arm_speeds.txt", ini_arm_speeds);
    std::vector<double> ini_wheels_speed;
    FileManager::readVectorFile("inputs/ini_wheels_speed.txt", ini_wheels_speed);
    std::vector<double> ini_wheels_steering = {0};

    Eigen::VectorXd x_ini =
        robot_model->getInitialStateVectorEigen(ini_rover_pose, ini_arm_positions);
    std::vector<Eigen::VectorXd> u_ini{number_time_steps,
                                       Eigen::VectorXd::Zero(robot_model->getNumberInputs())};
    u_ini[0] =
        robot_model->getInputVectorEigen(ini_arm_speeds, ini_wheels_speed, ini_wheels_steering);

    std::vector<Eigen::VectorXd> x0(number_time_steps,
                                    Eigen::VectorXd::Zero(robot_model->getNumberStates()));
    std::vector<Eigen::VectorXd> u0(number_time_steps,
                                    Eigen::VectorXd::Zero(robot_model->getNumberInputs()));

    std::vector<Eigen::VectorXd> xs(number_time_steps,
                                    Eigen::VectorXd::Zero(robot_model->getNumberStates()));
    std::vector<Eigen::VectorXd> us(number_time_steps,
                                    Eigen::VectorXd::Zero(robot_model->getNumberInputs()));

    x0[number_time_steps - 1] = robot_model->getGoalStateVectorEigen(goal_ee_pose);

    double ini_time = clock();
    EXPECT_EQ(1, robot_mp->generateConstrainedMotionPlan(x_ini, x0, xs, u_ini, u0, us, 100));
    std::cout << cyan << "[MWMP::constrained_mp_test] Elapsed time: "
              << (double)(clock() - ini_time) / CLOCKS_PER_SEC << " s" << nocolor << std::endl;

    std::vector<Eigen::VectorXd> x;
    std::vector<Eigen::VectorXd> u;
    robot_mp->getPlannedState(x);
    robot_mp->getPlannedControl(u);

    FileManager::writeMatrixFile("results/constrained_planned_state.txt", x);
    FileManager::writeMatrixFile("results/constrained_planned_control.txt", u);

    delete(robot_model);
    delete(robot_mp);
}

TEST(MWMP, ws_constrained_mp_test)
{
    StateSpaceModels::MobileManipulator * robot_model =
        new StateSpaceModels::MobileManipulator("exoter_ack");

    MWMP::Config mp_config;
    mp_config.time_horizon = 160;
    mp_config.time_step = 1.006289308;
    mp_config.max_iterations = 200;
    mp_config.control_threshold = 5e-2;
    mp_config.line_search_step = 0.32;
    mp_config.check_distance = true;
    mp_config.check_orientation = true;
    mp_config.track_reference_trajectory = true;
    uint number_time_steps = (uint)(mp_config.time_horizon / mp_config.time_step) + 1;

    MWMP::MapInfo mp_map;
    mp_map.map_resolution = 0.05;

    std::vector<double> goal_pose;
    std::vector<double> goal_ee_pose;
    FileManager::readVectorFile("inputs/goal_ee_pose.txt", goal_ee_pose);
    goal_pose.push_back(goal_ee_pose[0]);
    goal_pose.push_back(goal_ee_pose[1]);
    mp_map.goal_pose = goal_pose;

    FileManager::readMatrixFile("inputs/dummy_obstacles_map.txt", mp_map.obstacles_map);

    MotionPlanner * robot_mp = new MotionPlanner(robot_model, mp_config, mp_map);

    std::vector<double> ini_rover_pose;
    FileManager::readVectorFile("inputs/ini_rover_pose.txt", ini_rover_pose);
    std::vector<double> ini_arm_positions;
    FileManager::readVectorFile("inputs/ini_arm_positions.txt", ini_arm_positions);
    std::vector<double> ini_arm_speeds;
    FileManager::readVectorFile("inputs/ini_arm_speeds.txt", ini_arm_speeds);
    std::vector<double> ini_wheels_speed;
    FileManager::readVectorFile("inputs/ini_wheels_speed.txt", ini_wheels_speed);
    std::vector<double> ini_wheels_steering = {0};

    Eigen::VectorXd x_ini =
        robot_model->getInitialStateVectorEigen(ini_rover_pose, ini_arm_positions);
    std::vector<Eigen::VectorXd> u_ini{number_time_steps,
                                       Eigen::VectorXd::Zero(robot_model->getNumberInputs())};
    u_ini[0] =
        robot_model->getInputVectorEigen(ini_arm_speeds, ini_wheels_speed, ini_wheels_steering);

    std::vector<Eigen::VectorXd> x0(number_time_steps,
                                    Eigen::VectorXd::Zero(robot_model->getNumberStates()));
    std::vector<Eigen::VectorXd> u0(number_time_steps,
                                    Eigen::VectorXd::Zero(robot_model->getNumberInputs()));

    std::vector<Eigen::VectorXd> xs(number_time_steps,
                                    Eigen::VectorXd::Zero(robot_model->getNumberStates()));
    std::vector<Eigen::VectorXd> us(number_time_steps,
                                    Eigen::VectorXd::Zero(robot_model->getNumberInputs()));

    x0[number_time_steps - 1] = robot_model->getGoalStateVectorEigen(goal_ee_pose);

    double ini_time = clock();
    EXPECT_EQ(1, robot_mp->generateConstrainedMotionPlan(x_ini, x0, xs, u_ini, u0, us, 100));
    std::cout << cyan << "[MWMP::ws_constrained_mp_test] Elapsed time: "
              << (double)(clock() - ini_time) / CLOCKS_PER_SEC << " s" << nocolor << std::endl;

    std::vector<Eigen::VectorXd> x;
    std::vector<Eigen::VectorXd> u;
    robot_mp->getPlannedState(x);
    robot_mp->getPlannedControl(u);

    FileManager::writeMatrixFile("results/ws_constrained_planned_state.txt", x);
    FileManager::writeMatrixFile("results/ws_constrained_planned_control.txt", u);

    delete(robot_model);
    delete(robot_mp);
}

TEST(MWMP, unc_constrained_mp_test)
{
    StateSpaceModels::MobileManipulator * robot_model =
        new StateSpaceModels::MobileManipulator("exoter_ack");

    MWMP::Config mp_config;
    mp_config.time_horizon = 160;
    mp_config.time_step = 1.006289308;
    mp_config.max_iterations = 200;
    mp_config.control_threshold = 5e-2;
    mp_config.line_search_step = 0.32;
    mp_config.check_distance = true;
    mp_config.check_orientation = true;
    mp_config.track_reference_trajectory = false;
    uint number_time_steps = (uint)(mp_config.time_horizon / mp_config.time_step) + 1;

    MWMP::MapInfo mp_map;
    mp_map.map_resolution = 0.05;

    std::vector<double> goal_pose;
    std::vector<double> goal_ee_pose;
    FileManager::readVectorFile("inputs/goal_ee_pose.txt", goal_ee_pose);
    goal_pose.push_back(goal_ee_pose[0]);
    goal_pose.push_back(goal_ee_pose[1]);
    mp_map.goal_pose = goal_pose;

    FileManager::readMatrixFile("inputs/dummy_obstacles_map.txt", mp_map.obstacles_map);

    MotionPlanner * robot_mp = new MotionPlanner(robot_model, mp_config, mp_map);

    std::vector<double> ini_rover_pose;
    FileManager::readVectorFile("inputs/ini_rover_pose.txt", ini_rover_pose);
    std::vector<double> ini_arm_positions;
    FileManager::readVectorFile("inputs/ini_arm_positions.txt", ini_arm_positions);
    std::vector<double> ini_arm_speeds;
    FileManager::readVectorFile("inputs/ini_arm_speeds.txt", ini_arm_speeds);
    std::vector<double> ini_wheels_speed;
    FileManager::readVectorFile("inputs/ini_wheels_speed.txt", ini_wheels_speed);
    std::vector<double> ini_wheels_steering = {0};

    Eigen::VectorXd x_ini =
        robot_model->getInitialStateVectorEigen(ini_rover_pose, ini_arm_positions);
    std::vector<Eigen::VectorXd> u_ini{number_time_steps,
                                       Eigen::VectorXd::Zero(robot_model->getNumberInputs())};
    u_ini[0] =
        robot_model->getInputVectorEigen(ini_arm_speeds, ini_wheels_speed, ini_wheels_steering);

    std::vector<Eigen::VectorXd> x0(number_time_steps,
                                    Eigen::VectorXd::Zero(robot_model->getNumberStates()));
    std::vector<Eigen::VectorXd> u0(number_time_steps,
                                    Eigen::VectorXd::Zero(robot_model->getNumberInputs()));

    x0[number_time_steps - 1] = robot_model->getGoalStateVectorEigen(goal_ee_pose);

    double ini_time = clock();
    EXPECT_EQ(1, robot_mp->generateSteppedMotionPlan(x_ini, x0, u_ini, u0));
    std::cout << cyan << "[MWMP::unc_constrained_mp_test] Elapsed time: "
              << (double)(clock() - ini_time) / CLOCKS_PER_SEC << " s" << nocolor << std::endl;

    std::vector<Eigen::VectorXd> x;
    std::vector<Eigen::VectorXd> u;
    robot_mp->getPlannedState(x);
    robot_mp->getPlannedControl(u);

    FileManager::writeMatrixFile("results/unc_constrained_planned_state.txt", x);
    FileManager::writeMatrixFile("results/unc_constrained_planned_control.txt", u);

    delete(robot_model);
    delete(robot_mp);
}

TEST(MWMP, mwmp_mp_test)
{
    StateSpaceModels::MobileManipulator * robot_model =
        new StateSpaceModels::MobileManipulator("exoter_ack");

    MWMP::Config mp_config;
    mp_config.time_horizon = 160;
    mp_config.time_step = 1.006289308;
    mp_config.max_iterations = 200;
    mp_config.control_threshold = 5e-2;
    mp_config.line_search_step = 0.32;
    mp_config.check_distance = true;
    mp_config.check_orientation = true;
    mp_config.track_reference_trajectory = true;
    uint number_time_steps = (uint)(mp_config.time_horizon / mp_config.time_step) + 1;

    MWMP::MapInfo mp_map;
    mp_map.map_resolution = 0.05;

    std::vector<double> goal_pose;
    std::vector<double> goal_ee_pose;
    FileManager::readVectorFile("inputs/goal_ee_pose.txt", goal_ee_pose);
    goal_pose.push_back(goal_ee_pose[0]);
    goal_pose.push_back(goal_ee_pose[1]);
    mp_map.goal_pose = goal_pose;

    FileManager::readMatrixFile("inputs/dummy_obstacles_map.txt", mp_map.obstacles_map);

    MotionPlanner * robot_mp = new MotionPlanner(robot_model, mp_config, mp_map);

    std::vector<double> ini_rover_pose;
    FileManager::readVectorFile("inputs/ini_rover_pose.txt", ini_rover_pose);
    std::vector<double> ini_arm_positions;
    FileManager::readVectorFile("inputs/ini_arm_positions.txt", ini_arm_positions);
    std::vector<double> ini_arm_speeds;
    FileManager::readVectorFile("inputs/ini_arm_speeds.txt", ini_arm_speeds);
    std::vector<double> ini_wheels_speed;
    FileManager::readVectorFile("inputs/ini_wheels_speed.txt", ini_wheels_speed);
    std::vector<double> ini_wheels_steering = {0};

    Eigen::VectorXd x_ini =
        robot_model->getInitialStateVectorEigen(ini_rover_pose, ini_arm_positions);
    std::vector<Eigen::VectorXd> u_ini{number_time_steps,
                                       Eigen::VectorXd::Zero(robot_model->getNumberInputs())};
    u_ini[0] =
        robot_model->getInputVectorEigen(ini_arm_speeds, ini_wheels_speed, ini_wheels_steering);

    std::vector<Eigen::VectorXd> x0(number_time_steps,
                                    Eigen::VectorXd::Zero(robot_model->getNumberStates()));
    std::vector<Eigen::VectorXd> u0(number_time_steps,
                                    Eigen::VectorXd::Zero(robot_model->getNumberInputs()));

    x0[number_time_steps - 1] = robot_model->getGoalStateVectorEigen(goal_ee_pose);

    double ini_time = clock();
    EXPECT_EQ(1, robot_mp->generateSteppedMotionPlan(x_ini, x0, u_ini, u0));
    std::cout << cyan << "[MWMP::mwmp_mp_test] Elapsed time: "
              << (double)(clock() - ini_time) / CLOCKS_PER_SEC << " s" << nocolor << std::endl;

    std::vector<Eigen::VectorXd> x;
    std::vector<Eigen::VectorXd> u;
    robot_mp->getPlannedState(x);
    robot_mp->getPlannedControl(u);

    FileManager::writeMatrixFile("results/mwmp_planned_state.txt", x);
    FileManager::writeMatrixFile("results/mwmp_planned_control.txt", u);

    delete(robot_model);
    delete(robot_mp);
}
