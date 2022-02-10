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

#include "StateSpaceModels.hpp"

using namespace StateSpaceModels;
using namespace MatrixOperations;

MobileManipulator::MobileManipulator(std::string _robot_name)
{
    robot_name = _robot_name;

    if(robot_name == "exoter")
    {
        //******************//
        // Model properties //
        //******************//

        goal_distance_indexes = {0, 1, 2};
        goal_orientation_indexes = {3, 8, 9};

        world_ee_pose_indexes = {0, 1, 2, 3};
        base_ee_pose_indexes = {4, 5, 6, 7, 8, 9};

        robot_pose_indexes = {10, 11, 12};

        base_speed_indexes = {13, 14, 15};

        arm_position_indexes = {16, 17, 18, 19, 20};
        arm_actuators_indexes = {0, 1, 2, 3, 4};

        wheels_speed_indexes = {36, 37};
        wheels_actuators_indexes = {5, 6};

        state_constrained_indexes = {16, 17, 18, 19, 20, 40, 41};
        input_constrained_indexes = {0, 1, 2, 3, 4};

        goal_states_indexes = {0, 1, 2, 3, 8, 9};
        whole_states_indexes = {10, 11, 12, 31, 32, 33, 34, 35, 40, 41};

        whole_inputs_indexes = {0, 1, 2, 3, 4, 5, 6};

        //********************//
        // General parameters //
        //********************//
        rolling_resistance = 0.0036;
        gravity = 9.81;

        //******************//
        // Robot Parameters //
        //******************//
        differential_width = 0.3;
        differential_length = 0.27;

        robot_weight = 15;

        robot_height = 0.202;

        wheels_radius = 0.07;
        wheels_width = 0.09;
        wheels_weight = 0.484;

        nominal_speed = 0.10;

        state_limits = {-30 * pi / 180,
                        90 * pi / 180,
                        -190 * pi / 180,
                        -10 * pi / 180,
                        -160 * pi / 180,
                        160 * pi / 180,
                        -70 * pi / 180,
                        250 * pi / 180,
                        -160 * pi / 180,
                        160 * pi / 180,
                        -2.85,
                        2.85,
                        -2.85,
                        2.85};

        input_limits = {-0.01, 0.01, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01};

        arm_lengths = {0.0895, 0.206, 0.176, 0.0555, 0.14};
        arm_masses = {0.5, 0.8, 0.8, 0.3, 0.2};
        arm_widths = {0.055, 0.02, 0.02, 0.05, 0.02};

        position_offset_cb = {0.165, -0.15, 0.028};
        orientation_offset_cb = {0.0, pi / 2, 0.0};

        arm_reachability = 0.65;

        //********************************//
        // Model configuration parameters //
        //********************************//
        distance_threshold = 0.015;
        orientation_threshold = 0.15;

        horizon_speed_reduction = 90;
        horizon_arm_acceleration_reduction = 98;
        hardness_arm_acceleration_reduction = 100000;

        risk_distance = 0.30;
        safety_distance = 1.00;

        min_time_horizon = 10;

        //*****************//
        // Quadratic costs //
        //*****************//
        obstacles_repulsive_cost = 250.0;

        goal_states_cost = {1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000};
        goal_speed_cost = 1000000000;

        whole_states_cost = {20, 20, 20, 10, 10, 10, 10, 10, 100000, 100000};

        whole_inputs_cost = {1000000, 1000000, 1000000, 1000000, 1000000, 10000, 10000};

        yaw_linearization_cost = 0.02;

        //**********************//
        // Supporting variables //
        //**********************//
        number_states = 42;
        number_inputs = 7;

        number_arm_joints = 5;
        number_wheels = 6;

        number_si_constraints = 10;
        number_ps_constraints = 14;
    }
    else if(robot_name == "exoter_ack")
    {
        //******************//
        // Model properties //
        //******************//

        goal_distance_indexes = {0, 1, 2};
        goal_orientation_indexes = {3, 8, 9};

        world_ee_pose_indexes = {0, 1, 2, 3};
        base_ee_pose_indexes = {4, 5, 6, 7, 8, 9};

        robot_pose_indexes = {10, 11, 12};

        base_speed_indexes = {13, 14, 15};

        arm_position_indexes = {16, 17, 18, 19, 20};
        arm_actuators_indexes = {0, 1, 2, 3, 4};

        wheels_speed_indexes = {36, 37};
        wheels_actuators_indexes = {5};

        wheels_steering_indexes = {42};
        steering_actuators_indexes = {6};

        state_constrained_indexes = {16, 17, 18, 19, 20, 40, 41};
        input_constrained_indexes = {0, 1, 2, 3, 4};

        goal_states_indexes = {0, 1, 2, 3, 8, 9, 18};
        whole_states_indexes = {10, 11, 12, 18, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 40, 41};

        whole_inputs_indexes = {0, 1, 2, 3, 4, 5, 6};

        //********************//
        // General parameters //
        //********************//
        rolling_resistance = 0.0036;
        gravity = 9.81;

        //******************//
        // Robot Parameters //
        //******************//
        differential_width = 0.3;
        differential_length = 0.27;

        robot_weight = 15;

        robot_height = 0.202;

        wheels_radius = 0.07;
        wheels_width = 0.09;
        wheels_weight = 0.484;

        nominal_speed = 0.10;

        state_limits = {-30 * pi / 180,
                        90 * pi / 180,
                        -190 * pi / 180,
                        -10 * pi / 180,
                        -160 * pi / 180,
                        160 * pi / 180,
                        -70 * pi / 180,
                        250 * pi / 180,
                        -160 * pi / 180,
                        160 * pi / 180,
                        -2.85,
                        2.85,
                        -2.85,
                        2.85};

        input_limits = {-0.011, 0.011, -0.011, 0.011, -0.011, 0.011, -0.011, 0.011, -0.011, 0.011};

        arm_lengths = {0.0895, 0.206, 0.176, 0.0555, 0.14};
        arm_masses = {0.5, 0.8, 0.8, 0.3, 0.2};
        arm_widths = {0.055, 0.02, 0.02, 0.05, 0.02};

        position_offset_cb = {0.165, -0.15, 0.028};
        orientation_offset_cb = {0.0, pi / 2, 0.0};

        arm_reachability = 0.65;

        //********************************//
        // Model configuration parameters //
        //********************************//
        distance_threshold = 0.01;
        orientation_threshold = 0.15;

        horizon_speed_reduction = 100;
        horizon_arm_acceleration_reduction = 98;
        hardness_arm_acceleration_reduction = 1;

        risk_distance = 0.30;
        safety_distance = 1.00;

        min_time_horizon = 30;

        //*****************//
        // Quadratic costs //
        //*****************//
        obstacles_repulsive_cost = 200.0;

        goal_states_cost = {1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000};
        goal_speed_cost = 10000000;

        whole_states_cost = {20,
                             20,
                             20,
                             400,
                             60000000,
                             60000000,
                             60000000,
                             60000000,
                             60000000,
                             10,
                             10,
                             10,
                             10,
                             10,
                             100000,
                             100000};

        whole_inputs_cost = {400000, 400000, 400000, 400000, 100000, 20000, 150000};

        yaw_linearization_cost = 0.35;
        steering_forward_linearization_cost = 0.001;
        steering_angular_linearization_cost = 0.80;

        //**********************//
        // Supporting variables //
        //**********************//
        number_states = 43;
        number_inputs = 7;

        number_arm_joints = 5;
        number_wheels = 6;

        number_si_constraints = 10;
        number_ps_constraints = 14;
    }
    else
    {
        throw std::domain_error(
            RED +
            std::string("ERROR [MobileManipulator::MobileManipulator]: The provided robot name "
                        "doesn't match any of the available robot models") +
            NOCOLOR);
    }
}

MobileManipulator::~MobileManipulator() {}

uint MobileManipulator::getNumberStates()
{
    return number_states;
}

uint MobileManipulator::getNumberInputs()
{
    return number_inputs;
}

uint MobileManipulator::getNumberArmJoints()
{
    return number_arm_joints;
}

std::string MobileManipulator::getSteeringType()
{
    if(steering_actuators_indexes.size()) return "ackermann";

    return "differential";
}

std::vector<uint> MobileManipulator::getIndexesGoalDistance()
{
    return goal_distance_indexes;
}

double MobileManipulator::getThresholdGoalDistance()
{
    return distance_threshold;
}

std::vector<uint> MobileManipulator::getIndexesGoalOrientation()
{
    return goal_orientation_indexes;
}

double MobileManipulator::getThresholdGoalOrientation()
{
    return orientation_threshold;
}

std::vector<uint> MobileManipulator::getIndexesRobotPose()
{
    return robot_pose_indexes;
}

std::vector<double> MobileManipulator::getEEPose(Eigen::VectorXd & x)
{
    std::vector<double> ee_pose;

    for(uint i = 0; i < world_ee_pose_indexes.size(); i++)
    {
        ee_pose.push_back(x[world_ee_pose_indexes[i]]);
    }

    for(uint i = 4; i < base_ee_pose_indexes.size(); i++)
    {
        ee_pose.push_back(x[base_ee_pose_indexes[i]]);
    }

    return ee_pose;
}

std::vector<double> MobileManipulator::getRobotPose(Eigen::VectorXd & x)
{
    std::vector<double> robot_pose;

    for(uint i = 0; i < robot_pose_indexes.size(); i++)
    {
        robot_pose.push_back(x[robot_pose_indexes[i]]);
    }

    return robot_pose;
}

std::vector<double> MobileManipulator::getRobotBaseSpeed(Eigen::VectorXd & x)
{
    std::vector<double> robot_base_speed;

    for(uint i = 0; i < base_speed_indexes.size(); i++)
    {
        robot_base_speed.push_back(x[base_speed_indexes[i]]);
    }

    return robot_base_speed;
}

std::vector<double> MobileManipulator::getArmJointsPosition(Eigen::VectorXd & x)
{
    std::vector<double> arm_joints_position;

    for(uint i = 0; i < arm_position_indexes.size(); i++)
    {
        arm_joints_position.push_back(x[arm_position_indexes[i]]);
    }

    return arm_joints_position;
}

std::vector<double> MobileManipulator::getSteeringJointsPosition(Eigen::VectorXd & x)
{
    std::vector<double> steering_joints_position;

    if(robot_name == "exoter_ack")
    {
        double left_wheel_steer = x[wheels_steering_indexes[0]];

        steering_joints_position.push_back(left_wheel_steer);
        steering_joints_position.push_back(-left_wheel_steer);

        double right_wheel_steer = getRightWheelSteer(left_wheel_steer);

        steering_joints_position.push_back(right_wheel_steer);
        steering_joints_position.push_back(-right_wheel_steer);
    }
    else
    {
        for(uint i = 0; i < wheels_steering_indexes.size(); i++)
        {
            steering_joints_position.push_back(x[wheels_steering_indexes[i]]);
        }
    }

    return steering_joints_position;
}

std::vector<double> MobileManipulator::getArmJointsSpeedCommand(Eigen::VectorXd & u)
{
    std::vector<double> arm_speeds_command;

    for(uint i = 0; i < arm_actuators_indexes.size(); i++)
    {
        arm_speeds_command.push_back(u[arm_actuators_indexes[i]]);
    }

    return arm_speeds_command;
}

std::vector<double> MobileManipulator::getDrivingWheelsSpeedCommand(Eigen::VectorXd & x,
                                                                    Eigen::VectorXd & u)
{
    std::vector<double> driving_speeds_command;

    if(steering_actuators_indexes.size() == 1)
    {
        double left_wheels_speed = u[wheels_actuators_indexes[0]];
        driving_speeds_command.push_back(left_wheels_speed);

        double left_wheels_steer = x[wheels_steering_indexes[0]];
        double right_wheels_speed = getRightWheelsSpeed(left_wheels_speed, left_wheels_steer);
        driving_speeds_command.push_back(right_wheels_speed);
    }
    else
    {
        for(uint i = 0; i < wheels_actuators_indexes.size(); i++)
        {
            driving_speeds_command.push_back(u[wheels_actuators_indexes[i]]);
        }
    }

    return driving_speeds_command;
}

std::vector<double> MobileManipulator::getSteeringWheelsSpeedCommand(Eigen::VectorXd & x,
                                                                     Eigen::VectorXd & u)
{
    std::vector<double> steering_speeds_command;

    double left_speed_steer = u[steering_actuators_indexes[0]];
    steering_speeds_command.push_back(left_speed_steer);

    double left_wheel_steer = x[wheels_steering_indexes[0]];
    double right_wheel_steer = getRightWheelSteer(left_wheel_steer);
    double steering_ratio = left_wheel_steer == 0 ? 1 : right_wheel_steer / left_wheel_steer;

    steering_speeds_command.push_back(left_speed_steer * steering_ratio);

    return steering_speeds_command;
}

double MobileManipulator::getRiskDistance()
{
    return risk_distance;
}

double MobileManipulator::getSafetyDistance()
{
    return safety_distance;
}

double MobileManipulator::getMinTimeHorizon()
{
    return min_time_horizon;
}

Eigen::VectorXd MobileManipulator::getInitialStateVectorEigen(
    const std::vector<double> & robot_pose,
    const std::vector<double> & arm_positions)
{
    Eigen::VectorXd x = Eigen::VectorXd::Zero(number_states);

    if(robot_name == "exoter" || robot_name == "exoter_ack")
    {
        // TODO This should be obtained from a URDF file, not hardcoded
        if(robot_pose.size() != 3)
        {
            throw std::domain_error(
                RED +
                std::string("ERROR [MobileManipulator::getInitialStateVector]: The provided robot "
                            "pose doesn't match the expected size") +
                NOCOLOR);
        }

        if(arm_positions.size() != arm_position_indexes.size())
        {
            throw std::domain_error(
                RED +
                std::string("ERROR [MobileManipulator::getInitialStateVector]: The provided "
                            "arm_positions don't match the expected size") +
                NOCOLOR);
        }

        for(uint i = 0; i < robot_pose_indexes.size(); i++)
            x(robot_pose_indexes[i]) = robot_pose[i];

        for(uint i = 0; i < arm_position_indexes.size(); i++)
            x(arm_position_indexes[i]) = arm_positions[i];

        std::vector<std::vector<double>> transform_center_base =
            dot(getTraslation(position_offset_cb),
                dot(getXRot(orientation_offset_cb[0]),
                    dot(getYRot(orientation_offset_cb[1]), getZRot(orientation_offset_cb[2]))));

        std::vector<std::vector<double>> transform_base_ee(4, std::vector<double>(4, 0));
        if(!getDirectKinematicsTransform(arm_positions, arm_positions.size(), transform_base_ee))
        {
            throw std::domain_error(RED +
                                    std::string("ERROR [MobileManipulator::getInitialStateVector]: "
                                                "Failure while computing direct kinematics") +
                                    NOCOLOR);
        }

        std::vector<std::vector<double>> transform_center_ee =
            dot(transform_center_base, transform_base_ee);

        for(uint i = 0; i < base_ee_pose_indexes.size(); i++)
            switch(i)
            {
                case 0:
                    x(base_ee_pose_indexes[i]) = -transform_center_ee[2][3];
                    break;
                case 1:
                    x(base_ee_pose_indexes[i]) = transform_center_ee[i][3];
                    break;
                case 2:
                    x(base_ee_pose_indexes[i]) = transform_center_ee[0][3];
                    break;
                case 3:
                    x(base_ee_pose_indexes[i]) = arm_positions[4];
                    break;
                case 4:
                    x(base_ee_pose_indexes[i]) =
                        arm_positions[1] + arm_positions[2] + arm_positions[3];
                    break;
                case 5:
                    x(base_ee_pose_indexes[i]) = arm_positions[0];
                    break;
                default:
                    break;
            }

        std::vector<double> robot_position = robot_pose;
        robot_position[2] = robot_height;
        std::vector<std::vector<double>> transform_world_center =
            dot(getTraslation(robot_position), getZRot(robot_pose[2]));

        std::vector<std::vector<double>> transform_world_ee =
            dot(transform_world_center, transform_center_ee);

        for(uint i = 0; i < world_ee_pose_indexes.size() - 1; i++)
            x(world_ee_pose_indexes[i]) = transform_world_ee[i][3];
        x(world_ee_pose_indexes[world_ee_pose_indexes.size() - 1]) =
            robot_pose[2] + arm_positions[4];
    }

    return x;
}

Eigen::VectorXd MobileManipulator::getInitialStateVectorEigen(
    const std::vector<double> & robot_pose,
    const std::vector<double> & arm_positions,
    const std::vector<double> & arm_speeds,
    const std::vector<double> & driving_speeds,
    const std::vector<double> & steer_positions)
{
    Eigen::VectorXd x = Eigen::VectorXd::Zero(number_states);

    if(robot_name == "exoter" || robot_name == "exoter_ack")
    {
        // TODO This should be obtained from a URDF file, not hardcoded
        if(robot_pose.size() != 3)
        {
            throw std::domain_error(
                RED +
                std::string("ERROR [MobileManipulator::getInitialStateVector]: The provided robot "
                            "pose doesn't match the expected size") +
                NOCOLOR);
        }

        if(arm_positions.size() != arm_position_indexes.size())
        {
            throw std::domain_error(
                RED +
                std::string("ERROR [MobileManipulator::getInitialStateVector]: The provided "
                            "arm positions don't match the expected size") +
                NOCOLOR);
        }

        if(arm_speeds.size() != arm_position_indexes.size())
        {
            throw std::domain_error(
                RED +
                std::string("ERROR [MobileManipulator::getInitialStateVector]: The provided "
                            "arm speeds don't match the expected size") +
                NOCOLOR);
        }

        if(driving_speeds.size() != wheels_speed_indexes.size())
        {
            throw std::domain_error(
                RED +
                std::string("ERROR [MobileManipulator::getInitialStateVector]: The provided "
                            "driving speeds don't match the expected size") +
                NOCOLOR);
        }

        if(steer_positions.size() != wheels_steering_indexes.size())
        {
            throw std::domain_error(
                RED +
                std::string("ERROR [MobileManipulator::getInitialStateVector]: The provided "
                            "steering positions don't match the expected size") +
                NOCOLOR);
        }

        for(uint i = 0; i < robot_pose_indexes.size(); i++)
            x(robot_pose_indexes[i]) = robot_pose[i];

        for(uint i = 0; i < arm_position_indexes.size(); i++)
            x(arm_position_indexes[i]) = arm_positions[i];

        for(uint i = 0; i < arm_position_indexes.size(); i++)
        {
            // TODO fix this hardcode
            if(arm_speeds[i] < input_limits[0])
                x(arm_position_indexes[i] + arm_position_indexes.size()) = input_limits[0]*1.05;
            else if(arm_speeds[i] > input_limits[1])
                x(arm_position_indexes[i] + arm_position_indexes.size()) = input_limits[1]*0.95;
            else
                x(arm_position_indexes[i] + arm_position_indexes.size()) = arm_speeds[i];
        }

        for(uint i = 0; i < wheels_speed_indexes.size(); i++)
            x(wheels_speed_indexes[i]) = driving_speeds[i];

        if(robot_name == "exoter")
        {
            x(base_speed_indexes[0]) = wheels_radius / 2 * (driving_speeds[0] + driving_speeds[1]);
            x(base_speed_indexes[1]) = 0;
            x(base_speed_indexes[2]) =
                wheels_radius * (driving_speeds[0] - driving_speeds[1]) / (2 * differential_width);
        }
        else if(robot_name == "exoter_ack")
        {
            for(uint i = 0; i < wheels_steering_indexes.size(); i++)
                x(wheels_steering_indexes[i]) = steer_positions[i];

            // Additionnal Ackermann model elements
            double left_wheel_steer = x[wheels_steering_indexes[0]];
            double right_wheel_steer = getRightWheelSteer(left_wheel_steer);
            double steering_ratio =
                left_wheel_steer == 0 ? 1 : sin(left_wheel_steer) / sin(right_wheel_steer);

            x(base_speed_indexes[0]) = wheels_radius / 2 *
                                       (cos(left_wheel_steer) * driving_speeds[0] +
                                        cos(right_wheel_steer) * driving_speeds[1]);
            x(base_speed_indexes[1]) = 0;
            x(base_speed_indexes[2]) =
                x(base_speed_indexes[0]) * tan(left_wheel_steer) /
                (differential_length + differential_width * tan(left_wheel_steer));
        }

        std::vector<std::vector<double>> transform_center_base =
            dot(getTraslation(position_offset_cb),
                dot(getXRot(orientation_offset_cb[0]),
                    dot(getYRot(orientation_offset_cb[1]), getZRot(orientation_offset_cb[2]))));

        std::vector<std::vector<double>> transform_base_ee(4, std::vector<double>(4, 0));
        if(!getDirectKinematicsTransform(arm_positions, arm_positions.size(), transform_base_ee))
        {
            throw std::domain_error(RED +
                                    std::string("ERROR [MobileManipulator::getInitialStateVector]: "
                                                "Failure while computing direct kinematics") +
                                    NOCOLOR);
        }

        std::vector<std::vector<double>> transform_center_ee =
            dot(transform_center_base, transform_base_ee);

        for(uint i = 0; i < base_ee_pose_indexes.size(); i++)
            switch(i)
            {
                case 0:
                    x(base_ee_pose_indexes[i]) = -transform_center_ee[2][3];
                    break;
                case 1:
                    x(base_ee_pose_indexes[i]) = transform_center_ee[i][3];
                    break;
                case 2:
                    x(base_ee_pose_indexes[i]) = transform_center_ee[0][3];
                    break;
                case 3:
                    x(base_ee_pose_indexes[i]) = arm_positions[4];
                    break;
                case 4:
                    x(base_ee_pose_indexes[i]) =
                        arm_positions[1] + arm_positions[2] + arm_positions[3];
                    break;
                case 5:
                    x(base_ee_pose_indexes[i]) = arm_positions[0];
                    break;
                default:
                    break;
            }

        std::vector<double> robot_position = robot_pose;
        robot_position[2] = robot_height;
        std::vector<std::vector<double>> transform_world_center =
            dot(getTraslation(robot_position), getZRot(robot_pose[2]));

        std::vector<std::vector<double>> transform_world_ee =
            dot(transform_world_center, transform_center_ee);

        for(uint i = 0; i < world_ee_pose_indexes.size() - 1; i++)
            x(world_ee_pose_indexes[i]) = transform_world_ee[i][3];
        x(world_ee_pose_indexes[world_ee_pose_indexes.size() - 1]) =
            robot_pose[2] + arm_positions[4];
    }

    return x;
}

bool MobileManipulator::getInitialStateVectorEigen(const std::vector<double> & robot_pose,
                                                   const std::vector<double> & arm_positions,
                                                   Eigen::VectorXd & x)
{
    if(robot_name == "exoter" || robot_name == "exoter_ack")
    {
        // TODO This should be obtained from a URDF file, not hardcoded
        if(x.size() != number_states)
        {
            std::cout
                << RED
                << "ERROR [MobileManipulator::getInitialStateVector]: The passed-by-reference "
                   "vector doesn't match the expected size"
                << NOCOLOR << std::endl;
            return false;
        }

        if(robot_pose.size() != 3)
        {
            std::cout << RED
                      << "ERROR [MobileManipulator::getInitialStateVector]: The provided robot "
                         "pose doesn't match the expected size"
                      << NOCOLOR << std::endl;
            return false;
        }

        if(arm_positions.size() != arm_position_indexes.size())
        {
            std::cout << RED
                      << "ERROR [MobileManipulator::getInitialStateVector]: The provided arm "
                         "positions don't match the expected size"
                      << NOCOLOR << std::endl;
            return false;
        }

        for(uint i = 0; i < robot_pose_indexes.size(); i++)
            x(robot_pose_indexes[i]) = robot_pose[i];

        for(uint i = 0; i < arm_position_indexes.size(); i++)
            x(arm_position_indexes[i]) = arm_positions[i];

        std::vector<std::vector<double>> transform_center_base =
            dot(getTraslation(position_offset_cb),
                dot(getXRot(orientation_offset_cb[0]),
                    dot(getYRot(orientation_offset_cb[1]), getZRot(orientation_offset_cb[2]))));

        std::vector<std::vector<double>> transform_base_ee(4, std::vector<double>(4, 0));
        if(!getDirectKinematicsTransform(arm_positions, arm_positions.size(), transform_base_ee))
        {
            std::cout << RED
                      << "ERROR [MobileManipulator::getInitialStateVector]: Failure while "
                         "computing direct kinematics"
                      << NOCOLOR << std::endl;
            ;
        }

        std::vector<std::vector<double>> transform_center_ee =
            dot(transform_center_base, transform_base_ee);

        for(uint i = 0; i < base_ee_pose_indexes.size(); i++)
            switch(i)
            {
                case 0:
                    x(base_ee_pose_indexes[i]) = -transform_center_ee[2][3];
                    break;
                case 1:
                    x(base_ee_pose_indexes[i]) = transform_center_ee[i][3];
                    break;
                case 2:
                    x(base_ee_pose_indexes[i]) = transform_center_ee[0][3];
                    break;
                case 3:
                    x(base_ee_pose_indexes[i]) = arm_positions[4];
                    break;
                case 4:
                    x(base_ee_pose_indexes[i]) =
                        arm_positions[1] + arm_positions[2] + arm_positions[3];
                    break;
                case 5:
                    x(base_ee_pose_indexes[i]) = arm_positions[0];
                    break;
                default:
                    break;
            }

        std::vector<double> robot_position = robot_pose;
        robot_position[2] = robot_height;
        std::vector<std::vector<double>> transform_world_center =
            dot(getTraslation(robot_position), getZRot(robot_pose[2]));

        std::vector<std::vector<double>> transform_world_ee =
            dot(transform_world_center, transform_center_ee);

        for(uint i = 0; i < world_ee_pose_indexes.size() - 1; i++)
            x(world_ee_pose_indexes[i]) = transform_world_ee[i][3];
        x(world_ee_pose_indexes[world_ee_pose_indexes.size() - 1]) =
            robot_pose[2] + arm_positions[4];
    }

    return true;
}

std::vector<double> MobileManipulator::getInitialStateVector(
    const std::vector<double> & robot_pose,
    const std::vector<double> & arm_positions)
{
    std::vector<double> x(number_states, 0);

    if(robot_name == "exoter" || robot_name == "exoter_ack")
    {
        // TODO This should be obtained from a URDF file, not hardcoded
        if(robot_pose.size() != 3)
        {
            throw std::domain_error(
                RED +
                std::string("ERROR [MobileManipulator::getInitialStateVector]: The provided robot "
                            "pose doesn't match the expected size") +
                NOCOLOR);
        }

        if(arm_positions.size() != arm_position_indexes.size())
        {
            throw std::domain_error(
                RED +
                std::string("ERROR [MobileManipulator::getInitialStateVector]: The provided "
                            "arm_positions don't match the expected size") +
                NOCOLOR);
        }

        for(uint i = 0; i < robot_pose_indexes.size(); i++)
            x[robot_pose_indexes[i]] = robot_pose[i];

        for(uint i = 0; i < arm_position_indexes.size(); i++)
            x[arm_position_indexes[i]] = arm_positions[i];

        std::vector<std::vector<double>> transform_center_base =
            dot(getTraslation(position_offset_cb),
                dot(getXRot(orientation_offset_cb[0]),
                    dot(getYRot(orientation_offset_cb[1]), getZRot(orientation_offset_cb[2]))));

        std::vector<std::vector<double>> transform_base_ee(4, std::vector<double>(4, 0));
        if(!getDirectKinematicsTransform(arm_positions, arm_positions.size(), transform_base_ee))
        {
            throw std::domain_error(RED +
                                    std::string("ERROR [MobileManipulator::getInitialStateVector]: "
                                                "Failure while computing direct kinematics") +
                                    NOCOLOR);
        }

        std::vector<std::vector<double>> transform_center_ee =
            dot(transform_center_base, transform_base_ee);

        for(uint i = 0; i < base_ee_pose_indexes.size(); i++)
            switch(i)
            {
                case 0:
                    x[base_ee_pose_indexes[i]] = -transform_center_ee[2][3];
                    break;
                case 1:
                    x[base_ee_pose_indexes[i]] = transform_center_ee[i][3];
                    break;
                case 2:
                    x[base_ee_pose_indexes[i]] = transform_center_ee[0][3];
                    break;
                case 3:
                    x[base_ee_pose_indexes[i]] = arm_positions[4];
                    break;
                case 4:
                    x[base_ee_pose_indexes[i]] =
                        arm_positions[1] + arm_positions[2] + arm_positions[3];
                    break;
                case 5:
                    x[base_ee_pose_indexes[i]] = arm_positions[0];
                    break;
                default:
                    break;
            }

        std::vector<double> robot_position = robot_pose;
        robot_position[2] = robot_height;
        std::vector<std::vector<double>> transform_world_center =
            dot(getTraslation(robot_position), getZRot(robot_pose[2]));

        std::vector<std::vector<double>> transform_world_ee =
            dot(transform_world_center, transform_center_ee);

        for(uint i = 0; i < world_ee_pose_indexes.size() - 1; i++)
            x[world_ee_pose_indexes[i]] = transform_world_ee[i][3];
        x[world_ee_pose_indexes[world_ee_pose_indexes.size() - 1]] =
            robot_pose[2] + arm_positions[4];
    }

    return x;
}

bool MobileManipulator::getInitialStateVector(const std::vector<double> & robot_pose,
                                              const std::vector<double> & arm_positions,
                                              std::vector<double> & x)
{
    if(robot_name == "exoter" || robot_name == "exoter_ack")
    {
        // TODO This should be obtained from a URDF file, not hardcoded
        if(x.size() != number_states)
        {
            std::cout
                << RED
                << "ERROR [MobileManipulator::getInitialStateVector]: The passed-by-reference "
                   "vector doesn't match the expected size"
                << NOCOLOR << std::endl;
            return false;
        }

        if(robot_pose.size() != 3)
        {
            std::cout << RED
                      << "ERROR [MobileManipulator::getInitialStateVector]: The provided robot "
                         "pose doesn't match the expected size"
                      << NOCOLOR << std::endl;
            return false;
        }

        if(arm_positions.size() != arm_position_indexes.size())
        {
            std::cout << RED
                      << "ERROR [MobileManipulator::getInitialStateVector]: The provided arm "
                         "positions don't match the expected size"
                      << NOCOLOR << std::endl;
            return false;
        }

        for(uint i = 0; i < robot_pose_indexes.size(); i++)
            x[robot_pose_indexes[i]] = robot_pose[i];

        for(uint i = 0; i < arm_position_indexes.size(); i++)
            x[arm_position_indexes[i]] = arm_positions[i];

        std::vector<std::vector<double>> transform_center_base =
            dot(getTraslation(position_offset_cb),
                dot(getXRot(orientation_offset_cb[0]),
                    dot(getYRot(orientation_offset_cb[1]), getZRot(orientation_offset_cb[2]))));

        std::vector<std::vector<double>> transform_base_ee(4, std::vector<double>(4, 0));
        if(!getDirectKinematicsTransform(arm_positions, arm_positions.size(), transform_base_ee))
        {
            std::cout << RED
                      << "ERROR [MobileManipulator::getInitialStateVector]: Failure while "
                         "computing direct kinematics"
                      << NOCOLOR << std::endl;
            ;
        }

        std::vector<std::vector<double>> transform_center_ee =
            dot(transform_center_base, transform_base_ee);

        for(uint i = 0; i < base_ee_pose_indexes.size(); i++)
            switch(i)
            {
                case 0:
                    x[base_ee_pose_indexes[i]] = -transform_center_ee[2][3];
                    break;
                case 1:
                    x[base_ee_pose_indexes[i]] = transform_center_ee[i][3];
                    break;
                case 2:
                    x[base_ee_pose_indexes[i]] = transform_center_ee[0][3];
                    break;
                case 3:
                    x[base_ee_pose_indexes[i]] = arm_positions[4];
                    break;
                case 4:
                    x[base_ee_pose_indexes[i]] =
                        arm_positions[1] + arm_positions[2] + arm_positions[3];
                    break;
                case 5:
                    x[base_ee_pose_indexes[i]] = arm_positions[0];
                    break;
                default:
                    break;
            }

        std::vector<double> robot_position = robot_pose;
        robot_position[2] = robot_height;
        std::vector<std::vector<double>> transform_world_center =
            dot(getTraslation(robot_position), getZRot(robot_pose[2]));

        std::vector<std::vector<double>> transform_world_ee =
            dot(transform_world_center, transform_center_ee);

        for(uint i = 0; i < world_ee_pose_indexes.size() - 1; i++)
            x[world_ee_pose_indexes[i]] = transform_world_ee[i][3];
        x[world_ee_pose_indexes[world_ee_pose_indexes.size() - 1]] =
            robot_pose[2] + arm_positions[4];
    }

    return true;
}

Eigen::VectorXd MobileManipulator::getGoalStateVectorEigen(const std::vector<double> & goal_ee_pose)
{
    if(goal_ee_pose.size() != 6)
    {
        throw std::domain_error(RED +
                                std::string("ERROR [MobileManipulator::getGoalStateVector]: The "
                                            "goal ee pose doesn't match the expected size") +
                                NOCOLOR);
    }

    Eigen::VectorXd x = Eigen::VectorXd::Zero(number_states);

    for(uint i = 0; i < goal_distance_indexes.size(); i++)
        x(goal_distance_indexes[i]) = goal_ee_pose[i];

    for(uint i = 0; i < goal_orientation_indexes.size(); i++)
        x(goal_orientation_indexes[i]) = goal_ee_pose[i + 3];

    return x;
}

bool MobileManipulator::getGoalStateVectorEigen(const std::vector<double> & goal_ee_pose,
                                                Eigen::VectorXd & x)
{
    if(x.size() != number_states)
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::getGoalStateVector]: The passed-by-reference "
                     "vector doesn't match the expected size"
                  << NOCOLOR << std::endl;
        return false;
    }
    if(goal_ee_pose.size() != 6)
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::getGoalStateVector]: The goal ee pose doesn't "
                     "match the expected size"
                  << NOCOLOR << std::endl;
        return false;
    }

    for(uint i = 0; i < goal_distance_indexes.size(); i++)
        x(goal_distance_indexes[i]) = goal_ee_pose[i];

    for(uint i = 0; i < goal_orientation_indexes.size(); i++)
        x(goal_orientation_indexes[i]) = goal_ee_pose[i + 3];

    return true;
}

std::vector<double> MobileManipulator::getGoalStateVector(const std::vector<double> & goal_ee_pose)
{
    if(goal_ee_pose.size() != 6)
    {
        throw std::domain_error(RED +
                                std::string("ERROR [MobileManipulator::getGoalStateVector]: The "
                                            "goal ee pose doesn't match the expected size") +
                                NOCOLOR);
    }

    std::vector<double> x(number_states, 0);

    for(uint i = 0; i < goal_distance_indexes.size(); i++)
        x[goal_distance_indexes[i]] = goal_ee_pose[i];

    for(uint i = 0; i < goal_orientation_indexes.size(); i++)
        x[goal_orientation_indexes[i]] = goal_ee_pose[i + 3];

    return x;
}

bool MobileManipulator::getGoalStateVector(const std::vector<double> & goal_ee_pose,
                                           std::vector<double> & x)
{
    if(x.size() != number_states)
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::getGoalStateVector]: The passed-by-reference "
                     "vector doesn't match the expected size"
                  << NOCOLOR << std::endl;
        return false;
    }
    if(goal_ee_pose.size() != 6)
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::getGoalStateVector]: The goal ee pose doesn't "
                     "match the expected size"
                  << NOCOLOR << std::endl;
        return false;
    }

    for(uint i = 0; i < goal_distance_indexes.size(); i++)
        x[goal_distance_indexes[i]] = goal_ee_pose[i];

    for(uint i = 0; i < goal_orientation_indexes.size(); i++)
        x[goal_orientation_indexes[i]] = goal_ee_pose[i + 3];

    return true;
}

Eigen::VectorXd MobileManipulator::getInputVectorEigen(const std::vector<double> & arm_speeds,
                                                       const std::vector<double> & wheel_speeds,
                                                       const std::vector<double> & wheel_steerings)
{
    if(arm_speeds.size() != arm_actuators_indexes.size())
    {
        throw std::domain_error(RED +
                                std::string("ERROR [MobileManipulator::getInputVector]: The "
                                            "provided arm speeds don't match the expected size") +
                                NOCOLOR);
    }

    if(wheel_speeds.size() != wheels_actuators_indexes.size())
    {
        throw std::domain_error(RED +
                                std::string("ERROR [MobileManipulator::getInputVector]: The "
                                            "provided wheel speeds don't match the expected size") +
                                NOCOLOR);
    }

    if(wheel_steerings.size() != steering_actuators_indexes.size())
    {
        throw std::domain_error(
            RED +
            std::string("ERROR [MobileManipulator::getInputVector]: The "
                        "provided wheel steerings don't match the expected size") +
            NOCOLOR);
    }

    Eigen::VectorXd u = Eigen::VectorXd::Zero(number_inputs);

    for(uint i = 0; i < arm_actuators_indexes.size(); i++)
        u(arm_actuators_indexes[i]) = arm_speeds[i];

    for(uint i = 0; i < wheels_actuators_indexes.size(); i++)
        u(wheels_actuators_indexes[i]) = wheel_speeds[i];

    for(uint i = 0; i < steering_actuators_indexes.size(); i++)
        u(steering_actuators_indexes[i]) = wheel_steerings[i];

    return u;
}

bool MobileManipulator::getInputVectorEigen(const std::vector<double> & arm_speeds,
                                            const std::vector<double> & wheel_speeds,
                                            Eigen::VectorXd & u,
                                            const std::vector<double> & wheel_steerings)
{
    if(u.size() != number_inputs)
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::getInputVector]: The passed-by-reference "
                     "vector doesn't match the expected size"
                  << NOCOLOR << std::endl;
        return false;
    }

    if(arm_speeds.size() != arm_actuators_indexes.size())
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::getInputVector]: The provided arm speeds don't "
                     "match the expected size"
                  << NOCOLOR << std::endl;
        return false;
    }

    if(wheel_speeds.size() != wheels_actuators_indexes.size())
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::getInputVector]: The provided wheel speeds don't "
                     "match the expected size"
                  << NOCOLOR << std::endl;
        return false;
    }

    if(wheel_steerings.size() != steering_actuators_indexes.size())
    {
        throw std::domain_error(
            RED +
            std::string("ERROR [MobileManipulator::getInputVector]: The "
                        "provided wheel steerings don't match the expected size") +
            NOCOLOR);
    }

    for(uint i = 0; i < arm_actuators_indexes.size(); i++)
        u(arm_actuators_indexes[i]) = arm_speeds[i];

    for(uint i = 0; i < wheels_actuators_indexes.size(); i++)
        u(wheels_actuators_indexes[i]) = wheel_speeds[i];

    for(uint i = 0; i < steering_actuators_indexes.size(); i++)
        u(steering_actuators_indexes[i]) = wheel_steerings[i];

    return true;
}

std::vector<double> MobileManipulator::getInputVector(const std::vector<double> & arm_speeds,
                                                      const std::vector<double> & wheel_speeds,
                                                      const std::vector<double> & wheel_steerings)
{
    if(arm_speeds.size() != arm_actuators_indexes.size())
    {
        throw std::domain_error(RED +
                                std::string("ERROR [MobileManipulator::getInputVector]: The "
                                            "provided arm speeds don't match the expected size") +
                                NOCOLOR);
    }

    if(wheel_speeds.size() != wheels_actuators_indexes.size())
    {
        throw std::domain_error(RED +
                                std::string("ERROR [MobileManipulator::getInputVector]: The "
                                            "provided wheel speeds don't match the expected size") +
                                NOCOLOR);
    }

    if(wheel_steerings.size() != steering_actuators_indexes.size())
    {
        throw std::domain_error(
            RED +
            std::string("ERROR [MobileManipulator::getInputVector]: The "
                        "provided wheel steerings don't match the expected size") +
            NOCOLOR);
    }

    std::vector<double> u(number_inputs, 0);

    for(uint i = 0; i < arm_actuators_indexes.size(); i++)
        u[arm_actuators_indexes[i]] = arm_speeds[i];

    for(uint i = 0; i < wheels_actuators_indexes.size(); i++)
        u[wheels_actuators_indexes[i]] = wheel_speeds[i];

    for(uint i = 0; i < steering_actuators_indexes.size(); i++)
        u[steering_actuators_indexes[i]] = wheel_steerings[i];

    return u;
}

bool MobileManipulator::getInputVector(const std::vector<double> & arm_speeds,
                                       const std::vector<double> & wheel_speeds,
                                       std::vector<double> & u,
                                       const std::vector<double> & wheel_steerings)
{
    if(u.size() != number_inputs)
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::getInputVector]: The passed-by-reference "
                     "vector doesn't match the expected size"
                  << NOCOLOR << std::endl;
        return false;
    }

    if(arm_speeds.size() != arm_actuators_indexes.size())
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::getInputVector]: The provided arm speeds don't "
                     "match the expected size"
                  << NOCOLOR << std::endl;
        return false;
    }

    if(wheel_speeds.size() != wheels_actuators_indexes.size())
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::getInputVector]: The provided wheel speeds don't "
                     "match the expected size"
                  << NOCOLOR << std::endl;
        return false;
    }

    if(wheel_steerings.size() != steering_actuators_indexes.size())
    {
        throw std::domain_error(
            RED +
            std::string("ERROR [MobileManipulator::getInputVector]: The "
                        "provided wheel steerings don't match the expected size") +
            NOCOLOR);
    }

    for(uint i = 0; i < arm_actuators_indexes.size(); i++)
        u[arm_actuators_indexes[i]] = arm_speeds[i];

    for(uint i = 0; i < wheels_actuators_indexes.size(); i++)
        u[wheels_actuators_indexes[i]] = wheel_speeds[i];

    for(uint i = 0; i < steering_actuators_indexes.size(); i++)
        u[steering_actuators_indexes[i]] = wheel_steerings[i];

    return true;
}

bool MobileManipulator::getLinearizedMatrixA(const std::vector<double> & x,
                                             const std::vector<double> & u,
                                             double time_step,
                                             std::vector<std::vector<double>> & A)
{
    if(A.size() != number_states || A[0].size() != number_states)
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::getLinearizedMatrixA]: The passed-by-reference "
                     "matrix doesn't match the expected size"
                  << NOCOLOR << std::endl;
        return false;
    }

    double robot_yaw = x[robot_pose_indexes[2]];

    std::vector<double> robot_speed = {
        x[base_speed_indexes[0]], x[base_speed_indexes[1]], x[base_speed_indexes[2]]};

    std::vector<double> arm_positions;
    for(uint i = 0; i < number_arm_joints; i++)
    {
        arm_positions.push_back(x[arm_position_indexes[i]]);
    }

    // W2EEx
    A[world_ee_pose_indexes[0]][base_ee_pose_indexes[2]] = cos(robot_yaw);
    A[world_ee_pose_indexes[0]][base_ee_pose_indexes[1]] = -sin(robot_yaw);
    A[world_ee_pose_indexes[0]][robot_pose_indexes[0]] = 1;

    // W2EEy
    A[world_ee_pose_indexes[1]][base_ee_pose_indexes[2]] = sin(robot_yaw);
    A[world_ee_pose_indexes[1]][base_ee_pose_indexes[1]] = cos(robot_yaw);
    A[world_ee_pose_indexes[1]][robot_pose_indexes[1]] = 1;

    // W2EEz
    A[world_ee_pose_indexes[2]][world_ee_pose_indexes[2]] = 1;

    // W2EEroll
    A[world_ee_pose_indexes[3]][base_ee_pose_indexes[3]] = 1;
    A[world_ee_pose_indexes[3]][robot_pose_indexes[2]] = 1;

    // W2Cx
    A[robot_pose_indexes[0]][robot_pose_indexes[0]] = 1;
    A[robot_pose_indexes[0]][robot_pose_indexes[2]] =
        time_step * (-sin(robot_yaw) * robot_speed[0] * yaw_linearization_cost -
                     cos(robot_yaw) * robot_speed[1] * yaw_linearization_cost);
    A[robot_pose_indexes[0]][base_speed_indexes[0]] =
        time_step * (cos(robot_yaw) + sin(robot_yaw) * robot_yaw * yaw_linearization_cost);
    A[robot_pose_indexes[0]][base_speed_indexes[1]] =
        -time_step * (sin(robot_yaw) - cos(robot_yaw) * robot_yaw * yaw_linearization_cost);

    // W2Cy
    A[robot_pose_indexes[1]][robot_pose_indexes[1]] = 1;
    A[robot_pose_indexes[1]][robot_pose_indexes[2]] =
        time_step * (cos(robot_yaw) * robot_speed[0] * yaw_linearization_cost -
                     sin(robot_yaw) * robot_speed[1] * yaw_linearization_cost);
    A[robot_pose_indexes[1]][base_speed_indexes[0]] =
        time_step * (sin(robot_yaw) - cos(robot_yaw) * robot_yaw * yaw_linearization_cost);
    A[robot_pose_indexes[1]][base_speed_indexes[1]] =
        time_step * (cos(robot_yaw) + sin(robot_yaw) * robot_yaw * yaw_linearization_cost);

    // W2C Heading
    A[robot_pose_indexes[2]][robot_pose_indexes[2]] = 1;
    A[robot_pose_indexes[2]][base_speed_indexes[2]] = time_step;

    // Arm joints position
    for(uint i = 0; i < number_arm_joints; i++)
    {
        A[arm_position_indexes[i]][arm_position_indexes[i]] = 1;
    }

    // Arm joints acceleration
    for(uint i = 0; i < number_arm_joints; i++)
    {
        A[arm_position_indexes[i] + number_arm_joints * 2]
         [arm_position_indexes[i] + number_arm_joints] = -1 / time_step;
    }

    // Arm joints torques
    std::vector<std::vector<double>> I(number_arm_joints,
                                       std::vector<double>(number_arm_joints, 0));
    if(!getArmInertiaMatrix(arm_positions, I))
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::getLinearizedMatrixA]: Failure computing arm "
                     "inertia matrix"
                  << NOCOLOR << std::endl;
        return false;
    }
    for(uint i = 0; i < number_arm_joints; i++)
    {
        for(uint j = 0; j < number_arm_joints; j++)
        {
            A[arm_position_indexes[i] + number_arm_joints * 3]
             [arm_position_indexes[0] + number_arm_joints * 2 + j] = I[i][j];
        }
    }

    // Wheels torques
    for(uint i = 0; i < wheels_speed_indexes.size(); i++)
    {
        A[wheels_speed_indexes[i] + wheels_speed_indexes.size() * 2]
         [wheels_speed_indexes[i] + wheels_speed_indexes.size()] =
             getWheelInertia() + robot_weight / number_wheels * pow(wheels_radius, 2);
    }

    if(robot_name == "exoter")
    {
        // B2EE
        for(uint i = 0; i < base_ee_pose_indexes.size() - 3; i++)
        {
            A[base_ee_pose_indexes[i]][base_ee_pose_indexes[i]] = 1;
        }
        A[base_ee_pose_indexes[3]][arm_position_indexes[4]] = 1;
        A[base_ee_pose_indexes[4]][arm_position_indexes[3]] = 1;
        A[base_ee_pose_indexes[4]][arm_position_indexes[2]] = 1;
        A[base_ee_pose_indexes[4]][arm_position_indexes[1]] = 1;
        A[base_ee_pose_indexes[5]][arm_position_indexes[0]] = 1;

        // Wheels accelerations
        for(uint i = 0; i < wheels_speed_indexes.size(); i++)
        {
            A[wheels_speed_indexes[i] + wheels_speed_indexes.size()][wheels_speed_indexes[i]] =
                -1 / time_step;
        }
    }
    else if(robot_name == "exoter_ack")
    {
        // B2EE
        for(uint i = 0; i < base_ee_pose_indexes.size() - 3; i++)
        {
            A[base_ee_pose_indexes[i]][base_ee_pose_indexes[i]] = 1;
        }
        A[base_ee_pose_indexes[3]][arm_position_indexes[4]] = 1;
        A[base_ee_pose_indexes[4]][arm_position_indexes[3]] = 1;
        A[base_ee_pose_indexes[4]][arm_position_indexes[2]] = 1;
        A[base_ee_pose_indexes[4]][arm_position_indexes[1]] = 1;
        A[base_ee_pose_indexes[5]][arm_position_indexes[0]] = 1;

        // Additionnal Ackermann model elements
        double left_wheel_steer = x[wheels_steering_indexes[0]];
        double right_wheel_steer = getRightWheelSteer(left_wheel_steer);
        double left_wheels_speed = u[wheels_actuators_indexes[0]];
        double steering_ratio =
            left_wheel_steer == 0 ? 1 : sin(left_wheel_steer) / sin(right_wheel_steer);

        // Wheels accelerations
        A[wheels_speed_indexes[0] + wheels_speed_indexes.size()][wheels_speed_indexes[0]] =
            -1 / time_step;
        A[wheels_speed_indexes[1] + wheels_speed_indexes.size()][wheels_speed_indexes[0]] =
            -1 / time_step * steering_ratio;

        // W2C Speed x
        A[base_speed_indexes[0]][wheels_speed_indexes[1]] =
            wheels_radius / 2 * cos(right_wheel_steer);
        A[base_speed_indexes[0]][wheels_steering_indexes[0]] =
            -wheels_radius / 2 * sin(left_wheel_steer) * left_wheels_speed *
            steering_forward_linearization_cost;

        // W2C Speed Heading
        A[base_speed_indexes[2]][base_speed_indexes[0]] =
            (tan(left_wheel_steer) - (pow(tan(left_wheel_steer), 2) + 1) * left_wheel_steer *
                                         steering_angular_linearization_cost) /
            (differential_length + differential_width * tan(left_wheel_steer));
        A[base_speed_indexes[2]][wheels_steering_indexes[0]] =
            (robot_speed[0] * (pow(tan(left_wheel_steer), 2) + 1) *
             steering_angular_linearization_cost) /
            (differential_length + differential_width * tan(left_wheel_steer));

        // Steering Joints Position
        A[wheels_steering_indexes[0]][wheels_steering_indexes[0]] = 1;
    }

    return true;
}

bool MobileManipulator::getLinearizedMatrixA(const Eigen::VectorXd & x,
                                             const Eigen::VectorXd & u,
                                             double time_step,
                                             Eigen::MatrixXd & A)
{
    if(A.rows() != number_states || A.cols() != number_states)
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::getLinearizedMatrixA]: The passed-by-reference "
                     "matrix doesn't match the expected size"
                  << NOCOLOR << std::endl;
        return false;
    }

    double robot_yaw = x(robot_pose_indexes[2]);

    std::vector<double> robot_speed = {
        x(base_speed_indexes[0]), x(base_speed_indexes[1]), x(base_speed_indexes[2])};

    std::vector<double> arm_positions;
    for(uint i = 0; i < number_arm_joints; i++)
    {
        arm_positions.push_back(x(arm_position_indexes[i]));
    }

    // W2EEx
    A(world_ee_pose_indexes[0], base_ee_pose_indexes[2]) = cos(robot_yaw);
    A(world_ee_pose_indexes[0], base_ee_pose_indexes[1]) = -sin(robot_yaw);
    A(world_ee_pose_indexes[0], robot_pose_indexes[0]) = 1;

    // W2EEy
    A(world_ee_pose_indexes[1], base_ee_pose_indexes[2]) = sin(robot_yaw);
    A(world_ee_pose_indexes[1], base_ee_pose_indexes[1]) = cos(robot_yaw);
    A(world_ee_pose_indexes[1], robot_pose_indexes[1]) = 1;

    // W2EEz
    A(world_ee_pose_indexes[2], world_ee_pose_indexes[2]) = 1;

    // W2EEroll
    A(world_ee_pose_indexes[3], base_ee_pose_indexes[3]) = 1;
    A(world_ee_pose_indexes[3], robot_pose_indexes[2]) = 1;

    // W2Cx
    A(robot_pose_indexes[0], robot_pose_indexes[0]) = 1;
    A(robot_pose_indexes[0], robot_pose_indexes[2]) =
        time_step * (-sin(robot_yaw) * robot_speed[0] * yaw_linearization_cost -
                     cos(robot_yaw) * robot_speed[1] * yaw_linearization_cost);
    A(robot_pose_indexes[0], base_speed_indexes[0]) =
        time_step * (cos(robot_yaw) + sin(robot_yaw) * robot_yaw * yaw_linearization_cost);
    A(robot_pose_indexes[0], base_speed_indexes[1]) =
        -time_step * (sin(robot_yaw) - cos(robot_yaw) * robot_yaw * yaw_linearization_cost);

    // W2Cy
    A(robot_pose_indexes[1], robot_pose_indexes[1]) = 1;
    A(robot_pose_indexes[1], robot_pose_indexes[2]) =
        time_step * (cos(robot_yaw) * robot_speed[0] * yaw_linearization_cost -
                     sin(robot_yaw) * robot_speed[1] * yaw_linearization_cost);
    A(robot_pose_indexes[1], base_speed_indexes[0]) =
        time_step * (sin(robot_yaw) - cos(robot_yaw) * robot_yaw * yaw_linearization_cost);
    A(robot_pose_indexes[1], base_speed_indexes[1]) =
        time_step * (cos(robot_yaw) + sin(robot_yaw) * robot_yaw * yaw_linearization_cost);

    // W2C Heading
    A(robot_pose_indexes[2], robot_pose_indexes[2]) = 1;
    A(robot_pose_indexes[2], base_speed_indexes[2]) = time_step;

    // Arm joints position
    for(uint i = 0; i < number_arm_joints; i++)
    {
        A(arm_position_indexes[i], arm_position_indexes[i]) = 1;
    }

    // Arm joints acceleration
    for(uint i = 0; i < number_arm_joints; i++)
    {
        A(arm_position_indexes[i] + number_arm_joints * 2,
          arm_position_indexes[i] + number_arm_joints) = -1 / time_step;
    }

    // Arm joints torques
    std::vector<std::vector<double>> I(number_arm_joints,
                                       std::vector<double>(number_arm_joints, 0));
    if(!getArmInertiaMatrix(arm_positions, I))
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::getLinearizedMatrixA]: Failure while computing arm "
                     "inertia matrix"
                  << NOCOLOR << std::endl;
        return false;
    }
    for(uint i = 0; i < number_arm_joints; i++)
    {
        for(uint j = 0; j < number_arm_joints; j++)
        {
            A(arm_position_indexes[i] + number_arm_joints * 3,
              arm_position_indexes[0] + number_arm_joints * 2 + j) = I[i][j];
        }
    }

    // Wheels torques
    for(uint i = 0; i < wheels_speed_indexes.size(); i++)
    {
        A(wheels_speed_indexes[i] + wheels_speed_indexes.size() * 2,
          wheels_speed_indexes[i] + wheels_speed_indexes.size()) =
            getWheelInertia() + robot_weight / number_wheels * pow(wheels_radius, 2);
    }

    if(robot_name == "exoter")
    {
        // B2EE
        for(uint i = 0; i < base_ee_pose_indexes.size() - 3; i++)
        {
            A(base_ee_pose_indexes[i], base_ee_pose_indexes[i]) = 1;
        }
        A(base_ee_pose_indexes[3], arm_position_indexes[4]) = 1;
        A(base_ee_pose_indexes[4], arm_position_indexes[3]) = 1;
        A(base_ee_pose_indexes[4], arm_position_indexes[2]) = 1;
        A(base_ee_pose_indexes[4], arm_position_indexes[1]) = 1;
        A(base_ee_pose_indexes[5], arm_position_indexes[0]) = 1;

        // Wheels accelerations
        for(uint i = 0; i < wheels_speed_indexes.size(); i++)
        {
            A(wheels_speed_indexes[i] + wheels_speed_indexes.size(), wheels_speed_indexes[i]) =
                -1 / time_step;
        }
    }
    else if(robot_name == "exoter_ack")
    {
        // B2EE
        for(uint i = 0; i < base_ee_pose_indexes.size() - 3; i++)
        {
            A(base_ee_pose_indexes[i], base_ee_pose_indexes[i]) = 1;
        }
        A(base_ee_pose_indexes[3], arm_position_indexes[4]) = 1;
        A(base_ee_pose_indexes[4], arm_position_indexes[3]) = 1;
        A(base_ee_pose_indexes[4], arm_position_indexes[2]) = 1;
        A(base_ee_pose_indexes[4], arm_position_indexes[1]) = 1;
        A(base_ee_pose_indexes[5], arm_position_indexes[0]) = 1;

        // Additionnal Ackermann model elements
        double left_wheel_steer = x(wheels_steering_indexes[0]);
        double right_wheel_steer = getRightWheelSteer(left_wheel_steer);
        double left_wheels_speed = u(wheels_actuators_indexes[0]);
        double steering_ratio =
            left_wheel_steer == 0 ? 1 : sin(left_wheel_steer) / sin(right_wheel_steer);

        // Wheels accelerations
        A(wheels_speed_indexes[0] + wheels_speed_indexes.size(), wheels_speed_indexes[0]) =
            -1 / time_step;
        A(wheels_speed_indexes[1] + wheels_speed_indexes.size(), wheels_speed_indexes[0]) =
            -1 / time_step * steering_ratio;

        // W2C Speed x
        A(base_speed_indexes[0], wheels_speed_indexes[1]) =
            wheels_radius / 2 * cos(right_wheel_steer);
        A(base_speed_indexes[0], wheels_steering_indexes[0]) =
            -wheels_radius / 2 * sin(left_wheel_steer) * left_wheels_speed *
            steering_forward_linearization_cost;

        // W2C Speed Heading
        A(base_speed_indexes[2], base_speed_indexes[0]) =
            (tan(left_wheel_steer) - (pow(tan(left_wheel_steer), 2) + 1) * left_wheel_steer *
                                         steering_angular_linearization_cost) /
            (differential_length + differential_width * tan(left_wheel_steer));
        A(base_speed_indexes[2], wheels_steering_indexes[0]) =
            (robot_speed[0] * (pow(tan(left_wheel_steer), 2) + 1) *
             steering_angular_linearization_cost) /
            (differential_length + differential_width * tan(left_wheel_steer));

        // Steering Joints Position
        A(wheels_steering_indexes[0], wheels_steering_indexes[0]) = 1;
    }

    return true;
}

bool MobileManipulator::getLinearizedMatrixB(const std::vector<double> & x,
                                             const std::vector<double> & u,
                                             double time_step,
                                             std::vector<std::vector<double>> & B)
{
    if(B.size() != number_states || B[0].size() != number_inputs)
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::getLinearizedMatrixB]: The passed-by-reference "
                     "matrix doesn't match the expected size"
                  << NOCOLOR << std::endl;
        return false;
    }

    std::vector<double> arm_positions;
    for(uint i = 0; i < number_arm_joints; i++)
    {
        arm_positions.push_back(x[arm_position_indexes[i]]);
    }

    std::vector<double> arm_actuators;
    for(uint i = 0; i < number_arm_joints; i++)
    {
        arm_actuators.push_back(u[arm_actuators_indexes[i]]);
    }

    // WTEEz
    std::vector<std::vector<double>> J(6, std::vector<double>(number_arm_joints, 0));
    if(!getArmJacobianMatrix(arm_positions, J))
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::getLinearizedMatrixB]: Failure while computing "
                     "jacobian matrix"
                  << NOCOLOR << std::endl;
        return false;
    }

    for(uint i = 0; i < number_arm_joints; i++)
    {
        B[world_ee_pose_indexes[2]][arm_actuators_indexes[i]] = -time_step * J[1][i];
    }

    // BTEE
    for(uint i = 0; i < base_ee_pose_indexes.size() - 3; i++)
    {
        for(uint j = 0; j < number_arm_joints; j++)
        {
            B[base_ee_pose_indexes[i]][arm_actuators_indexes[j]] = time_step * J[i][j];
        }
    }

    // Arm joints position
    for(uint i = 0; i < number_arm_joints; i++)
    {
        B[arm_position_indexes[i]][arm_actuators_indexes[i]] = time_step;
    }

    // Arm joints speed
    for(uint i = 0; i < number_arm_joints; i++)
    {
        B[arm_position_indexes[i] + number_arm_joints][arm_actuators_indexes[i]] = 1;
    }

    // Arm joints acceleration
    for(uint i = 0; i < number_arm_joints; i++)
    {
        B[arm_position_indexes[i] + number_arm_joints * 2][arm_actuators_indexes[i]] =
            1 / time_step;
    }

    // Arm joints torques
    std::vector<std::vector<double>> C(number_arm_joints,
                                       std::vector<double>(number_arm_joints, 0));
    if(!getArmCoriolisMatrix(arm_positions, arm_actuators, C))
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::getLinearizedMatrixB]: Failure while computing arm "
                     "coriolis matrix"
                  << NOCOLOR << std::endl;
        return false;
    }
    for(uint i = 0; i < number_arm_joints; i++)
    {
        for(uint j = 0; j < number_arm_joints; j++)
        {
            B[arm_position_indexes[i] + number_arm_joints * 3][arm_actuators_indexes[j]] = C[i][j];
        }
    }

    if(robot_name == "exoter")
    {
        // W2C Speed x
        B[base_speed_indexes[0]][wheels_actuators_indexes[0]] = wheels_radius / 2;
        B[base_speed_indexes[0]][wheels_actuators_indexes[1]] = wheels_radius / 2;

        // W2C Speed heading
        B[base_speed_indexes[2]][wheels_actuators_indexes[0]] =
            wheels_radius / (2 * differential_width);
        B[base_speed_indexes[2]][wheels_actuators_indexes[1]] =
            -wheels_radius / (2 * differential_width);

        // Wheels speeds
        for(uint i = 0; i < wheels_speed_indexes.size(); i++)
        {
            B[wheels_speed_indexes[i]][wheels_actuators_indexes[i]] = 1;
        }

        // Wheels accelerations
        for(uint i = 0; i < wheels_speed_indexes.size(); i++)
        {
            B[wheels_speed_indexes[i] + wheels_speed_indexes.size()][wheels_actuators_indexes[i]] =
                1 / time_step;
        }
    }
    else if(robot_name == "exoter_ack")
    {
        double left_wheel_steer = x[wheels_steering_indexes[0]];
        double right_wheel_steer = getRightWheelSteer(left_wheel_steer);
        double steering_ratio =
            left_wheel_steer == 0 ? 1 : sin(left_wheel_steer) / sin(right_wheel_steer);

        // W2C Speed x
        B[base_speed_indexes[0]][wheels_actuators_indexes[0]] =
            wheels_radius / 2 *
            (cos(left_wheel_steer) +
             left_wheel_steer * sin(left_wheel_steer) * steering_forward_linearization_cost);

        // Wheels speeds
        B[wheels_speed_indexes[0]][wheels_actuators_indexes[0]] = 1;
        B[wheels_speed_indexes[1]][wheels_actuators_indexes[0]] = steering_ratio;

        // Wheels accelerations
        B[wheels_speed_indexes[0] + wheels_speed_indexes.size()][wheels_actuators_indexes[0]] =
            1 / time_step;
        B[wheels_speed_indexes[1] + wheels_speed_indexes.size()][wheels_actuators_indexes[0]] =
            1 / time_step * steering_ratio;

        // Steering Joints Position
        B[wheels_steering_indexes[0]][steering_actuators_indexes[0]] = 1;
    }

    return true;
}

bool MobileManipulator::getLinearizedMatrixB(const Eigen::VectorXd & x,
                                             const Eigen::VectorXd & u,
                                             double time_step,
                                             Eigen::MatrixXd & B)
{
    if(B.rows() != number_states || B.cols() != number_inputs)
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::getLinearizedMatrixB]: The passed-by-reference "
                     "matrix doesn't match the expected size"
                  << NOCOLOR << std::endl;
        return false;
    }

    std::vector<double> arm_positions;
    for(uint i = 0; i < number_arm_joints; i++)
    {
        arm_positions.push_back(x[arm_position_indexes[i]]);
    }

    std::vector<double> arm_actuators;
    for(uint i = 0; i < number_arm_joints; i++)
    {
        arm_actuators.push_back(u[arm_actuators_indexes[i]]);
    }

    // WTEEz
    std::vector<std::vector<double>> J(6, std::vector<double>(number_arm_joints, 0));
    if(!getArmJacobianMatrix(arm_positions, J))
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::getLinearizedMatrixB]: Failure while computing "
                     "jacobian matrix"
                  << NOCOLOR << std::endl;
        return false;
    }

    for(uint i = 0; i < number_arm_joints; i++)
    {
        B(world_ee_pose_indexes[2], arm_actuators_indexes[i]) = -time_step * J[0][i];
    }

    // BTEE
    for(uint i = 0; i < base_ee_pose_indexes.size() - 3; i++)
    {
        for(uint j = 0; j < number_arm_joints; j++)
        {
            B(base_ee_pose_indexes[i], arm_actuators_indexes[j]) = time_step * J[i][j];
        }
    }

    // Arm joints position
    for(uint i = 0; i < number_arm_joints; i++)
    {
        B(arm_position_indexes[i], arm_actuators_indexes[i]) = time_step;
    }

    // Arm joints speed
    for(uint i = 0; i < number_arm_joints; i++)
    {
        B(arm_position_indexes[i] + number_arm_joints, arm_actuators_indexes[i]) = 1;
    }

    // Arm joints acceleration
    for(uint i = 0; i < number_arm_joints; i++)
    {
        B(arm_position_indexes[i] + number_arm_joints * 2, arm_actuators_indexes[i]) =
            1 / time_step;
    }

    // Arm joints torques
    std::vector<std::vector<double>> C(number_arm_joints,
                                       std::vector<double>(number_arm_joints, 0));
    if(!getArmCoriolisMatrix(arm_positions, arm_actuators, C))
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::getLinearizedMatrixB]: Failure while computing arm "
                     "coriolis matrix"
                  << NOCOLOR << std::endl;
        return false;
    }
    for(uint i = 0; i < number_arm_joints; i++)
    {
        for(uint j = 0; j < number_arm_joints; j++)
        {
            B(arm_position_indexes[i] + number_arm_joints * 3, arm_actuators_indexes[j]) = C[i][j];
        }
    }

    if(robot_name == "exoter")
    {
        // W2C Speed x
        B(base_speed_indexes[0], wheels_actuators_indexes[0]) = wheels_radius / 2;
        B(base_speed_indexes[0], wheels_actuators_indexes[1]) = wheels_radius / 2;

        // W2C Speed heading
        B(base_speed_indexes[2], wheels_actuators_indexes[0]) =
            wheels_radius / (2 * differential_width);
        B(base_speed_indexes[2], wheels_actuators_indexes[1]) =
            -wheels_radius / (2 * differential_width);

        // Wheels speeds
        for(uint i = 0; i < wheels_speed_indexes.size(); i++)
        {
            B(wheels_speed_indexes[i], wheels_actuators_indexes[i]) = 1;
        }

        // Wheels accelerations
        for(uint i = 0; i < wheels_speed_indexes.size(); i++)
        {
            B(wheels_speed_indexes[i] + wheels_speed_indexes.size(), wheels_actuators_indexes[i]) =
                1 / time_step;
        }
    }
    else if(robot_name == "exoter_ack")
    {
        double left_wheel_steer = x(wheels_steering_indexes[0]);
        double right_wheel_steer = getRightWheelSteer(left_wheel_steer);
        double steering_ratio =
            left_wheel_steer == 0 ? 1 : sin(left_wheel_steer) / sin(right_wheel_steer);

        // W2C Speed x
        B(base_speed_indexes[0], wheels_actuators_indexes[0]) =
            wheels_radius / 2 *
            (cos(left_wheel_steer) +
             left_wheel_steer * sin(left_wheel_steer) * steering_forward_linearization_cost);

        // Wheels speeds
        B(wheels_speed_indexes[0], wheels_actuators_indexes[0]) = 1;
        B(wheels_speed_indexes[1], wheels_actuators_indexes[0]) = steering_ratio;

        // Wheels accelerations
        B(wheels_speed_indexes[0] + wheels_speed_indexes.size(), wheels_actuators_indexes[0]) =
            1 / time_step;
        B(wheels_speed_indexes[1] + wheels_speed_indexes.size(), wheels_actuators_indexes[0]) =
            1 / time_step * steering_ratio;

        // Steering Joints Position
        B(wheels_steering_indexes[0], steering_actuators_indexes[0]) = 1;
    }

    return true;
}

uint MobileManipulator::getNumberStateInputConstraints()
{
    return number_si_constraints;
}

bool MobileManipulator::getConstraintsMatrixC(std::vector<std::vector<double>> & C)
{
    if(C.size() != number_si_constraints || C[0].size() != number_states)
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::getConstraintsMatrixC]: The passed-by-reference "
                     "matrix doesn't match the expected size"
                  << NOCOLOR << std::endl;
        return false;
    }

    C = std::vector<std::vector<double>>(number_si_constraints,
                                         std::vector<double>(number_states, 0));

    return true;
}

bool MobileManipulator::getConstraintsMatrixC(Eigen::MatrixXd & C)
{
    if(C.rows() != number_si_constraints || C.cols() != number_states)
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::getConstraintsMatrixC]: The passed-by-reference "
                     "matrix doesn't match the expected size"
                  << NOCOLOR << std::endl;
        return false;
    }

    C = Eigen::MatrixXd::Zero(number_si_constraints, number_states);

    return true;
}

bool MobileManipulator::getConstraintsMatrixD(std::vector<std::vector<double>> & D)
{
    if(D.size() != number_si_constraints || D[0].size() != number_inputs)
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::getConstraintsMatrixD]: The passed-by-reference "
                     "matrix doesn't match the expected size"
                  << NOCOLOR << std::endl;
        return false;
    }

    for(uint i = 0; i < number_si_constraints; i += 2)
    {
        D[i][input_constrained_indexes[i / 2]] = -1;
        D[i + 1][input_constrained_indexes[i / 2]] = 1;
    }

    return true;
}

bool MobileManipulator::getConstraintsMatrixD(Eigen::MatrixXd & D)
{
    if(D.rows() != number_si_constraints || D.cols() != number_inputs)
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::getConstraintsMatrixD]: The passed-by-reference "
                     "matrix doesn't match the expected size"
                  << NOCOLOR << std::endl;
        return false;
    }

    for(uint i = 0; i < number_si_constraints; i += 2)
    {
        D(i, input_constrained_indexes[i / 2]) = -1;
        D(i + 1, input_constrained_indexes[i / 2]) = 1;
    }

    return true;
}

bool MobileManipulator::getConstraintsMatrixR(std::vector<double> & r)
{
    if(r.size() != number_si_constraints)
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::getConstraintsMatrixR]: The passed-by-reference "
                     "matrix doesn't match the expected size"
                  << NOCOLOR << std::endl;
        return false;
    }

    for(uint i = 0; i < number_si_constraints; i += 2)
    {
        r[i] = input_limits[i];
        r[i + 1] = -input_limits[i + 1];
    }

    return true;
}

bool MobileManipulator::getConstraintsMatrixR(Eigen::VectorXd & r)
{
    if(r.size() != number_si_constraints)
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::getConstraintsMatrixR]: The passed-by-reference "
                     "matrix doesn't match the expected size"
                  << NOCOLOR << std::endl;
        return false;
    }

    for(uint i = 0; i < number_si_constraints; i += 2)
    {
        r(i) = input_limits[i];
        r(i + 1) = -input_limits[i + 1];
    }

    return true;
}

uint MobileManipulator::getNumberPureStateConstraints()
{
    return number_ps_constraints;
}

bool MobileManipulator::getConstraintsMatrixG(std::vector<std::vector<double>> & G)
{
    if(G.size() != number_ps_constraints || G[0].size() != number_states)
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::getConstraintsMatrixG]: The passed-by-reference "
                     "matrix doesn't match the expected size"
                  << NOCOLOR << std::endl;
        return false;
    }

    for(uint i = 0; i < number_ps_constraints; i += 2)
    {
        G[i][state_constrained_indexes[i / 2]] = -1;
        G[i + 1][state_constrained_indexes[i / 2]] = 1;
    }

    return true;
}

bool MobileManipulator::getConstraintsMatrixG(Eigen::MatrixXd & G)
{
    if(G.rows() != number_ps_constraints || G.cols() != number_states)
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::getConstraintsMatrixG]: The passed-by-reference "
                     "matrix doesn't match the expected size"
                  << NOCOLOR << std::endl;
        return false;
    }

    for(uint i = 0; i < number_ps_constraints; i += 2)
    {
        G(i, state_constrained_indexes[i / 2]) = -1;
        G(i + 1, state_constrained_indexes[i / 2]) = 1;
    }

    return true;
}

bool MobileManipulator::getConstraintsMatrixH(std::vector<double> & h)
{
    if(h.size() != number_ps_constraints)
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::getConstraintsMatrixH]: The passed-by-reference "
                     "matrix doesn't match the expected size"
                  << NOCOLOR << std::endl;
        return false;
    }

    for(uint i = 0; i < number_ps_constraints; i += 2)
    {
        h[i] = state_limits[i];
        h[i + 1] = -state_limits[i + 1];
    }

    return true;
}

bool MobileManipulator::getConstraintsMatrixH(Eigen::VectorXd & h)
{
    if(h.size() != number_ps_constraints)
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::getConstraintsMatrixH]: The passed-by-reference "
                     "matrix doesn't match the expected size"
                  << NOCOLOR << std::endl;
        return false;
    }

    for(uint i = 0; i < number_ps_constraints; i += 2)
    {
        h(i) = state_limits[i];
        h(i + 1) = -state_limits[i + 1];
    }

    return true;
}

bool MobileManipulator::getStateCostMatrix(double percentage_horizon,
                                           double time_horizon,
                                           std::vector<std::vector<double>> & Q,
                                           bool track_reference_trajectory)
{
    if(Q.size() != number_states || Q[0].size() != number_states)
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::getStateCostMatrix]: The passed-by-reference "
                     "matrix doesn't match the expected size"
                  << NOCOLOR << std::endl;
        return false;
    }

    // The costs are proportional to the time horizon
    double time_ratio = time_horizon / 60;

    for(uint i = 0; i < whole_states_indexes.size(); i++)
    {
        if((whole_states_indexes[i] == robot_pose_indexes[0] ||
            whole_states_indexes[i] == robot_pose_indexes[1] ||
            whole_states_indexes[i] == robot_pose_indexes[2]) &&
           track_reference_trajectory)
            Q[whole_states_indexes[i]][whole_states_indexes[i]] = whole_states_cost[i] * time_ratio;
        else
            Q[whole_states_indexes[i]][whole_states_indexes[i]] = whole_states_cost[i] / time_ratio;
    }

    if(percentage_horizon == 100)
    {
        for(uint i = 0; i < goal_states_indexes.size(); i++)
        {
            Q[goal_states_indexes[i]][goal_states_indexes[i]] = goal_states_cost[i] / time_ratio;
        }
    }
    else if(percentage_horizon > horizon_speed_reduction)
    {
        double linear_cost =
            (percentage_horizon - horizon_speed_reduction) / (100 - horizon_speed_reduction);

        for(uint i = 0; i < base_speed_indexes.size(); i++)
        {
            Q[base_speed_indexes[i]][base_speed_indexes[i]] =
                linear_cost * goal_speed_cost / time_ratio;
        }
    }

    if(percentage_horizon > horizon_arm_acceleration_reduction)
    {
        for(uint i = 0; i < number_arm_joints; i++)
        {
            Q[arm_position_indexes[i] + 2 * number_arm_joints]
             [arm_position_indexes[i] + 2 * number_arm_joints] *=
                hardness_arm_acceleration_reduction;
        }
    }

    return true;
}

bool MobileManipulator::getStateCostMatrix(double percentage_horizon,
                                           double time_horizon,
                                           Eigen::MatrixXd & Q,
                                           bool track_reference_trajectory)
{
    if(Q.rows() != number_states || Q.cols() != number_states)
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::getStateCostMatrix]: The passed-by-reference "
                     "matrix doesn't match the expected size"
                  << NOCOLOR << std::endl;
        return false;
    }

    // The costs are proportional to the time horizon
    double time_ratio = time_horizon / 60;

    for(uint i = 0; i < whole_states_indexes.size(); i++)
    {
        if((whole_states_indexes[i] == robot_pose_indexes[0] ||
            whole_states_indexes[i] == robot_pose_indexes[1] ||
            whole_states_indexes[i] == robot_pose_indexes[2]) &&
           track_reference_trajectory)
            Q(whole_states_indexes[i], whole_states_indexes[i]) = whole_states_cost[i] * time_ratio;
        else
            Q(whole_states_indexes[i], whole_states_indexes[i]) = whole_states_cost[i] / time_ratio;
    }

    if(percentage_horizon == 100)
    {
        for(uint i = 0; i < goal_states_indexes.size(); i++)
        {
            Q(goal_states_indexes[i], goal_states_indexes[i]) = goal_states_cost[i] / time_ratio;
        }
    }

    if(percentage_horizon > horizon_speed_reduction)
    {
        double linear_cost =
            (percentage_horizon - horizon_speed_reduction) / (100 - horizon_speed_reduction);

        for(uint i = 0; i < base_speed_indexes.size(); i++)
        {
            Q(base_speed_indexes[i], base_speed_indexes[i]) =
                linear_cost * goal_speed_cost / time_ratio;
        }
    }

    if(percentage_horizon > horizon_arm_acceleration_reduction)
    {
        for(uint i = 0; i < number_arm_joints; i++)
        {
            Q(arm_position_indexes[i] + 2 * number_arm_joints,
              arm_position_indexes[i] + 2 * number_arm_joints) *=
                hardness_arm_acceleration_reduction;
        }
    }

    return true;
}

bool MobileManipulator::getInputCostMatrix(std::vector<std::vector<double>> & R,
                                           double time_horizon)
{
    if(R.size() != number_inputs || R[0].size() != number_inputs)
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::getInputCostMatrix]: The passed-by-reference "
                     "matrix doesn't match the expected size"
                  << NOCOLOR << std::endl;
        return false;
    }

    // The costs are proportional to the time horizon
    double time_ratio = time_horizon / 60;

    for(uint i = 0; i < whole_inputs_indexes.size(); i++)
    {
        if(whole_inputs_indexes[i] == wheels_actuators_indexes[0] ||
           whole_inputs_indexes[i] == wheels_actuators_indexes[1])
            R[whole_inputs_indexes[i]][whole_inputs_indexes[i]] = whole_inputs_cost[i] / time_ratio;
        else
            R[whole_inputs_indexes[i]][whole_inputs_indexes[i]] = whole_inputs_cost[i] * time_ratio;
    }

    return true;
}

bool MobileManipulator::getInputCostMatrix(Eigen::MatrixXd & R, double time_horizon)
{
    if(R.rows() != number_inputs || R.cols() != number_inputs)
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::getInputCostMatrix]: The passed-by-reference "
                     "matrix doesn't match the expected size"
                  << NOCOLOR << std::endl;
        return false;
    }

    // The costs are proportional to the time horizon
    double time_ratio = time_horizon / 60;

    for(uint i = 0; i < whole_inputs_indexes.size(); i++)
    {
        if(whole_inputs_indexes[i] == wheels_actuators_indexes[0] ||
           whole_inputs_indexes[i] == wheels_actuators_indexes[1])
            R(whole_inputs_indexes[i], whole_inputs_indexes[i]) = whole_inputs_cost[i] / time_ratio;
        else
            R(whole_inputs_indexes[i], whole_inputs_indexes[i]) = whole_inputs_cost[i] * time_ratio;
    }

    return true;
}

bool MobileManipulator::getStateInputCostMatrix(std::vector<std::vector<double>> & K)
{
    if(K.size() != number_states || K[0].size() != number_inputs)
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::getStateInputCostMatrix]: The passed-by-reference "
                     "matrix doesn't match the expected size"
                  << NOCOLOR << std::endl;
        return false;
    }

    K = std::vector<std::vector<double>>(number_states, std::vector<double>(number_inputs, 0));

    return true;
}

bool MobileManipulator::getStateInputCostMatrix(Eigen::MatrixXd & K)
{
    if(K.rows() != number_states || K.cols() != number_inputs)
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::getStateInputCostMatrix]: The passed-by-reference "
                     "matrix doesn't match the expected size"
                  << NOCOLOR << std::endl;
        return false;
    }

    K = Eigen::MatrixXd::Zero(number_states, number_inputs);

    return true;
}

bool MobileManipulator::getArmGravityMatrix(std::vector<double> arm_positions,
                                            std::vector<double> & G)
{
    if(G.size() != number_arm_joints)
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::getArmGravityMatrix]: The passed-by-reference "
                     "matrix doesn't match the expected size"
                  << NOCOLOR << std::endl;
        return false;
    }

    if(robot_name == "exoter" || robot_name == "exoter_ack")
    {
        // TODO This should be obtained from a URDF file, not hardcoded
        G[0] = 2.45e-4 * sin(arm_positions[0]) *
               (333.0 * sin(arm_positions[1] + arm_positions[2] + arm_positions[3]) +
                5630.0 * cos(arm_positions[1] + arm_positions[2]) + 1.4e+4 * cos(arm_positions[1]));

        G[1] = 0.404 * sin(arm_positions[0] + arm_positions[1]) -
               0.404 * sin(arm_positions[0] - 1.0 * arm_positions[1]) +
               2.63 * cos(arm_positions[0]) * sin(arm_positions[1]) +
               1.38 * sin(arm_positions[1] + arm_positions[2]) * cos(arm_positions[0]) -
               0.0817 * cos(arm_positions[1] + arm_positions[2]) * cos(arm_positions[0]) *
                   cos(arm_positions[3]) +
               0.0817 * sin(arm_positions[1] + arm_positions[2]) * cos(arm_positions[0]) *
                   sin(arm_positions[3]);

        G[2] = -2.45e-4 * cos(arm_positions[0]) *
               (333.0 * cos(arm_positions[1] + arm_positions[2] + arm_positions[3]) -
                5630.0 * sin(arm_positions[1] + arm_positions[2]));

        G[3] = -0.136 * cos(arm_positions[1] + arm_positions[2] + arm_positions[3]) *
               cos(arm_positions[0]);

        G[4] = 0;
    }
    return true;
}

bool MobileManipulator::getArmInertiaMatrix(const std::vector<double> & arm_positions,
                                            std::vector<std::vector<double>> & I)
{
    if(I.size() != number_arm_joints || I[0].size() != number_arm_joints)
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::getArmInertiaMatrix]: The passed-by-reference "
                     "matrix doesn't match the expected size"
                  << NOCOLOR << std::endl;
        return false;
    }

    if(robot_name == "exoter" || robot_name == "exoter_ack")
    {
        // TODO This should be obtained from a URDF file, not hardcoded
        I[0][0] =
            0.029 * cos(2.0 * arm_positions[1] + arm_positions[2]) +
            0.00147 * sin(2.0 * arm_positions[1] + 2.0 * arm_positions[2] + arm_positions[3]) +
            0.0318 * cos(2.0 * arm_positions[1]) +
            0.00171 * sin(arm_positions[2] + arm_positions[3]) +
            0.00171 * sin(2.0 * arm_positions[1] + arm_positions[2] + arm_positions[3]) +
            0.029 * cos(arm_positions[2]) + 0.00147 * sin(arm_positions[3]) +
            0.00852 * cos(2.0 * arm_positions[1] + 2.0 * arm_positions[2]) -
            3.17e-4 *
                cos(2.0 * arm_positions[1] + 2.0 * arm_positions[2] + 2.0 * arm_positions[3]) +
            0.047;

        I[0][4] = 4.0e-5 * cos(arm_positions[1] + arm_positions[2] + arm_positions[3]);

        I[1][1] = 0.00343 * sin(arm_positions[2] + arm_positions[3]) +
                  0.058 * cos(arm_positions[2]) + 0.00293 * sin(arm_positions[3]) + 0.0821;

        I[1][2] = 0.00171 * sin(arm_positions[2] + arm_positions[3]) +
                  0.029 * cos(arm_positions[2]) + 0.00293 * sin(arm_positions[3]) + 0.0182;

        I[1][3] = 0.00286 * sin(arm_positions[2] + arm_positions[3]) +
                  0.00195 * sin(arm_positions[3]) + 0.00105;

        I[2][1] = 0.00171 * sin(arm_positions[2] + arm_positions[3]) +
                  0.029 * cos(arm_positions[2]) + 0.00293 * sin(arm_positions[3]) + 0.0182;

        I[2][2] = 0.00293 * sin(arm_positions[3]) + 0.0182;

        I[2][3] = 0.00195 * sin(arm_positions[3]) + 0.00105;

        I[3][1] = 0.00286 * sin(arm_positions[2] + arm_positions[3]) +
                  0.00195 * sin(arm_positions[3]) + 0.00105;

        I[3][2] = 0.00195 * sin(arm_positions[3]) + 0.00105;

        I[3][3] = 0.0012;

        I[4][0] = 4.0e-5 * cos(arm_positions[1] + arm_positions[2] + arm_positions[3]);

        I[4][4] = 4.0e-5;
    }

    return true;
}

bool MobileManipulator::getArmCoriolisMatrix(const std::vector<double> & arm_positions,
                                             const std::vector<double> & arm_speeds,
                                             std::vector<std::vector<double>> & C)
{
    if(C.size() != number_arm_joints || C[0].size() != number_arm_joints)
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::getArmCoriolisMatrix]: The passed-by-reference "
                     "matrix doesn't match the expected size"
                  << NOCOLOR << std::endl;
        return false;
    }

    if(robot_name == "exoter" || robot_name == "exoter_ack")
    {
        // TODO This should be obtained from a URDF file, not hardcoded
        C[0][1] =
            0.00293 * arm_speeds[0] *
                cos(2.0 * arm_positions[1] + 2.0 * arm_positions[2] + arm_positions[3]) -
            0.058 * arm_speeds[0] * sin(2.0 * arm_positions[1] + arm_positions[2]) -
            0.0637 * arm_speeds[0] * sin(2.0 * arm_positions[1]) +
            0.00343 * arm_speeds[0] *
                cos(2.0 * arm_positions[1] + arm_positions[2] + arm_positions[3]) -
            0.017 * arm_speeds[0] * sin(2.0 * arm_positions[1] + 2.0 * arm_positions[2]) +
            6.35e-4 * arm_speeds[0] *
                sin(2.0 * arm_positions[1] + 2.0 * arm_positions[2] + 2.0 * arm_positions[3]) -
            4.0e-5 * arm_speeds[4] * sin(arm_positions[1] + arm_positions[2] + arm_positions[3]);

        C[0][2] =
            0.00293 * arm_speeds[0] *
                cos(2.0 * arm_positions[1] + 2.0 * arm_positions[2] + arm_positions[3]) -
            0.029 * arm_speeds[0] * sin(2.0 * arm_positions[1] + arm_positions[2]) +
            0.00171 * arm_speeds[0] * cos(arm_positions[2] + arm_positions[3]) +
            0.00171 * arm_speeds[0] *
                cos(2.0 * arm_positions[1] + arm_positions[2] + arm_positions[3]) -
            0.029 * arm_speeds[0] * sin(arm_positions[2]) -
            0.017 * arm_speeds[0] * sin(2.0 * arm_positions[1] + 2.0 * arm_positions[2]) +
            6.35e-4 * arm_speeds[0] *
                sin(2.0 * arm_positions[1] + 2.0 * arm_positions[2] + 2.0 * arm_positions[3]) -
            4.0e-5 * arm_speeds[4] * sin(arm_positions[1] + arm_positions[2] + arm_positions[3]);

        C[0][3] =
            0.00147 * arm_speeds[0] *
                cos(2.0 * arm_positions[1] + 2.0 * arm_positions[2] + arm_positions[3]) +
            0.00171 * arm_speeds[0] * cos(arm_positions[2] + arm_positions[3]) +
            0.00171 * arm_speeds[0] *
                cos(2.0 * arm_positions[1] + arm_positions[2] + arm_positions[3]) +
            0.00147 * arm_speeds[0] * cos(arm_positions[3]) +
            6.35e-4 * arm_speeds[0] *
                sin(2.0 * arm_positions[1] + 2.0 * arm_positions[2] + 2.0 * arm_positions[3]) -
            4.0e-5 * arm_speeds[4] * sin(arm_positions[1] + arm_positions[2] + arm_positions[3]);

        C[1][0] =
            0.029 * arm_speeds[0] * sin(2.0 * arm_positions[1] + arm_positions[2]) -
            0.00147 * arm_speeds[0] *
                cos(2.0 * arm_positions[1] + 2.0 * arm_positions[2] + arm_positions[3]) +
            0.0318 * arm_speeds[0] * sin(2.0 * arm_positions[1]) -
            0.00171 * arm_speeds[0] *
                cos(2.0 * arm_positions[1] + arm_positions[2] + arm_positions[3]) +
            0.00852 * arm_speeds[0] * sin(2.0 * arm_positions[1] + 2.0 * arm_positions[2]) -
            3.17e-4 * arm_speeds[0] *
                sin(2.0 * arm_positions[1] + 2.0 * arm_positions[2] + 2.0 * arm_positions[3]) +
            2.0e-5 * arm_speeds[4] * sin(arm_positions[1] + arm_positions[2] + arm_positions[3]);

        C[1][2] = 0.00343 * arm_speeds[1] * cos(arm_positions[2] + arm_positions[3]) +
                  0.00171 * arm_speeds[2] * cos(arm_positions[2] + arm_positions[3]) +
                  0.00286 * arm_speeds[3] * cos(arm_positions[2] + arm_positions[3]) -
                  0.058 * arm_speeds[1] * sin(arm_positions[2]) -
                  0.029 * arm_speeds[2] * sin(arm_positions[2]);

        C[1][3] = 0.00343 * arm_speeds[1] * cos(arm_positions[2] + arm_positions[3]) +
                  0.00171 * arm_speeds[2] * cos(arm_positions[2] + arm_positions[3]) +
                  0.00286 * arm_speeds[3] * cos(arm_positions[2] + arm_positions[3]) +
                  0.00293 * arm_speeds[1] * cos(arm_positions[3]) +
                  0.00293 * arm_speeds[2] * cos(arm_positions[3]) +
                  0.00195 * arm_speeds[3] * cos(arm_positions[3]);

        C[1][4] =
            2.0e-5 * arm_speeds[0] * sin(arm_positions[1] + arm_positions[2] + arm_positions[3]);

        C[2][0] =
            0.0145 * arm_speeds[0] * sin(2.0 * arm_positions[1] + arm_positions[2]) -
            0.00147 * arm_speeds[0] *
                cos(2.0 * arm_positions[1] + 2.0 * arm_positions[2] + arm_positions[3]) -
            8.57e-4 * arm_speeds[0] * cos(arm_positions[2] + arm_positions[3]) -
            8.57e-4 * arm_speeds[0] *
                cos(2.0 * arm_positions[1] + arm_positions[2] + arm_positions[3]) +
            0.0145 * arm_speeds[0] * sin(arm_positions[2]) +
            0.00852 * arm_speeds[0] * sin(2.0 * arm_positions[1] + 2.0 * arm_positions[2]) -
            3.17e-4 * arm_speeds[0] *
                sin(2.0 * arm_positions[1] + 2.0 * arm_positions[2] + 2.0 * arm_positions[3]) +
            2.0e-5 * arm_speeds[4] * sin(arm_positions[1] + arm_positions[2] + arm_positions[3]);

        C[2][1] = 0.029 * arm_speeds[1] * sin(arm_positions[2]) -
                  8.57e-4 * arm_speeds[2] * cos(arm_positions[2] + arm_positions[3]) -
                  0.00143 * arm_speeds[3] * cos(arm_positions[2] + arm_positions[3]) -
                  0.00171 * arm_speeds[1] * cos(arm_positions[2] + arm_positions[3]) +
                  0.0145 * arm_speeds[2] * sin(arm_positions[2]);

        C[2][2] =
            2.58e-6 * arm_speeds[1] *
            (333.0 * cos(arm_positions[2] + arm_positions[3]) - 5630.0 * sin(arm_positions[2]));

        C[2][3] = 2.86e-4 * arm_speeds[1] * cos(arm_positions[2] + arm_positions[3]) +
                  0.00293 * arm_speeds[1] * cos(arm_positions[3]) +
                  0.00293 * arm_speeds[2] * cos(arm_positions[3]) +
                  0.00195 * arm_speeds[3] * cos(arm_positions[3]);

        C[2][4] =
            2.0e-5 * arm_speeds[0] * sin(arm_positions[1] + arm_positions[2] + arm_positions[3]);

        C[3][0] =
            2.0e-5 * arm_speeds[4] * sin(arm_positions[1] + arm_positions[2] + arm_positions[3]) -
            8.57e-4 * arm_speeds[0] * cos(arm_positions[2] + arm_positions[3]) -
            8.57e-4 * arm_speeds[0] *
                cos(2.0 * arm_positions[1] + arm_positions[2] + arm_positions[3]) -
            7.33e-4 * arm_speeds[0] * cos(arm_positions[3]) -
            3.17e-4 * arm_speeds[0] *
                sin(2.0 * arm_positions[1] + 2.0 * arm_positions[2] + 2.0 * arm_positions[3]) -
            7.33e-4 * arm_speeds[0] *
                cos(2.0 * arm_positions[1] + 2.0 * arm_positions[2] + arm_positions[3]);

        C[3][1] = -0.00171 * arm_speeds[1] * cos(arm_positions[2] + arm_positions[3]) -
                  8.57e-4 * arm_speeds[2] * cos(arm_positions[2] + arm_positions[3]) -
                  0.00143 * arm_speeds[3] * cos(arm_positions[2] + arm_positions[3]) -
                  0.00147 * arm_speeds[1] * cos(arm_positions[3]) -
                  0.00147 * arm_speeds[2] * cos(arm_positions[3]) -
                  9.77e-4 * arm_speeds[3] * cos(arm_positions[3]);

        C[3][2] = 0.002 * arm_speeds[1] * cos(arm_positions[2] + arm_positions[3]) -
                  0.00147 * arm_speeds[1] * cos(arm_positions[3]) -
                  0.00147 * arm_speeds[2] * cos(arm_positions[3]) -
                  9.77e-4 * arm_speeds[3] * cos(arm_positions[3]);
        C[3][3] = 0.00143 * arm_speeds[1] * cos(arm_positions[2] + arm_positions[3]) +
                  9.77e-4 * arm_speeds[1] * cos(arm_positions[3]) +
                  9.77e-4 * arm_speeds[2] * cos(arm_positions[3]);

        C[3][4] =
            2.0e-5 * arm_speeds[0] * sin(arm_positions[1] + arm_positions[2] + arm_positions[3]);

        C[4][1] =
            -4.0e-5 * arm_speeds[0] * sin(arm_positions[1] + arm_positions[2] + arm_positions[3]);

        C[4][2] =
            -4.0e-5 * arm_speeds[0] * sin(arm_positions[1] + arm_positions[2] + arm_positions[3]);

        C[4][3] =
            -4.0e-5 * arm_speeds[0] * sin(arm_positions[1] + arm_positions[2] + arm_positions[3]);
    }

    return true;
}

bool MobileManipulator::getArmJacobianMatrix(const std::vector<double> & arm_positions,
                                             std::vector<std::vector<double>> & J)
{
    if(J.size() != 6 || J[0].size() != number_arm_joints)
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::getArmJacobianMatrix]: The passed-by-reference "
                     "matrix doesn't match the expected size"
                  << NOCOLOR << std::endl;
        return false;
    }

    if(robot_name == "exoter" || robot_name == "exoter_ack")
    {
        // TODO This should be obtained from a URDF file, not hardcoded
        std::vector<std::vector<std::vector<double>>> TB(
            number_arm_joints + 1, std::vector<std::vector<double>>(4, std::vector<double>(4, 0)));

        if(!getDirectKinematicsTransform(arm_positions, TB))
        {
            std::cout << RED
                      << "ERROR [MobileManipulator::getArmJacobianMatrix]: Failure while "
                         "computing direct kinematics"
                      << NOCOLOR << std::endl;
            return false;
        }

        for(uint i = 0; i < TB[0].size(); i++)
        {
            for(uint j = 0; j < TB[0][0].size(); j++)
            {
                if(abs(TB[0][i][j]) < 1e-6) TB[0][i][j] = 0;
                if(abs(TB[1][i][j]) < 1e-6) TB[1][i][j] = 0;
                if(abs(TB[2][i][j]) < 1e-6) TB[2][i][j] = 0;
                if(abs(TB[3][i][j]) < 1e-6) TB[3][i][j] = 0;
                if(abs(TB[4][i][j]) < 1e-6) TB[4][i][j] = 0;
                if(abs(TB[5][i][j]) < 1e-6) TB[5][i][j] = 0;
            }
        }

        std::vector<std::vector<double>> z;
        z.push_back(std::vector<double>{TB[0][0][2], TB[0][1][2], TB[0][2][2]});
        z.push_back(std::vector<double>{TB[1][0][2], TB[1][1][2], TB[1][2][2]});
        z.push_back(std::vector<double>{TB[2][0][2], TB[2][1][2], TB[2][2][2]});
        z.push_back(std::vector<double>{TB[3][0][2], TB[3][1][2], TB[3][2][2]});
        z.push_back(std::vector<double>{TB[4][0][2], TB[4][1][2], TB[4][2][2]});
        z.push_back(std::vector<double>{TB[5][0][2], TB[5][1][2], TB[5][2][2]});

        std::vector<std::vector<double>> p;
        p.push_back(std::vector<double>{TB[0][0][3], TB[0][1][3], TB[0][2][3]});
        p.push_back(std::vector<double>{TB[1][0][3], TB[1][1][3], TB[1][2][3]});
        p.push_back(std::vector<double>{TB[2][0][3], TB[2][1][3], TB[2][2][3]});
        p.push_back(std::vector<double>{TB[3][0][3], TB[3][1][3], TB[3][2][3]});
        p.push_back(std::vector<double>{TB[4][0][3], TB[4][1][3], TB[4][2][3]});
        p.push_back(std::vector<double>{TB[5][0][3], TB[5][1][3], TB[5][2][3]});

        std::vector<std::vector<double>> Jp, Jo;

        for(uint i = 1; i < p.size(); i++)
        {
            std::vector<double> diff(p[0].size(), 0);
            std::vector<double> cross_product(3, 0);
            if(!getDifference(p[p.size() - 1], p[i - 1], diff) ||
               !getCrossProduct(z[i - 1], diff, cross_product))
            {
                std::cout << RED
                          << "ERROR [MobileManipulator::getArmJacobianMatrix]: Failure while "
                             "computing position jacobian"
                          << NOCOLOR << std::endl;
                return false;
            }

            Jp.push_back(cross_product);
            // Jo.push_back(z[i - 1]);
        }

        // Manually generating orientation jacobian
        Jo.resize(3);
        Jo[0] = {0, 0, 0, 0, 1};
        Jo[1] = {0, 1, 1, 1, 0};
        Jo[2] = {1, 0, 0, 0, 0};

        for(uint i = 0; i < number_arm_joints; i++)
        {
            for(uint j = 0; j < 6; j++)
            {
                if(j < 3)
                    J[j][i] = Jp[i][j];
                else
                    J[j][i] = Jo[j - 3][i];

                if(abs(J[j][i]) < 1e-6) J[j][i] = 0;
            }
        }
    }

    return true;
}

bool MobileManipulator::getDirectKinematicsTransform(const std::vector<double> & arm_positions,
                                                     uint joint_index,
                                                     std::vector<std::vector<double>> & T)
{
    if(T.size() != 4 || T[0].size() != 4)
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::getDirectKinematicsTransform]: The "
                     "passed-by-reference matrix doesn't match the expected size"
                  << NOCOLOR << std::endl;
        return false;
    }

    if(robot_name == "exoter" || robot_name == "exoter_ack")
    {
        // TODO This should be obtained from a URDF file, not hardcoded
        if(joint_index > number_arm_joints)
        {
            std::cout << RED
                      << "ERROR [MobileManipulator::getDirectKinematicsTransform]: Provided index "
                         "is greater than the number of joints"
                      << NOCOLOR << std::endl;
            return false;
        }

        std::vector<std::vector<double>> TB0(4, std::vector<double>(4, 0));

        if(!getTraslation({0, 0, 0}, TB0))
        {
            std::cout << RED
                      << "ERROR [MobileManipulator::getDirectKinematicsTransform]: Failure while "
                         "computing a traslation matrix"
                      << NOCOLOR << std::endl;
            return false;
        }

        if(joint_index == 0)
        {
            T = TB0;
            return true;
        }

        std::vector<std::vector<double>> TB1(4, std::vector<double>(4, 0));

        std::vector<std::vector<double>> T01 =
            dot(getTraslation({0, 0, arm_lengths[0]}),
                dot(getZRot(arm_positions[0]), dot(getTraslation({0, 0, 0}), getXRot(-pi / 2))));

        TB1 = dot(TB0, T01);
        if(joint_index == 1)
        {
            T = TB1;
            return true;
        }

        std::vector<std::vector<double>> TB2(4, std::vector<double>(4, 0));

        std::vector<std::vector<double>> T12 = dot(
            getTraslation({0, 0, 0}),
            dot(getZRot(arm_positions[1]), dot(getTraslation({arm_lengths[1], 0, 0}), getXRot(0))));

        TB2 = dot(TB1, T12);

        if(joint_index == 2)
        {
            T = TB2;
            return true;
        }

        std::vector<std::vector<double>> TB3(4, std::vector<double>(4, 0));

        std::vector<std::vector<double>> T23 = dot(
            getTraslation({0, 0, 0}),
            dot(getZRot(arm_positions[2]), dot(getTraslation({arm_lengths[2], 0, 0}), getXRot(0))));

        TB3 = dot(TB2, T23);
        if(joint_index == 3)
        {
            T = TB3;
            return true;
        }

        std::vector<std::vector<double>> TB4(4, std::vector<double>(4, 0));

        std::vector<std::vector<double>> T34 =
            dot(getTraslation({0, 0, 0}),
                dot(getZRot(arm_positions[3]), dot(getTraslation({0, 0, 0}), getXRot(pi / 2))));

        TB4 = dot(TB3, T34);
        if(joint_index == 4)
        {
            T = TB4;
            return true;
        }

        std::vector<std::vector<double>> TB5(4, std::vector<double>(4, 0));

        std::vector<std::vector<double>> T45 =
            dot(getTraslation({0, 0, arm_lengths[3] + arm_lengths[4]}),
                dot(getZRot(arm_positions[4]), dot(getTraslation({0, 0, 0}), getXRot(0))));

        TB5 = dot(TB4, T45);
        T = TB5;
    }

    return true;
}

bool MobileManipulator::getDirectKinematicsTransform(
    const std::vector<double> & arm_positions,
    std::vector<std::vector<std::vector<double>>> & T)
{
    if(T.size() != number_arm_joints + 1 || T[0].size() != 4 || T[0][0].size() != 4)
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::getDirectKinematicsTransform]: The "
                     "passed-by-reference matrix doesn't match the expected size"
                  << NOCOLOR << std::endl;
        return false;
    }

    if(robot_name == "exoter" || robot_name == "exoter_ack")
    {
        // TODO This should be obtained from a URDF file, not hardcoded

        if(!getTraslation({0, 0, 0}, T[0]))
        {
            std::cout << RED
                      << "ERROR [MobileManipulator::getDirectKinematicsTransform]: Failure while "
                         "computing direct kinematics TB0"
                      << NOCOLOR << std::endl;
            return false;
        }

        std::vector<std::vector<double>> z_traslation(4, std::vector<double>(4, 0));
        std::vector<std::vector<double>> z_rotation(4, std::vector<double>(4, 0));
        std::vector<std::vector<double>> x_traslation(4, std::vector<double>(4, 0));
        std::vector<std::vector<double>> x_rotation(4, std::vector<double>(4, 0));

        std::vector<std::vector<double>> T_aux1(4, std::vector<double>(4, 0));
        std::vector<std::vector<double>> T_aux2(4, std::vector<double>(4, 0));
        std::vector<std::vector<double>> T_aux3(4, std::vector<double>(4, 0));

        // Computing TB1 sequentially (TB0*z_tras*z_rot*x_tras*x_rot)
        if(!getTraslation({0, 0, arm_lengths[0]}, z_traslation) ||
           !getZRot(arm_positions[0], z_rotation) || !getTraslation({0, 0, 0}, x_traslation) ||
           !getXRot(-pi / 2, x_rotation) ||

           !dot(z_traslation, z_rotation, T_aux1) || !dot(T_aux1, x_traslation, T_aux2) ||
           !dot(T_aux2, x_rotation, T_aux3) || !dot(T[0], T_aux3, T[1]))
        {
            std::cout << RED
                      << "ERROR [MobileManipulator::getDirectKinematicsTransform]: Failure while "
                         "computing direct kinematics TB1"
                      << NOCOLOR << std::endl;
            return false;
        }

        // Computing TB2 sequentially (TB1*z_tras*z_rot*x_tras*x_rot)
        if(!getTraslation({0, 0, 0}, z_traslation) || !getZRot(arm_positions[1], z_rotation) ||
           !getTraslation({arm_lengths[1], 0, 0}, x_traslation) || !getXRot(0, x_rotation) ||

           !dot(z_traslation, z_rotation, T_aux1) || !dot(T_aux1, x_traslation, T_aux2) ||
           !dot(T_aux2, x_rotation, T_aux3) || !dot(T[1], T_aux3, T[2]))
        {
            std::cout << RED
                      << "ERROR [MobileManipulator::getDirectKinematicsTransform]: Failure while "
                         "computing direct kinematics TB2"
                      << NOCOLOR << std::endl;
            return false;
        }

        // Computing TB3 sequentially (TB2*z_tras*z_rot*x_tras*x_rot)
        if(!getTraslation({0, 0, 0}, z_traslation) || !getZRot(arm_positions[2], z_rotation) ||
           !getTraslation({arm_lengths[2], 0, 0}, x_traslation) || !getXRot(0, x_rotation) ||

           !dot(z_traslation, z_rotation, T_aux1) || !dot(T_aux1, x_traslation, T_aux2) ||
           !dot(T_aux2, x_rotation, T_aux3) || !dot(T[2], T_aux3, T[3]))
        {
            std::cout << RED
                      << "ERROR [MobileManipulator::getDirectKinematicsTransform]: Failure while "
                         "computing direct kinematics TB3"
                      << NOCOLOR << std::endl;
            return false;
        }

        // Computing TB4 sequentially (TB3*z_tras*z_rot*x_tras*x_rot)
        if(!getTraslation({0, 0, 0}, z_traslation) || !getZRot(arm_positions[3], z_rotation) ||
           !getTraslation({0, 0, 0}, x_traslation) || !getXRot(pi / 2, x_rotation) ||

           !dot(z_traslation, z_rotation, T_aux1) || !dot(T_aux1, x_traslation, T_aux2) ||
           !dot(T_aux2, x_rotation, T_aux3) || !dot(T[3], T_aux3, T[4]))
        {
            std::cout << RED
                      << "ERROR [MobileManipulator::getDirectKinematicsTransform]: Failure while "
                         "computing direct kinematics TB4"
                      << NOCOLOR << std::endl;
            return false;
        }

        // Computing TB5 sequentially (TB4*z_tras*z_rot*x_tras*x_rot)
        if(!getTraslation({0, 0, arm_lengths[3] + arm_lengths[4]}, z_traslation) ||
           !getZRot(arm_positions[4], z_rotation) || !getTraslation({0, 0, 0}, x_traslation) ||
           !getXRot(0, x_rotation) ||

           !dot(z_traslation, z_rotation, T_aux1) || !dot(T_aux1, x_traslation, T_aux2) ||
           !dot(T_aux2, x_rotation, T_aux3) || !dot(T[4], T_aux3, T[5]))
        {
            std::cout << RED
                      << "ERROR [MobileManipulator::getDirectKinematicsTransform]: Failure while "
                         "computing direct kinematics TB5"
                      << NOCOLOR << std::endl;
            return false;
        }
    }

    return true;
}

bool MobileManipulator::getObstaclesCost(
    const std::vector<double> & x,
    double map_resolution,
    const std::vector<std::vector<double>> & gradient_obstacles_map_X,
    const std::vector<std::vector<double>> & gradient_obstacles_map_Y,
    double time_horizon,
    std::vector<double> & obstacles_cost)
{
    if(obstacles_cost.size() != number_states)
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::getObstaclesCost]: The passed-by-reference vector "
                     "doesn't match the expected size"
                  << NOCOLOR << std::endl;
        return false;
    }

    uint m = gradient_obstacles_map_X[0].size();
    uint n = gradient_obstacles_map_X.size();

    if(m != gradient_obstacles_map_Y[0].size() || n != gradient_obstacles_map_Y.size())
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::getObstaclesCost]: The gradient matrixes doesn't "
                     "have the same size"
                  << NOCOLOR << std::endl;
        return false;
    }

    std::vector<double> robot_pose = {x[robot_pose_indexes[0]], x[robot_pose_indexes[1]]};
    uint ix = (uint)(robot_pose[0] / map_resolution);
    uint iy = (uint)(robot_pose[1] / map_resolution);

    if(ix > m - 3 || ix < 0 + 2 || iy > n - 3 || iy < 0 + 2)
    {
        std::cout << MAGENTA
                  << "WARNING [MobileManipulator::getObstaclesCost]: The robot is out of the map"
                  << NOCOLOR << std::endl;

        if(ix > m - 3) ix = m - 3;
        if(ix < 0 + 2) ix = 0 + 2;
        if(iy > n - 3) iy = n - 3;
        if(iy < 0 + 2) iy = 0 + 2;
    }

    double time_ratio = time_horizon / 60;

    obstacles_cost[robot_pose_indexes[0]] =
        obstacles_repulsive_cost * gradient_obstacles_map_X[iy][ix] * time_ratio;
    obstacles_cost[robot_pose_indexes[1]] =
        obstacles_repulsive_cost * gradient_obstacles_map_Y[iy][ix] * time_ratio;

    return true;
}

bool MobileManipulator::getObstaclesCost(
    const Eigen::VectorXd & x,
    double map_resolution,
    const std::vector<std::vector<double>> & gradient_obstacles_map_X,
    const std::vector<std::vector<double>> & gradient_obstacles_map_Y,
    double time_horizon,
    Eigen::VectorXd & obstacles_cost)
{
    if(obstacles_cost.size() != number_states)
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::getObstaclesCost]: The passed-by-reference vector "
                     "doesn't match the expected size"
                  << NOCOLOR << std::endl;
        return false;
    }

    uint m = gradient_obstacles_map_X[0].size();
    uint n = gradient_obstacles_map_X.size();

    if(m != gradient_obstacles_map_Y[0].size() || n != gradient_obstacles_map_Y.size())
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::getObstaclesCost]: The gradient matrixes doesn't "
                     "have the same size"
                  << NOCOLOR << std::endl;
        return false;
    }

    std::vector<double> robot_pose = {x(robot_pose_indexes[0]), x(robot_pose_indexes[1])};
    uint ix = (uint)(robot_pose[0] / map_resolution);
    uint iy = (uint)(robot_pose[1] / map_resolution);

    if(ix > m - 3 || ix < 0 + 2 || iy > n - 3 || iy < 0 + 2)
    {
        std::cout << MAGENTA
                  << "WARNING [MobileManipulator::getObstaclesCost]: The robot is out of the map"
                  << NOCOLOR << std::endl;

        if(ix > m - 3) ix = m - 3;
        if(ix < 0 + 2) ix = 0 + 2;
        if(iy > n - 3) iy = n - 3;
        if(iy < 0 + 2) iy = 0 + 2;
    }

    double time_ratio = time_horizon / 60;

    obstacles_cost[robot_pose_indexes[0]] =
        obstacles_repulsive_cost * gradient_obstacles_map_X[iy][ix] * time_ratio;
    obstacles_cost[robot_pose_indexes[1]] =
        obstacles_repulsive_cost * gradient_obstacles_map_Y[iy][ix] * time_ratio;

    return true;
}

double MobileManipulator::getWheelInertia()
{
    double I = wheels_weight * pow(wheels_radius, 2) / 2;
    return I;
}

double MobileManipulator::getRightWheelSteer(double left_wheel_steer)
{
    double right_wheel_steer =
        atan2(differential_length,
              differential_length / tan(left_wheel_steer) + 2 * differential_width) +
        0.000000000000001;

    while(right_wheel_steer > pi / 2)
        right_wheel_steer = right_wheel_steer - pi;
    while(right_wheel_steer < -pi / 2)
        right_wheel_steer = right_wheel_steer + pi;

    return right_wheel_steer;
}

double MobileManipulator::getRightWheelsSpeed(double left_wheels_speed, double left_wheel_steer)
{
    double right_wheel_steer =
        atan2(differential_length,
              differential_length / tan(left_wheel_steer) + 2 * differential_width) +
        0.000000000000001;

    while(right_wheel_steer > pi / 2)
        right_wheel_steer = right_wheel_steer - pi;
    while(right_wheel_steer < -pi / 2)
        right_wheel_steer = right_wheel_steer + pi;

    double steering_ratio =
        left_wheel_steer == 0 ? 1 : sin(left_wheel_steer) / sin(right_wheel_steer);

    return left_wheels_speed * steering_ratio;
}

bool MobileManipulator::forwardIntegrateModel(std::vector<double> x,
                                              std::vector<double> u,
                                              double time_step,
                                              std::vector<double> & xf)
{
    if(xf.size() != number_states || x.size() != number_states || u.size() != number_inputs)
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::forwardIntegrateModel]: The arguments doesn't "
                     "match the expected size"
                  << NOCOLOR << std::endl;
        return false;
    }

    std::vector<double> world_ee_pose;
    for(uint i = 0; i < world_ee_pose_indexes.size(); i++)
    {
        world_ee_pose.push_back(x[world_ee_pose_indexes[i]]);
    }

    std::vector<double> arm_positions;
    for(uint i = 0; i < number_arm_joints; i++)
    {
        arm_positions.push_back(x[arm_position_indexes[i]]);
    }

    std::vector<double> arm_speeds;
    for(uint i = 0; i < number_arm_joints; i++)
    {
        arm_speeds.push_back(u[arm_actuators_indexes[i]]);
    }

    std::vector<double> arm_previous_speeds;
    for(uint i = 0; i < number_arm_joints; i++)
    {
        arm_previous_speeds.push_back(x[arm_position_indexes[i] + number_arm_joints]);
    }

    std::vector<double> arm_accelerations;
    for(uint i = 0; i < number_arm_joints; i++)
    {
        arm_accelerations.push_back(x[arm_position_indexes[i] + number_arm_joints * 2]);
    }

    std::vector<double> base_ee_pose;
    for(uint i = 0; i < base_ee_pose_indexes.size(); i++)
    {
        base_ee_pose.push_back(x[base_ee_pose_indexes[i]]);
    }

    std::vector<double> robot_pose;
    for(uint i = 0; i < robot_pose_indexes.size(); i++)
    {
        robot_pose.push_back(x[robot_pose_indexes[i]]);
    }

    double robot_yaw = x[robot_pose_indexes[2]];

    std::vector<double> base_speed;
    for(uint i = 0; i < base_speed_indexes.size(); i++)
    {
        base_speed.push_back(x[base_speed_indexes[i]]);
    }

    std::vector<double> wheels_speed;
    for(uint i = 0; i < wheels_actuators_indexes.size(); i++)
    {
        wheels_speed.push_back(u[wheels_actuators_indexes[i]]);
    }

    std::vector<double> wheels_previous_speed;
    for(uint i = 0; i < wheels_speed_indexes.size(); i++)
    {
        wheels_previous_speed.push_back(x[wheels_speed_indexes[i]]);
    }

    std::vector<double> wheels_accelerations;
    for(uint i = 0; i < wheels_speed_indexes.size(); i++)
    {
        wheels_accelerations.push_back(x[wheels_speed_indexes[i] + wheels_speed_indexes.size()]);
    }

    std::vector<std::vector<double>> J(6, std::vector<double>(number_arm_joints, 0));
    if(!getArmJacobianMatrix(arm_positions, J))
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::forwardIntegrateModel]: Failure while computing "
                     "jacobian matrix"
                  << NOCOLOR << std::endl;
        return false;
    }

    // W2EE (x, y, z, roll)
    xf[world_ee_pose_indexes[0]] =
        cos(robot_yaw) * base_ee_pose[2] - sin(robot_yaw) * base_ee_pose[1] + robot_pose[0];
    xf[world_ee_pose_indexes[1]] =
        sin(robot_yaw) * base_ee_pose[2] + cos(robot_yaw) * base_ee_pose[1] + robot_pose[1];
    xf[world_ee_pose_indexes[2]] = world_ee_pose[2] - dot(J[0], arm_speeds) * time_step;
    xf[world_ee_pose_indexes[3]] = robot_yaw + base_ee_pose[3];

    // W2C
    xf[robot_pose_indexes[0]] = robot_pose[0] + cos(robot_yaw) * base_speed[0] * time_step -
                                sin(robot_yaw) * base_speed[1] * time_step;
    xf[robot_pose_indexes[1]] = robot_pose[1] + sin(robot_yaw) * base_speed[0] * time_step +
                                cos(robot_yaw) * base_speed[1] * time_step;
    xf[robot_pose_indexes[2]] = robot_pose[2] + base_speed[2] * time_step;

    // Arm joints position
    for(uint i = 0; i < number_arm_joints; i++)
    {
        xf[arm_position_indexes[i]] = arm_positions[i] + arm_speeds[i] * time_step;
    }

    // Arm joints speed
    for(uint i = 0; i < number_arm_joints; i++)
    {
        xf[arm_position_indexes[i] + number_arm_joints] = arm_speeds[i];
    }

    // Arm joints acceleration
    for(uint i = 0; i < number_arm_joints; i++)
    {
        xf[arm_position_indexes[i] + number_arm_joints * 2] =
            (arm_speeds[i] - arm_previous_speeds[i]) / time_step;
    }

    // Arm joints torques
    std::vector<double> G(number_arm_joints, 0);
    std::vector<std::vector<double>> I(number_arm_joints,
                                       std::vector<double>(number_arm_joints, 0));
    std::vector<std::vector<double>> C(number_arm_joints,
                                       std::vector<double>(number_arm_joints, 0));
    if(!getArmGravityMatrix(arm_positions, G) || !getArmInertiaMatrix(arm_positions, I) ||
       !getArmCoriolisMatrix(arm_positions, arm_speeds, C))
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::forwardIntegrateModel]: Failure computing arm "
                     "dynamic matrixes"
                  << NOCOLOR << std::endl;
        return false;
    }

    for(uint i = 0; i < number_arm_joints; i++)
    {
        xf[arm_position_indexes[i] + number_arm_joints * 3] =
            dot(I[i], arm_accelerations) + dot(C[i], arm_speeds) + G[i];
    }

    // Wheels torques
    xf[wheels_speed_indexes[0] + wheels_speed_indexes.size() * 2] =
        wheels_accelerations[0] *
            (getWheelInertia() + robot_weight / number_wheels * pow(wheels_radius, 2)) +
        rolling_resistance * robot_weight * gravity * wheels_radius / number_wheels;
    xf[wheels_speed_indexes[1] + wheels_speed_indexes.size() * 2] =
        wheels_accelerations[1] *
            (getWheelInertia() + robot_weight / number_wheels * pow(wheels_radius, 2)) +
        rolling_resistance * robot_weight * gravity * wheels_radius / number_wheels;

    if(robot_name == "exoter")
    {
        // B2EE (x, y, z, roll, pitch, yaw)
        for(uint i = 0; i < 3; i++)
        {
            xf[base_ee_pose_indexes[i]] = base_ee_pose[i] + dot(J[i], arm_speeds) * time_step;
        }

        xf[base_ee_pose_indexes[3]] = arm_positions[4];
        xf[base_ee_pose_indexes[4]] = arm_positions[1] + arm_positions[2] + arm_positions[3];
        xf[base_ee_pose_indexes[5]] = arm_positions[0];

        // BSpeed
        xf[base_speed_indexes[0]] = wheels_radius / 2 * (wheels_speed[0] + wheels_speed[1]);
        xf[base_speed_indexes[1]] = 0;
        xf[base_speed_indexes[2]] =
            wheels_radius * (wheels_speed[0] - wheels_speed[1]) / (2 * differential_width);

        // Wheels speeds
        xf[wheels_speed_indexes[0]] = wheels_speed[0];
        xf[wheels_speed_indexes[1]] = wheels_speed[1];

        // Wheels accelerations
        xf[wheels_speed_indexes[0] + wheels_speed_indexes.size()] =
            (wheels_speed[0] - wheels_previous_speed[0]) / time_step;
        xf[wheels_speed_indexes[1] + wheels_speed_indexes.size()] =
            (wheels_speed[1] - wheels_previous_speed[1]) / time_step;
    }
    else if(robot_name == "exoter_ack")
    {
        // B2EE (x, y, z, roll, pitch, yaw)
        for(uint i = 0; i < 3; i++)
        {
            xf[base_ee_pose_indexes[i]] = base_ee_pose[i] + dot(J[i], arm_speeds) * time_step;
        }

        xf[base_ee_pose_indexes[3]] = arm_positions[4];
        xf[base_ee_pose_indexes[4]] = arm_positions[1] + arm_positions[2] + arm_positions[3];
        xf[base_ee_pose_indexes[5]] = arm_positions[0];

        // Additionnal Ackermann model elements
        double left_wheel_steer = x[wheels_steering_indexes[0]];
        double left_wheel_steer_speed = u[steering_actuators_indexes[0]];
        double right_wheel_steer = getRightWheelSteer(left_wheel_steer);
        double steering_ratio =
            left_wheel_steer == 0 ? 1 : sin(left_wheel_steer) / sin(right_wheel_steer);

        // BSpeed
        xf[base_speed_indexes[0]] = wheels_radius / 2 *
                                    (cos(left_wheel_steer) * wheels_speed[0] +
                                     cos(right_wheel_steer) * wheels_previous_speed[1]);
        xf[base_speed_indexes[1]] = 0;
        xf[base_speed_indexes[2]] =
            base_speed[0] * tan(left_wheel_steer) /
            (differential_length + differential_width * tan(left_wheel_steer));

        // Wheels speeds
        xf[wheels_speed_indexes[0]] = wheels_speed[0];
        xf[wheels_speed_indexes[1]] = wheels_speed[0] * steering_ratio;

        // Wheels accelerations
        xf[wheels_speed_indexes[0] + wheels_speed_indexes.size()] =
            (wheels_speed[0] - wheels_previous_speed[0]) / time_step;
        xf[wheels_speed_indexes[1] + wheels_speed_indexes.size()] =
            (wheels_speed[0] - wheels_previous_speed[0]) / time_step * steering_ratio;

        // Steering joints position
        xf[wheels_steering_indexes[0]] = left_wheel_steer + left_wheel_steer_speed * time_step;
    }

    return true;
}

bool MobileManipulator::forwardIntegrateModel(const Eigen::VectorXd & x,
                                              const Eigen::VectorXd & u,
                                              double time_step,
                                              Eigen::VectorXd & xf)
{
    if(xf.size() != number_states || x.size() != number_states || u.size() != number_inputs)
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::forwardIntegrateModel]: The arguments doesn't "
                     "match the expected size"
                  << NOCOLOR << std::endl;
        return false;
    }

    std::vector<double> world_ee_pose;
    for(uint i = 0; i < world_ee_pose_indexes.size(); i++)
    {
        world_ee_pose.push_back(x(world_ee_pose_indexes[i]));
    }

    std::vector<double> arm_positions;
    for(uint i = 0; i < number_arm_joints; i++)
    {
        arm_positions.push_back(x(arm_position_indexes[i]));
    }

    std::vector<double> arm_speeds;
    for(uint i = 0; i < number_arm_joints; i++)
    {
        arm_speeds.push_back(u(arm_actuators_indexes[i]));
    }

    std::vector<double> arm_previous_speeds;
    for(uint i = 0; i < number_arm_joints; i++)
    {
        arm_previous_speeds.push_back(x(arm_position_indexes[i] + number_arm_joints));
    }

    std::vector<double> arm_accelerations;
    for(uint i = 0; i < number_arm_joints; i++)
    {
        arm_accelerations.push_back(x(arm_position_indexes[i] + number_arm_joints * 2));
    }

    std::vector<double> base_ee_pose;
    for(uint i = 0; i < base_ee_pose_indexes.size(); i++)
    {
        base_ee_pose.push_back(x(base_ee_pose_indexes[i]));
    }

    std::vector<double> robot_pose;
    for(uint i = 0; i < robot_pose_indexes.size(); i++)
    {
        robot_pose.push_back(x(robot_pose_indexes[i]));
    }

    double robot_yaw = x(robot_pose_indexes[2]);

    std::vector<double> base_speed;
    for(uint i = 0; i < base_speed_indexes.size(); i++)
    {
        base_speed.push_back(x(base_speed_indexes[i]));
    }

    std::vector<double> wheels_speed;
    for(uint i = 0; i < wheels_actuators_indexes.size(); i++)
    {
        wheels_speed.push_back(u(wheels_actuators_indexes[i]));
    }

    std::vector<double> wheels_previous_speed;
    for(uint i = 0; i < wheels_speed_indexes.size(); i++)
    {
        wheels_previous_speed.push_back(x(wheels_speed_indexes[i]));
    }

    std::vector<double> wheels_accelerations;
    for(uint i = 0; i < wheels_speed_indexes.size(); i++)
    {
        wheels_accelerations.push_back(x(wheels_speed_indexes[i] + wheels_speed_indexes.size()));
    }

    std::vector<std::vector<double>> J(6, std::vector<double>(number_arm_joints, 0));
    if(!getArmJacobianMatrix(arm_positions, J))
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::forwardIntegrateModel]: Failure while computing "
                     "jacobian matrix"
                  << NOCOLOR << std::endl;
        return false;
    }

    // W2EE (x, y, z, roll)
    xf(world_ee_pose_indexes[0]) =
        cos(robot_yaw) * base_ee_pose[2] - sin(robot_yaw) * base_ee_pose[1] + robot_pose[0];
    xf(world_ee_pose_indexes[1]) =
        sin(robot_yaw) * base_ee_pose[2] + cos(robot_yaw) * base_ee_pose[1] + robot_pose[1];
    xf(world_ee_pose_indexes[2]) = world_ee_pose[2] - dot(J[0], arm_speeds) * time_step;
    xf(world_ee_pose_indexes[3]) = robot_yaw + base_ee_pose[3];

    // W2C (x, y, yaw)
    xf(robot_pose_indexes[0]) = robot_pose[0] + cos(robot_yaw) * base_speed[0] * time_step -
                                sin(robot_yaw) * base_speed[1] * time_step;
    xf(robot_pose_indexes[1]) = robot_pose[1] + sin(robot_yaw) * base_speed[0] * time_step +
                                cos(robot_yaw) * base_speed[1] * time_step;
    xf(robot_pose_indexes[2]) = robot_pose[2] + base_speed[2] * time_step;

    // Arm joints position
    for(uint i = 0; i < number_arm_joints; i++)
    {
        xf(arm_position_indexes[i]) = arm_positions[i] + arm_speeds[i] * time_step;
    }

    // Arm joints speed
    for(uint i = 0; i < number_arm_joints; i++)
    {
        xf(arm_position_indexes[i] + number_arm_joints) = arm_speeds[i];
    }

    // Arm joints acceleration
    for(uint i = 0; i < number_arm_joints; i++)
    {
        xf(arm_position_indexes[i] + number_arm_joints * 2) =
            (arm_speeds[i] - arm_previous_speeds[i]) / time_step;
    }

    // Arm joints torques
    std::vector<double> G(number_arm_joints, 0);
    std::vector<std::vector<double>> I(number_arm_joints,
                                       std::vector<double>(number_arm_joints, 0));
    std::vector<std::vector<double>> C(number_arm_joints,
                                       std::vector<double>(number_arm_joints, 0));
    if(!getArmGravityMatrix(arm_positions, G) || !getArmInertiaMatrix(arm_positions, I) ||
       !getArmCoriolisMatrix(arm_positions, arm_speeds, C))
    {
        std::cout << RED
                  << "ERROR [MobileManipulator::forwardIntegrateModel]: Failure computing arm "
                     "dynamic matrixes"
                  << NOCOLOR << std::endl;
        return false;
    }

    for(uint i = 0; i < number_arm_joints; i++)
    {
        xf(arm_position_indexes[i] + number_arm_joints * 3) =
            dot(I[i], arm_accelerations) + dot(C[i], arm_speeds) + G[i];
    }

    // Wheels torques
    xf(wheels_speed_indexes[0] + wheels_speed_indexes.size() * 2) =
        wheels_accelerations[0] *
            (getWheelInertia() + robot_weight / number_wheels * pow(wheels_radius, 2)) +
        rolling_resistance * robot_weight * gravity * wheels_radius / number_wheels;
    xf(wheels_speed_indexes[1] + wheels_speed_indexes.size() * 2) =
        wheels_accelerations[1] *
            (getWheelInertia() + robot_weight / number_wheels * pow(wheels_radius, 2)) +
        rolling_resistance * robot_weight * gravity * wheels_radius / number_wheels;

    if(robot_name == "exoter")
    {
        // B2EE (x, y, z, roll, pitch, yaw)
        for(uint i = 0; i < 3; i++)
        {
            xf(base_ee_pose_indexes[i]) = base_ee_pose[i] + dot(J[i], arm_speeds) * time_step;
        }

        xf(base_ee_pose_indexes[3]) = arm_positions[4];
        xf(base_ee_pose_indexes[4]) = arm_positions[1] + arm_positions[2] + arm_positions[3];
        xf(base_ee_pose_indexes[5]) = arm_positions[0];

        // BSpeed
        xf(base_speed_indexes[0]) = wheels_radius / 2 * (wheels_speed[0] + wheels_speed[1]);
        xf(base_speed_indexes[1]) = 0;
        xf(base_speed_indexes[2]) =
            wheels_radius * (wheels_speed[0] - wheels_speed[1]) / (2 * differential_width);

        // Wheels speeds
        xf(wheels_speed_indexes[0]) = wheels_speed[0];
        xf(wheels_speed_indexes[1]) = wheels_speed[1];

        // Wheels accelerations
        xf(wheels_speed_indexes[0] + wheels_speed_indexes.size()) =
            (wheels_speed[0] - wheels_previous_speed[0]) / time_step;
        xf(wheels_speed_indexes[1] + wheels_speed_indexes.size()) =
            (wheels_speed[1] - wheels_previous_speed[1]) / time_step;
    }
    else if(robot_name == "exoter_ack")
    {
        // B2EE (x, y, z, roll, pitch, yaw)
        for(uint i = 0; i < 3; i++)
        {
            xf(base_ee_pose_indexes[i]) = base_ee_pose[i] + dot(J[i], arm_speeds) * time_step;
        }

        xf(base_ee_pose_indexes[3]) = arm_positions[4];
        xf(base_ee_pose_indexes[4]) = arm_positions[1] + arm_positions[2] + arm_positions[3];
        xf(base_ee_pose_indexes[5]) = arm_positions[0];

        // Additionnal Ackermann model elements
        double left_wheel_steer = x[wheels_steering_indexes[0]];
        double left_wheel_steer_speed = u[steering_actuators_indexes[0]];
        double right_wheel_steer = getRightWheelSteer(left_wheel_steer);
        double steering_ratio =
            left_wheel_steer == 0 ? 1 : sin(left_wheel_steer) / sin(right_wheel_steer);

        // BSpeed
        xf(base_speed_indexes[0]) = wheels_radius / 2 *
                                    (cos(left_wheel_steer) * wheels_speed[0] +
                                     cos(right_wheel_steer) * wheels_previous_speed[1]);
        xf(base_speed_indexes[1]) = 0;
        xf(base_speed_indexes[2]) =
            base_speed[0] * tan(left_wheel_steer) /
            (differential_length + differential_width * tan(left_wheel_steer));

        // Wheels speeds
        xf(wheels_speed_indexes[0]) = wheels_speed[0];
        xf(wheels_speed_indexes[1]) = wheels_speed[0] * steering_ratio;

        // Wheels accelerations
        xf(wheels_speed_indexes[0] + wheels_speed_indexes.size()) =
            (wheels_speed[0] - wheels_previous_speed[0]) / time_step;
        xf(wheels_speed_indexes[1] + wheels_speed_indexes.size()) =
            (wheels_speed[0] - wheels_previous_speed[0]) / time_step * steering_ratio;

        // Steering joints position
        xf(wheels_steering_indexes[0]) = left_wheel_steer + left_wheel_steer_speed * time_step;
    }

    return true;
}
