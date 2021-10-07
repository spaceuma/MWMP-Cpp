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

        world_ee_pose_indexes = {0, 1, 2};
        base_ee_pose_indexes = {3, 4, 5, 6, 7, 8};

        rover_pose_indexes = {9, 10};
        yaw_index = 11;

        base_speed_indexes = {12, 13, 14};

        arm_position_indexes = {15, 16, 17, 18, 19};
        arm_actuators_indexes = {0, 1, 2, 3, 4};

        wheels_speed_indexes = {35,36};
        wheels_actuators_indexes = {5,6};

        state_constrained_indexes = {15,16,17,18,19,39,40};
        input_constrained_indexes = {0,1,2,3,4};

        goal_states_indexes = {0,1,2,15,7,19};
        whole_states_indexes = {9,10,11,30,31,32,33,34,39,40};

        whole_inputs_indexes = {0,1,2,3,4,5,6};

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

        state_limits = {90*pi/180, -30*pi/180, -10*pi/180, -190*pi/180, 160*pi/180, -160*pi/180, 250*pi/180,
                        -70*pi/180, 160*pi/180, -160*pi/180, 2.85, -2.85, 2.85, -2.85};

        input_limits = {0.01, -0.01, 0.01, -0.01, 0.01,
                        -0.01, 0.01, -0.01, 0.01, -0.01};

        arm_lengths = {0.0895, 0.206, 0.176, 0.0555, 0.14};
        arm_masses = {0.5, 0.8, 0.8, 0.3, 0.2};
        arm_widths = {0.055, 0.02, 0.02, 0.05, 0.02};

        position_offset_cb = {0.165, -0.15, 0.028};
        orientation_offset_cb = {0.0, pi/2, 0.0};

        arm_reachability = 0.65;

        //********************************//
        // Model configuration parameters //
        //********************************//
        horizon_speed_reduction = 90;

        risk_distance = 0.30;
        safety_distance = 1.00;

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
        number_states = 41;
        number_inputs = 7;

        number_arm_joints = 5;
        number_wheels = 6;

        number_si_constraints = 10;
        number_ps_constraints = 14;
    }
}

std::vector<std::vector<double>> MobileManipulator::getLinearizedMatrixA(
                                                std::vector<double> x,
                                                double time_step)
{
    std::vector<std::vector<double>> A(number_states,std::vector<double>(number_states,0));

    double rover_yaw = x[yaw_index];

    std::vector<double> rover_speed = {x[base_speed_indexes[0]], x[base_speed_indexes[1]], x[base_speed_indexes[2]]};

    std::vector<double> arm_positions;
    for(uint i = 0; i < number_arm_joints; i++)
    {
        arm_positions.push_back(x[arm_position_indexes[i]]);
    }

    // W2EEx
    A[world_ee_pose_indexes[0]][base_ee_pose_indexes[2]] = cos(rover_yaw);
    A[world_ee_pose_indexes[0]][base_ee_pose_indexes[1]] = -sin(rover_yaw);
    A[world_ee_pose_indexes[0]][rover_pose_indexes[0]] = 1;

    // W2EEy
    A[world_ee_pose_indexes[1]][base_ee_pose_indexes[2]] = sin(rover_yaw);
    A[world_ee_pose_indexes[1]][base_ee_pose_indexes[1]] = cos(rover_yaw);
    A[world_ee_pose_indexes[1]][rover_pose_indexes[1]]  = 1;

    // W2EEz
    A[world_ee_pose_indexes[2]][world_ee_pose_indexes[2]] = 1;

    // B2EE
    for(uint i = 0; i < base_ee_pose_indexes.size(); i ++)
    {
        A[base_ee_pose_indexes[i]][base_ee_pose_indexes[i]] = 1;
    }

    // W2Cx
    A[rover_pose_indexes[0]][rover_pose_indexes[0]] = 1;
    A[rover_pose_indexes[0]][yaw_index] = time_step*
                         (-sin(rover_yaw)*rover_speed[0]*yaw_linearization_cost -
                           cos(rover_yaw)*rover_speed[1]*yaw_linearization_cost);
    A[rover_pose_indexes[0]][base_speed_indexes[0]] = time_step*(cos(rover_yaw) +
                          sin(rover_yaw)*rover_yaw*yaw_linearization_cost);
    A[rover_pose_indexes[0]][base_speed_indexes[1]] = -time_step*(sin(rover_yaw) -
                           cos(rover_yaw)*rover_yaw*yaw_linearization_cost);

    // W2Cy
    A[rover_pose_indexes[1]][rover_pose_indexes[1]] = 1;
    A[rover_pose_indexes[1]][yaw_index] = time_step*(cos(rover_yaw)*rover_speed[0]*yaw_linearization_cost -
                           sin(rover_yaw)*rover_speed[1]*yaw_linearization_cost);
    A[rover_pose_indexes[1]][base_speed_indexes[0]] = time_step*(sin(rover_yaw) -
                           cos(rover_yaw)*rover_yaw*yaw_linearization_cost);
    A[rover_pose_indexes[1]][base_speed_indexes[1]] = time_step*(cos(rover_yaw) +
                           sin(rover_yaw)*rover_yaw*yaw_linearization_cost);

    // W2C Heading
    A[yaw_index][yaw_index] = 1;
    A[yaw_index][base_speed_indexes[2]] = time_step;

    // Arm joints position
    for(uint i = 0; i < number_arm_joints; i++)
    {
        A[arm_position_indexes[i]][arm_position_indexes[i]] = 1;
    }

    // Arm joints acceleration
    for(uint i = 0; i < number_arm_joints; i++)
    {
        A[arm_position_indexes[i]+number_arm_joints*2]
         [arm_position_indexes[i]+number_arm_joints] = -1/time_step;
    }

    // Arm joints torques
    std::vector<std::vector<double>> B = getArmInertiaMatrix(arm_positions);
    for(uint i = 0; i < number_arm_joints; i++)
    {
        for(uint j = 0; j < number_arm_joints; j++)
        {
            A[arm_position_indexes[i]+number_arm_joints*3]
             [arm_position_indexes[0]+number_arm_joints*2+j] = B[i][j];
        }
    }

    // Wheels accelerations
    for(uint i = 0; i < wheels_speed_indexes.size(); i++)
    {
        A[wheels_speed_indexes[i]+wheels_speed_indexes.size()]
         [wheels_speed_indexes[i]] = -1/time_step;
    }

    // Wheels torques
    for(uint i = 0; i < wheels_speed_indexes.size(); i++)
    {
        A[wheels_speed_indexes[i]+wheels_speed_indexes.size()*2]
         [wheels_speed_indexes[i]+wheels_speed_indexes.size()] = getWheelInertia() +
                                robot_weight/number_wheels*pow(wheels_radius,2);
    }

    return A;
}

std::vector<std::vector<double>> MobileManipulator::getLinearizedMatrixB(
                                                std::vector<double> x,
                                                std::vector<double> u,
                                                double time_step)
{
    std::vector<std::vector<double>> B(number_states,std::vector<double>(number_states,0));

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
    std::vector<std::vector<double>> J = getArmJacobianMatrix(arm_positions);

    for(uint i = 0; i < number_arm_joints; i++)
    {
        B[world_ee_pose_indexes[2]][arm_actuators_indexes[i]] = -time_step*J[1][i];
    }

    // BTEE
    for(uint i = 0; i < number_arm_joints; i++)
    {
        for(uint j = 0; j < number_arm_joints; j++)
        {
            B[base_ee_pose_indexes[i]][arm_actuators_indexes[j]] = -time_step*J[i][j];
        }
    }

    // W2C Speed x
    B[base_speed_indexes[0]][wheels_actuators_indexes[0]] = wheels_radius/2;
    B[base_speed_indexes[0]][wheels_actuators_indexes[1]] = wheels_radius/2;

    // W2C Speed heading
    B[base_speed_indexes[2]][wheels_actuators_indexes[0]] =
                                        wheels_radius/(2*differential_width);
    B[base_speed_indexes[2]][wheels_actuators_indexes[1]] =
                                       -wheels_radius/(2*differential_width);

    // Arm joints position
    for(uint i = 0; i < number_arm_joints; i++)
    {
        B[arm_position_indexes[i]][arm_actuators_indexes[i]] = time_step;
    }

    // Arm joints speed
    for(uint i = 0; i < number_arm_joints; i++)
    {
        B[arm_position_indexes[i]+number_arm_joints][arm_actuators_indexes[i]] = 1;
    }

    // Arm joints acceleration
    for(uint i = 0; i < number_arm_joints; i++)
    {
        B[arm_position_indexes[i]+number_arm_joints*2][arm_actuators_indexes[i]] = 1/time_step;
    }
    
    // Arm joints torques
    std::vector<std::vector<double>> C = getArmCoriolisMatrix(arm_positions, arm_actuators);
    for(uint i = 0; i < number_arm_joints; i++)
    {
        for(uint j = 0; j < number_arm_joints; j++)
        {
            B[arm_position_indexes[i]+number_arm_joints*3][arm_actuators_indexes[j]] = C[i][j];
        }
    }

    // Wheels speeds
    for(uint i = 0; i < wheels_speed_indexes.size(); i++)
    {
        B[wheels_speed_indexes[i]]
         [wheels_actuators_indexes[i]] = 1;
    }

    // Wheels speeds
    for(uint i = 0; i < wheels_speed_indexes.size(); i++)
    {
        B[wheels_speed_indexes[i] + wheels_speed_indexes.size()]
         [wheels_actuators_indexes[i]] = 1/time_step;
    }

    return B;
}

int MobileManipulator::getNumberStateInputConstraints()
{
    return number_si_constraints;
}

std::vector<std::vector<double>> MobileManipulator::getConstraintsMatrixC()
{
    std::vector<std::vector<double>> C(number_si_constraints,
                                       std::vector<double>(number_states,0));

    return C;
}

std::vector<std::vector<double>> MobileManipulator::getConstraintsMatrixD()
{
    std::vector<std::vector<double>> D(number_si_constraints,
                                       std::vector<double>(number_inputs,0));

    for(uint i = 0; i < number_si_constraints; i+=2)
    {
        D[i][input_constrained_indexes[i/2]] = -1;
        D[i+1][input_constrained_indexes[i/2]] = 1;
    }

    return D;
}


std::vector<double> MobileManipulator::getConstraintsMatrixR()
{
    std::vector<double> r(number_si_constraints,0);

    for(uint i = 0; i < number_si_constraints; i+=2)
    {
        r[i] = -input_limits[i];
        r[i+1] = input_limits[i+1];
    }

    return r;
}

int MobileManipulator::getNumberPureStateConstraints()
{
    return number_ps_constraints;
}

std::vector<std::vector<double>> MobileManipulator::getConstraintsMatrixG()
{
    std::vector<std::vector<double>> G(number_ps_constraints,
                                       std::vector<double>(number_states,0));

    for(uint i = 0; i < number_ps_constraints; i+=2)
    {
        G[i][input_constrained_indexes[i/2]] = -1;
        G[i+1][input_constrained_indexes[i/2]] = 1;
    }

    return G;
}

std::vector<double> MobileManipulator::getConstraintsMatrixH()
{
    std::vector<double> h(number_ps_constraints,0);

    for(uint i = 0; i < number_ps_constraints; i+=2)
    {
        h[i] = -state_limits[i];
        h[i+1] = state_limits[i+number_ps_constraints];
    }

    return h;
}

std::vector<std::vector<double>> MobileManipulator::getStateCostMatrix(double percentage_horizon)
{
    std::vector<std::vector<double>> Q(number_states,
                                       std::vector<double>(number_states,0));

    for(uint i = 0; i < whole_states_indexes.size(); i++)
    {
        Q[whole_states_indexes[i]][whole_states_indexes[i]] = whole_states_cost[i];
    }

    if(percentage_horizon == 100)
    {
        for(uint i = 0; i < goal_states_indexes.size(); i++)
        {
            Q[goal_states_indexes[i]][goal_states_indexes[i]] = goal_states_cost[i];
        }
    }
    else if (percentage_horizon > horizon_speed_reduction)
    {
        double linear_cost = (percentage_horizon - horizon_speed_reduction)/
                             (100 - horizon_speed_reduction);

        for(uint i = 0; i < base_speed_indexes.size(); i++)
        {
            Q[base_speed_indexes[i]][base_speed_indexes[i]] = linear_cost*goal_speed_cost;
        }
    }


    return Q;
}

std::vector<std::vector<double>> MobileManipulator::getInputCostMatrix()
{
    std::vector<std::vector<double>> R(number_inputs,
                                       std::vector<double>(number_inputs,0));

    for(uint i = 0; i < whole_inputs_indexes.size(); i++)
    {
        R[whole_inputs_indexes[i]][whole_inputs_indexes[i]] = whole_inputs_cost[i];
    }

    return R;
}

std::vector<std::vector<double>> MobileManipulator::getStateInputCostMatrix()
{
    std::vector<std::vector<double>> K(number_states,
                                       std::vector<double>(number_inputs,0));

    return K;
}

std::vector<double> MobileManipulator::getArmGravityMatrix(std::vector<double> arm_positions)
{
    std::vector<double> G(number_arm_joints,0);

    if(robot_name == "exoter")
    {
        //TODO This should be obtained from a URDF file, not hardcoded
        G[0] = 2.45e-4*sin(arm_positions[0])*
               (333.0*sin(arm_positions[1] + arm_positions[2] + arm_positions[3]) +
               5630.0*cos(arm_positions[1] + arm_positions[2]) + 1.4e+4*cos(arm_positions[1]));

        G[1] = 0.404*sin(arm_positions[0] + arm_positions[1]) -
               0.404*sin(arm_positions[0] - 1.0*arm_positions[1]) +
               2.63*cos(arm_positions[0])*sin(arm_positions[1]) +
               1.38*sin(arm_positions[1] + arm_positions[2])*cos(arm_positions[0]) -
               0.0817*cos(arm_positions[1] + 
               arm_positions[2])*cos(arm_positions[0])*cos(arm_positions[3]) +
               0.0817*sin(arm_positions[1] + arm_positions[2])*
               cos(arm_positions[0])*sin(arm_positions[3]);

        G[2] = -2.45e-4*cos(arm_positions[0])*(333.0*cos(arm_positions[1] +
               arm_positions[2] + arm_positions[3]) -
               5630.0*sin(arm_positions[1] + arm_positions[2]));

        G[3] = -0.136*cos(arm_positions[1] + arm_positions[2] + arm_positions[3])*
               cos(arm_positions[0]);

        G[4] = 0;
    }
    return G;
}

std::vector<std::vector<double>> MobileManipulator::getArmInertiaMatrix(std::vector<double> arm_positions)
{
    std::vector<std::vector<double>> B(number_arm_joints,
                                       std::vector<double>(number_arm_joints,0));

    if(robot_name == "exoter")
    {
        //TODO This should be obtained from a URDF file, not hardcoded
        B[0][0] = 0.029*cos(2.0*arm_positions[1] + arm_positions[2]) + 
                  0.00147*sin(2.0*arm_positions[1] + 2.0*arm_positions[2] + arm_positions[3]) +
                  0.0318*cos(2.0*arm_positions[1]) +
                  0.00171*sin(arm_positions[2] + arm_positions[3]) +
                  0.00171*sin(2.0*arm_positions[1] + arm_positions[2] + arm_positions[3]) +
                  0.029*cos(arm_positions[2]) + 0.00147*sin(arm_positions[3]) +
                  0.00852*cos(2.0*arm_positions[1] + 2.0*arm_positions[2]) -
                  3.17e-4*cos(2.0*arm_positions[1] + 2.0*arm_positions[2] +
                              2.0*arm_positions[3]) + 0.047;

        B[0][4] = 4.0e-5*cos(arm_positions[1] + arm_positions[2] + arm_positions[3]);

        B[1][1] = 0.00343*sin(arm_positions[2] + arm_positions[3]) +
                  0.058*cos(arm_positions[2]) + 0.00293*sin(arm_positions[3]) + 0.0821;

        B[1][2] = 0.00171*sin(arm_positions[2] + arm_positions[3]) +
                  0.029*cos(arm_positions[2]) + 0.00293*sin(arm_positions[3]) + 0.0182;

        B[1][3] = 0.00286*sin(arm_positions[2] + arm_positions[3]) +
                  0.00195*sin(arm_positions[3]) + 0.00105;

        B[2][1] = 0.00171*sin(arm_positions[2] + arm_positions[3]) +
                  0.029*cos(arm_positions[2]) + 0.00293*sin(arm_positions[3]) + 0.0182;

        B[2][2] = 0.00293*sin(arm_positions[3]) + 0.0182;

        B[2][3] = 0.00195*sin(arm_positions[3]) + 0.00105;

        B[3][1] = 0.00286*sin(arm_positions[2] + arm_positions[3]) +
                  0.00195*sin(arm_positions[3]) + 0.00105;

        B[3][2] = 0.00195*sin(arm_positions[3]) + 0.00105;

        B[3][3] = 0.0012;

        B[4][0] = 4.0e-5*cos(arm_positions[1] + arm_positions[2] + arm_positions[3]);

        B[4][4] = 4.0e-5;
    }

    return B;
}

std::vector<std::vector<double>> MobileManipulator::getArmCoriolisMatrix(
                                                      std::vector<double> arm_positions,
                                                      std::vector<double> arm_speeds)
{
    std::vector<std::vector<double>> C(number_arm_joints,
                                       std::vector<double>(number_arm_joints,0));

    if(robot_name == "exoter")
    {
        //TODO This should be obtained from a URDF file, not hardcoded
        C[0][1] = 0.00293*arm_speeds[0]*cos(2.0*arm_positions[1] + 2.0*arm_positions[2] +
                                            arm_positions[3]) -
                  0.058*arm_speeds[0]*sin(2.0*arm_positions[1] + arm_positions[2]) -
                  0.0637*arm_speeds[0]*sin(2.0*arm_positions[1]) +
                  0.00343*arm_speeds[0]*cos(2.0*arm_positions[1] + arm_positions[2] +
                                            arm_positions[3]) -
                  0.017*arm_speeds[0]*sin(2.0*arm_positions[1] + 2.0*arm_positions[2]) +
                  6.35e-4*arm_speeds[0]*sin(2.0*arm_positions[1] + 2.0*arm_positions[2] +
                                            2.0*arm_positions[3]) -
                  4.0e-5*arm_speeds[4]*sin(arm_positions[1] + arm_positions[2] +
                                           arm_positions[3]);

        C[0][2] = 0.00293*arm_speeds[0]*cos(2.0*arm_positions[1] + 2.0*arm_positions[2] +
                                            arm_positions[3]) -
                  0.029*arm_speeds[0]*sin(2.0*arm_positions[1] + arm_positions[2]) +
                  0.00171*arm_speeds[0]*cos(arm_positions[2] + arm_positions[3]) +
                  0.00171*arm_speeds[0]*cos(2.0*arm_positions[1] + arm_positions[2] +
                                            arm_positions[3]) -
                  0.029*arm_speeds[0]*sin(arm_positions[2]) -
                  0.017*arm_speeds[0]*sin(2.0*arm_positions[1] + 2.0*arm_positions[2]) +
                  6.35e-4*arm_speeds[0]*sin(2.0*arm_positions[1] + 2.0*arm_positions[2] +
                                            2.0*arm_positions[3]) -
                  4.0e-5*arm_speeds[4]*sin(arm_positions[1] + arm_positions[2] +
                                           arm_positions[3]);

        C[0][3] = 0.00147*arm_speeds[0]*cos(2.0*arm_positions[1] + 2.0*arm_positions[2] +
                                            arm_positions[3]) +
                  0.00171*arm_speeds[0]*cos(arm_positions[2] + arm_positions[3]) +
                  0.00171*arm_speeds[0]*cos(2.0*arm_positions[1] + arm_positions[2] +
                                            arm_positions[3]) +
                  0.00147*arm_speeds[0]*cos(arm_positions[3]) +
                  6.35e-4*arm_speeds[0]*sin(2.0*arm_positions[1] + 2.0*arm_positions[2] +
                                            2.0*arm_positions[3]) -
                  4.0e-5*arm_speeds[4]*sin(arm_positions[1] + arm_positions[2] +
                                           arm_positions[3]);

        C[1][0] = 0.029*arm_speeds[0]*sin(2.0*arm_positions[1] + arm_positions[2]) -
                  0.00147*arm_speeds[0]*cos(2.0*arm_positions[1] + 2.0*arm_positions[2] +
                                            arm_positions[3]) +
                  0.0318*arm_speeds[0]*sin(2.0*arm_positions[1]) -
                  0.00171*arm_speeds[0]*cos(2.0*arm_positions[1] + arm_positions[2] +
                                            arm_positions[3]) +
                  0.00852*arm_speeds[0]*sin(2.0*arm_positions[1] + 2.0*arm_positions[2]) -
                  3.17e-4*arm_speeds[0]*sin(2.0*arm_positions[1] + 2.0*arm_positions[2] +
                                            2.0*arm_positions[3]) +
                  2.0e-5*arm_speeds[4]*sin(arm_positions[1] + arm_positions[2] +
                                           arm_positions[3]);

        C[1][2] = 0.00343*arm_speeds[1]*cos(arm_positions[2] + arm_positions[3]) +
                  0.00171*arm_speeds[2]*cos(arm_positions[2] + arm_positions[3]) +
                  0.00286*arm_speeds[3]*cos(arm_positions[2] + arm_positions[3]) -
                  0.058*arm_speeds[1]*sin(arm_positions[2]) -
                  0.029*arm_speeds[2]*sin(arm_positions[2]);

        C[1][3] = 0.00343*arm_speeds[1]*cos(arm_positions[2] + arm_positions[3]) +
                  0.00171*arm_speeds[2]*cos(arm_positions[2] + arm_positions[3]) +
                  0.00286*arm_speeds[3]*cos(arm_positions[2] + arm_positions[3]) +
                  0.00293*arm_speeds[1]*cos(arm_positions[3]) +
                  0.00293*arm_speeds[2]*cos(arm_positions[3]) +
                  0.00195*arm_speeds[3]*cos(arm_positions[3]);

        C[1][4] = 2.0e-5*arm_speeds[0]*sin(arm_positions[1] + arm_positions[2] +
                                           arm_positions[3]);

        C[2][0] = 0.0145*arm_speeds[0]*sin(2.0*arm_positions[1] + arm_positions[2]) -
                  0.00147*arm_speeds[0]*cos(2.0*arm_positions[1] + 2.0*arm_positions[2] +
                                            arm_positions[3]) -
                  8.57e-4*arm_speeds[0]*cos(arm_positions[2] + arm_positions[3]) -
                  8.57e-4*arm_speeds[0]*cos(2.0*arm_positions[1] + arm_positions[2] +
                                            arm_positions[3]) +
                  0.0145*arm_speeds[0]*sin(arm_positions[2]) +
                  0.00852*arm_speeds[0]*sin(2.0*arm_positions[1] + 2.0*arm_positions[2]) -
                  3.17e-4*arm_speeds[0]*sin(2.0*arm_positions[1] + 2.0*arm_positions[2] +
                                            2.0*arm_positions[3]) +
                  2.0e-5*arm_speeds[4]*sin(arm_positions[1] + arm_positions[2] +
                                            arm_positions[3]);

        C[2][1] = 0.029*arm_speeds[1]*sin(arm_positions[2]) -
                  8.57e-4*arm_speeds[2]*cos(arm_positions[2] + arm_positions[3]) -
                  0.00143*arm_speeds[3]*cos(arm_positions[2] + arm_positions[3]) -
                  0.00171*arm_speeds[1]*cos(arm_positions[2] + arm_positions[3]) +
                  0.0145*arm_speeds[2]*sin(arm_positions[2]);

        C[2][2] = 2.58e-6*arm_speeds[1]*(333.0*cos(arm_positions[2] + arm_positions[3]) -
                  5630.0*sin(arm_positions[2]));

        C[2][3] = 2.86e-4*arm_speeds[1]*cos(arm_positions[2] + arm_positions[3]) +
                  0.00293*arm_speeds[1]*cos(arm_positions[3]) +
                  0.00293*arm_speeds[2]*cos(arm_positions[3]) +
                  0.00195*arm_speeds[3]*cos(arm_positions[3]);

        C[2][4] = 2.0e-5*arm_speeds[0]*sin(arm_positions[1] + arm_positions[2] +
                                           arm_positions[3]);

        C[3][0] = 2.0e-5*arm_speeds[4]*sin(arm_positions[1] + arm_positions[2] +
                                           arm_positions[3]) -
                  8.57e-4*arm_speeds[0]*cos(arm_positions[2] + arm_positions[3]) -
                  8.57e-4*arm_speeds[0]*cos(2.0*arm_positions[1] + arm_positions[2] +
                                            arm_positions[3]) -
                  7.33e-4*arm_speeds[0]*cos(arm_positions[3]) -
                  3.17e-4*arm_speeds[0]*sin(2.0*arm_positions[1] + 2.0*arm_positions[2] +
                                            2.0*arm_positions[3]) -
                  7.33e-4*arm_speeds[0]*cos(2.0*arm_positions[1] + 2.0*arm_positions[2] +
                                            arm_positions[3]);

        C[3][1] = - 0.00171*arm_speeds[1]*cos(arm_positions[2] + arm_positions[3]) -
                    8.57e-4*arm_speeds[2]*cos(arm_positions[2] + arm_positions[3]) -
                    0.00143*arm_speeds[3]*cos(arm_positions[2] + arm_positions[3]) -
                    0.00147*arm_speeds[1]*cos(arm_positions[3]) -
                    0.00147*arm_speeds[2]*cos(arm_positions[3]) -
                    9.77e-4*arm_speeds[3]*cos(arm_positions[3]);

        C[3][2] = 0.002*arm_speeds[1]*cos(arm_positions[2] + arm_positions[3]) -
                  0.00147*arm_speeds[1]*cos(arm_positions[3]) -
                  0.00147*arm_speeds[2]*cos(arm_positions[3]) -
                  9.77e-4*arm_speeds[3]*cos(arm_positions[3]);
        C[3][3] = 0.00143*arm_speeds[1]*cos(arm_positions[2] + arm_positions[3]) +
                  9.77e-4*arm_speeds[1]*cos(arm_positions[3]) +
                  9.77e-4*arm_speeds[2]*cos(arm_positions[3]);

        C[3][4] = 2.0e-5*arm_speeds[0]*sin(arm_positions[1] + arm_positions[2] +
                                           arm_positions[3]);

        C[4][1] = -4.0e-5*arm_speeds[0]*sin(arm_positions[1] + arm_positions[2] +
                                            arm_positions[3]);

        C[4][2] = -4.0e-5*arm_speeds[0]*sin(arm_positions[1] + arm_positions[2] +
                                            arm_positions[3]);

        C[4][3] = -4.0e-5*arm_speeds[0]*sin(arm_positions[1] + arm_positions[2] +
                                            arm_positions[3]);
    }

    return C;
}

std::vector<std::vector<double>> MobileManipulator::getArmJacobianMatrix(
                                                    std::vector<double> arm_positions)
{
    std::vector<std::vector<double>> J(6,
                                       std::vector<double>(number_arm_joints,0));
    if(robot_name == "exoter")
    {
        //TODO This should be obtained from a URDF file, not hardcoded
    }


    return J;
}

std::vector<std::vector<double>> MobileManipulator::getDirectKinematicsTransform(
                                                    std::vector<double> arm_positions,
                                                    uint joint_index)
{
    if(robot_name == "exoter")
    {
        //TODO This should be obtained from a URDF file, not hardcoded
        if(joint_index > number_arm_joints)
        {
            throw std::domain_error( "[StateSpaceModel] ERROR: index provided greater than the number of joints");
        }

        std::vector<std::vector<double>> TB0(4,std::vector<double>(4,0));

        TB0 = getTraslation({0, 0, 0});
        if(joint_index == 0)
        {
            return TB0;
        }

        std::vector<std::vector<double>> TB1(4,std::vector<double>(4,0));

        std::vector<std::vector<double>> T01 = dot(getTraslation({0, 0,
                                                                  arm_lengths[0]}),
                                               dot(getZrot(arm_positions[0]),
                                               dot(getTraslation({0, 0, 0}),
                                                   getXrot(-pi/2))));

        TB1 = dot(TB0,T01);
        if(joint_index == 1)
        {
            return TB1;
        }

        std::vector<std::vector<double>> TB2(4,std::vector<double>(4,0));

        std::vector<std::vector<double>> T12 = dot(getTraslation({0, 0, 0}),
                                               dot(getZrot(arm_positions[1]),
                                               dot(getTraslation({arm_lengths[1],
                                                                  0, 0}),
                                                   getXrot(0))));

        TB2 = dot(TB1,T12);

        if(joint_index == 2)
        {
            return TB2;
        }
               
        std::vector<std::vector<double>> TB3(4,std::vector<double>(4,0));

        std::vector<std::vector<double>> T23 = dot(getTraslation({0, 0, 0}),
                                               dot(getZrot(arm_positions[2]),
                                               dot(getTraslation({arm_lengths[2],
                                                                  0, 0}),
                                                   getXrot(0))));

        TB3 = dot(TB2,T23);
        if(joint_index == 3)
        {
            return TB3;
        }
               
        std::vector<std::vector<double>> TB4(4,std::vector<double>(4,0));

        std::vector<std::vector<double>> T34 = dot(getTraslation({0, 0, 0}),
                                               dot(getZrot(arm_positions[3]),
                                               dot(getTraslation({0, 0, 0}),
                                                   getXrot(pi/2))));

        TB4 = dot(TB3,T34);
        if(joint_index == 4)
        {
            return TB4;
        }

        std::vector<std::vector<double>> TB5(4,std::vector<double>(4,0));

        std::vector<std::vector<double>> T45 = dot(getTraslation({0,
                                                 0, arm_lengths[3]+arm_lengths[4]}),
                                               dot(getZrot(arm_positions[4]),
                                               dot(getTraslation({0, 0, 0}),
                                                   getXrot(0))));

        TB5 = dot(TB4,T45);
        return TB5;
    }

    return std::vector<std::vector<double>>(4,std::vector<double>(4,0));
}
