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

#ifndef __STATE_SPACE_MODEL__
#define __STATE_SPACE_MODEL__

#include <iostream>
#include <exception>
#include <vector>
#include <math.h>
#include "MatrixOperations.hpp"

#define pi 3.14159265359

namespace StateSpaceModels
{

/***************************************************************************
  Library containing state space models for different robots.

  CONSTRUCTORS:
  StateSpaceModels::MobileManipulator * robot_ss_model = 
                 new StateSpaceModels::MobileManipulator(string robot_name);

  "string robot_name" should contain the name of the robot model to be used.

  Available robot models: ExoTeR

***************************************************************************/

class MobileManipulator
{

private:

    //********************//
    // Dependency classes //
    //********************//

    //******************//
    // Model properties //
    //******************//

    // Indexes of the pose of the arm end effector from the world reference frame
    // Size: 3
    std::vector<uint> world_ee_pose_indexes;

    // Indexes of the pose of the arm end effector from its base reference frame
    // Size: 6
    std::vector<uint> base_ee_pose_indexes;

    // Indexes of the XY pose of the robot
    // Size: 2
    std::vector<uint> robot_pose_indexes;

    // Index of the yaw of the robot
    uint yaw_index;

    // Indexes of the XY and yaw speed of the robot
    // Size: 3
    std::vector<uint> base_speed_indexes;

    // Indexes of the arm joints position of the robot
    // Size: number_arm_joints
    std::vector<uint> arm_position_indexes;

    // Indexes of the wheels speed
    // Size: 2 (right and left speed trains, differential steering)
    std::vector<uint> wheels_speed_indexes;

    // Indexes of the arm joints speed control input of the robot
    // Size: number_arm_joints
    std::vector<uint> arm_actuators_indexes;

    // Indexes of the arm joints speed control input of the robot
    // Size: 2 (right and left speed trains, differential steering)
    std::vector<uint> wheels_actuators_indexes;

    // Indexes of the constrained states
    std::vector<uint> state_constrained_indexes;

    // Indexes of the constrained inputs
    std::vector<uint> input_constrained_indexes;
     
    // Indexes of the goal states with associated cost
    std::vector<uint> goal_states_indexes;
     
    // Indexes of the states with an associated cost during the whole motion
    std::vector<uint> whole_states_indexes;
     
    // Indexes of the inputs with an associated cost during the whole motion
    std::vector<uint> whole_inputs_indexes;

    //********************//
    // General parameters //
    //********************//

    // Wheels-to-ground rolling resistance
    double rolling_resistance;

    // Gravitational acceleration
    double gravity;

    //******************//
    // Robot Parameters //
    //******************//

    // Robot base shape
    double differential_width;
    double differential_length;

    // Robot base weight
    double robot_weight;

    // Robot base height from ground
    double robot_height;

    // Wheels shape
    double wheels_radius;
    double wheels_width;

    // Wheels weight
    double wheels_weight;

    // Nominal robot base speed
    double nominal_speed;

    // State limits (upper and lower)
    std::vector<double> state_limits;

    // Input limits (upper and lower)
    std::vector<double> input_limits;

    // Arm links lengths
    std::vector<double> arm_lengths;

    // Arm links masses
    std::vector<double> arm_masses;

    // Arm links width
    std::vector<double> arm_widths;

    // Offset robot center to arm base
    std::vector<double> position_offset_cb;
    std::vector<double> orientation_offset_cb;

    // Arm maximum reachability distance
    double arm_reachability;

    //********************************//
    // Model configuration parameters //
    //********************************//

    // Percentage of the time horizon where to start reducing the robot speed progressively
    double horizon_speed_reduction;

    // Distance to obstacles considered risky for the robot
    double risk_distance;

    // Distance to obstacles the robot should maintain to ensure safety
    double safety_distance;

    //*****************//
    // Quadratic costs //
    //*****************//

    // Map cost
    double obstacles_repulsive_cost;

    // State costs
    std::vector<double> goal_states_cost;
    double goal_speed_cost;

    std::vector<double> whole_states_cost;

    // Control input costs
    std::vector<double> whole_inputs_cost;

    // Extra costs
    double yaw_linearization_cost;

    //**********************//
    // Supporting variables //
    //**********************//
   
    // Size of the state vector
    uint number_states;

    // Size of the control input vector
    uint number_inputs;

    // Name of the robot
    std::string robot_name;

    // Numebr of manipulator degrees of freedom
    uint number_arm_joints;

    // Number of wheels of the robot base
    uint number_wheels;

    // Number of state input constraints
    uint number_si_constraints;

    // Number of pure state constraints
    uint number_ps_constraints;

    //**********************//
    // Supporting Functions //
    //**********************//

public:

    //*******************//
    // Class Constructor //
    //*******************//

    MobileManipulator(std::string _robot_name);

    ~MobileManipulator();

    //**********//
    // Get Data //
    //**********//

    // Returns the linearized state space model matrix A.
    // Size of A: number_states x number_states
    std::vector<std::vector<double>> getLinearizedMatrixA(
                                                   std::vector<double> x,
                                                   double time_step);

    // Returns the linearized state space model matrix B.
    // Size of B: number_states x number_inputs
    std::vector<std::vector<double>> getLinearizedMatrixB(
                                                   std::vector<double> x,
                                                   std::vector<double> u,
                                                   double time_step);


    // Returns the number of state input constraints.
    int getNumberStateInputConstraints();

    // Returns the state input constraint matrix C.
    // Size of C: number_si_constraints x number_states
    std::vector<std::vector<double>> getConstraintsMatrixC();

    // Returns the state input constraint matrix D.
    // Size of D: number_si_constraints x number_inputs
    std::vector<std::vector<double>> getConstraintsMatrixD();

    // Returns the state input constraint matrix r.
    // Size of r: number_si_constraints
    std::vector<double> getConstraintsMatrixR();


    // Returns the number of pure state constraints.
    int getNumberPureStateConstraints();

    // Returns the pure state constraint matrix G.
    // Size of G: number_ps_constraints x number_states
    std::vector<std::vector<double>> getConstraintsMatrixG();

    // Returns the pure state constraint matrix h.
    // Size of G: number_ps_constraints
    std::vector<double> getConstraintsMatrixH();


    // Returns the pure state cost matrix Q, in function of the time step provided
    // e.g. If percentage_horizon is 100, the goal state cost matrix will be returned
    // Size of Q: number_states x number_states
    std::vector<std::vector<double>> getStateCostMatrix(double percentage_horizon);

    // Returns the pure input cost matrix R.
    // Size of R: number_inputs x number_inputs
    std::vector<std::vector<double>> getInputCostMatrix();

    // Returns the state input cost matrix K.
    // Size of K: number_states x number_inputs
    std::vector<std::vector<double>> getStateInputCostMatrix();


    // Returns the gravity matrix given the arm state. 
    // Size of G: number_arm_joints
    std::vector<double> getArmGravityMatrix(std::vector<double> arm_positions);

    // Returns the inertia matrix given the arm state. 
    // Size of B: number_arm_joints x number_arm_joints
    std::vector<std::vector<double>> getArmInertiaMatrix(std::vector<double> arm_positions);

    // Returns the coriolis matrix given the arm state. 
    // Size of C: number_arm_joints x number_arm_joints
    std::vector<std::vector<double>> getArmCoriolisMatrix(std::vector<double> arm_positions,
                                                          std::vector<double> arm_speeds);

    // Returns the jacobian matrix given the arm state. 
    // Size of J: 6DoF x number_arm_joints
    std::vector<std::vector<double>> getArmJacobianMatrix(std::vector<double> arm_positions);

    // Returns the base-to-joint_index transform matrix given the arm state, 
    // using direct kinematics
    // Size of TBEE: 4 x 4
    std::vector<std::vector<double>> getDirectKinematicsTransform(
                                        std::vector<double> arm_positions,
                                        uint joint_index);


    // Returns the wheel inertia
    double getWheelInertia();


    // Returns the updated state after applying the control input u into the previous state x
    // over a time interval time_step.
    std::vector<double> forwardIntegrateModel(std::vector<double> x,
                                              std::vector<double> u,
                                              double time_step);

    //********************//
    // Checking Functions //
    //********************//

};
}

#endif
