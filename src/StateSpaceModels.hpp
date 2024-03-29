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

#include "MatrixOperations.hpp"
#include <exception>
#include <iostream>
#include <math.h>
#include <vector>

#define pi 3.14159265359

#define NOCOLOR "\033[0m"
#define BLACK "\033[1;30m"
#define RED "\033[1;31m"
#define GREEN "\033[1;32m"
#define YELLOW "\033[1;33m"
#define BLUE "\033[1;34m"
#define MAGENTA "\033[1;35m"
#define CYAN "\033[1;36m"
#define WHITE "\033[1;37m"

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
    //******************//
    // Model properties //
    //******************//

    // Indexes of the distance goal states. Size: 3 (x, y, z)
    std::vector<uint> goal_distance_indexes;

    // Indexes of the orientation goal states. Size: 3 (roll, pitch, yaw)
    std::vector<uint> goal_orientation_indexes;

    // Indexes of the pose of the arm end effector from the world reference frame
    // Size: 3 (x, y, z)
    std::vector<uint> world_ee_pose_indexes;

    // Indexes of the pose of the arm end effector from its base reference frame
    // Size: 6 (x, y, z, roll, pitch, yaw)
    std::vector<uint> base_ee_pose_indexes;

    // Indexes of the XY pose of the robot
    // Size: 3 (x, y, yaw)
    std::vector<uint> robot_pose_indexes;

    // Indexes of the XY and yaw speed of the robot
    // Size: 3 (x, y, yaw)
    std::vector<uint> base_speed_indexes;

    // Indexes of the arm joints position of the robot
    // Size: number_arm_joints
    std::vector<uint> arm_position_indexes;

    // Indexes of the wheels speed
    // Size: 2 (right and left speed trains)
    std::vector<uint> wheels_speed_indexes;

    // Indexes of the wheels steering angle
    // Size: 1 (the rest steering angles are geometrically dependant)
    std::vector<uint> wheels_steering_indexes;

    // Indexes of the arm joints speed control input of the robot
    // Size: number_arm_joints
    std::vector<uint> arm_actuators_indexes;

    // Indexes of the wheels speed control input
    // Size: 2 (right and left speed trains)
    std::vector<uint> wheels_actuators_indexes;

    // Indexes of the wheels steering speed control input
    // Size: 1 (the rest steering angles are geometrically dependant)
    std::vector<uint> steering_actuators_indexes;

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

    // Distance to goal threshold to consider convergence is reached
    double distance_threshold;

    // Goal orientation threshold to consider convergence is reached
    double orientation_threshold;

    // Percentage of the time horizon where to start reducing the robot speed progressively
    double horizon_speed_reduction;

    // Percentage of the time horizon where to start reducing the robot arm acceleration
    double horizon_arm_acceleration_reduction;

    // Hardness of reduction of the robot arm acceleration
    double hardness_arm_acceleration_reduction;

    // Distance to obstacles considered risky for the robot
    double risk_distance;

    // Distance to obstacles the robot should maintain to ensure safety
    double safety_distance;

    // Minimum time horizon in which convergence is ensured
    double min_time_horizon;

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
    double steering_forward_linearization_cost;
    double steering_angular_linearization_cost;

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

public:
    //*******************//
    // Class Constructor //
    //*******************//

    MobileManipulator(std::string _robot_name);

    ~MobileManipulator();

    //**********//
    // Get Data //
    //**********//

    // Returns the number of states in the state vector.
    uint getNumberStates();

    // Returns the number of inputs in the state vector.
    uint getNumberInputs();

    // Returns the number of arm joints.
    uint getNumberArmJoints();

    // Returns the type of steering.
    std::string getSteeringType();

    // Returns the indexes where to check for distance to the goal.
    std::vector<uint> getIndexesGoalDistance();

    // Returns the threshold distance to the goal.
    double getThresholdGoalDistance();

    // Returns the indexes where to check for goal orientation.
    std::vector<uint> getIndexesGoalOrientation();

    // Returns the threshold orientation to the goal.
    double getThresholdGoalOrientation();

    // Returns the indexes where to check for the robot pose.
    std::vector<uint> getIndexesRobotPose();

    // Return the end effector pose given the state vector.
    std::vector<double> getEEPose(Eigen::VectorXd & x);

    // Return the robot pose given the state vector.
    std::vector<double> getRobotPose(Eigen::VectorXd & x);

    // Return the robot base speed given the state vector.
    std::vector<double> getRobotBaseSpeed(Eigen::VectorXd & x);

    // Return the arm joints position given the state vector.
    std::vector<double> getArmJointsPosition(Eigen::VectorXd & x);

    // Return the wheels steering joints position given the state vector.
    std::vector<double> getSteeringJointsPosition(Eigen::VectorXd & x);

    // Return the arm actuation speed given the control vector.
    std::vector<double> getArmJointsSpeedCommand(Eigen::VectorXd & u);

    // Return the driving wheels speed given the control vector.
    std::vector<double> getDrivingWheelsSpeedCommand(Eigen::VectorXd & x, Eigen::VectorXd & u);

    // Return the steering wheels speed given the control vector.
    std::vector<double> getSteeringWheelsSpeedCommand(Eigen::VectorXd & x, Eigen::VectorXd & u);

    // Returns the risk distance to obstacles of the robot.
    double getRiskDistance();

    // Returns the safety distance to obstacles of the robot.
    double getSafetyDistance();

    // Returns the minimum allowed time horizon.
    double getMinTimeHorizon();

    // Returns the state vector given the robot state.
    Eigen::VectorXd getInitialStateVectorEigen(const std::vector<double> & robot_pose,
                                               const std::vector<double> & arm_positions);

    Eigen::VectorXd getInitialStateVectorEigen(const std::vector<double> & robot_pose,
                                               const std::vector<double> & arm_positions,
                                               const std::vector<double> & arm_speeds,
                                               const std::vector<double> & driving_speeds,
                                               const std::vector<double> & steer_positions = {});

    bool getInitialStateVectorEigen(const std::vector<double> & robot_pose,
                                    const std::vector<double> & arm_positions,
                                    Eigen::VectorXd & x);

    // Returns the state vector given the robot state.
    std::vector<double> getInitialStateVector(const std::vector<double> & robot_pose,
                                              const std::vector<double> & arm_positions);

    bool getInitialStateVector(const std::vector<double> & robot_pose,
                               const std::vector<double> & arm_positions,
                               std::vector<double> & x);

    // Returns the goal state vector given de desired goal ee pose.
    Eigen::VectorXd getGoalStateVectorEigen(const std::vector<double> & goal_ee_pose);

    // Returns the goal state vector given de desired goal ee pose.
    bool getGoalStateVectorEigen(const std::vector<double> & goal_ee_pose, Eigen::VectorXd & x);

    // Returns the goal state vector given de desired goal ee pose.
    std::vector<double> getGoalStateVector(const std::vector<double> & goal_ee_pose);

    // Returns the goal state vector given de desired goal ee pose.
    bool getGoalStateVector(const std::vector<double> & goal_ee_pose, std::vector<double> & x);

    // Returns the input vector given the robot inputs.
    Eigen::VectorXd getInputVectorEigen(const std::vector<double> & arm_speeds,
                                        const std::vector<double> & wheel_speeds,
                                        const std::vector<double> & wheel_steerings = {});

    // Returns the input vector given the robot inputs.
    bool getInputVectorEigen(const std::vector<double> & arm_speeds,
                             const std::vector<double> & wheel_speeds,
                             Eigen::VectorXd & u,
                             const std::vector<double> & wheel_steerings = {});

    // Returns the input vector given the robot inputs.
    std::vector<double> getInputVector(const std::vector<double> & arm_speeds,
                                       const std::vector<double> & wheel_speeds,
                                       const std::vector<double> & wheel_steerings = {});

    // Returns the input vector given the robot inputs.
    bool getInputVector(const std::vector<double> & arm_speeds,
                        const std::vector<double> & wheel_speeds,
                        std::vector<double> & u,
                        const std::vector<double> & wheel_steerings = {});

    // Returns the linearized state space model matrix A.
    // Size of A: number_states x number_states
    bool getLinearizedMatrixA(const std::vector<double> & x,
                              const std::vector<double> & u,
                              double time_step,
                              std::vector<std::vector<double>> & A);

    // Eigen overload
    bool getLinearizedMatrixA(const Eigen::VectorXd & x,
                              const Eigen::VectorXd & u,
                              double time_step,
                              Eigen::MatrixXd & A);

    // Returns the linearized state space model matrix B.
    // Size of B: number_states x number_inputs
    bool getLinearizedMatrixB(const std::vector<double> & x,
                              const std::vector<double> & u,
                              double time_step,
                              std::vector<std::vector<double>> & B);

    // Eigen overload
    bool getLinearizedMatrixB(const Eigen::VectorXd & x,
                              const Eigen::VectorXd & u,
                              double time_step,
                              Eigen::MatrixXd & B);

    // Returns the number of state input constraints.
    uint getNumberStateInputConstraints();

    // Returns the state input constraint matrix C.
    // Size of C: number_si_constraints x number_states
    bool getConstraintsMatrixC(std::vector<std::vector<double>> & C);

    // Eigen overload
    bool getConstraintsMatrixC(Eigen::MatrixXd & C);

    // Returns the state input constraint matrix D.
    // Size of D: number_si_constraints x number_inputs
    bool getConstraintsMatrixD(std::vector<std::vector<double>> & D);

    // Eigen overload
    bool getConstraintsMatrixD(Eigen::MatrixXd & D);

    // Returns the state input constraint matrix r.
    // Size of r: number_si_constraints
    bool getConstraintsMatrixR(std::vector<double> & r);

    // Eigen overload
    bool getConstraintsMatrixR(Eigen::VectorXd & r);

    // Returns the number of pure state constraints.
    uint getNumberPureStateConstraints();

    // Returns the pure state constraint matrix G.
    // Size of G: number_ps_constraints x number_states
    bool getConstraintsMatrixG(std::vector<std::vector<double>> & G);

    // Eigen overload
    bool getConstraintsMatrixG(Eigen::MatrixXd & G);

    // Returns the pure state constraint matrix h.
    // Size of h: number_ps_constraints
    bool getConstraintsMatrixH(std::vector<double> & h);

    // Eigen overload
    bool getConstraintsMatrixH(Eigen::VectorXd & h);

    // Returns the pure state cost matrix Q, in function of the time step provided
    // e.g. If percentage_horizon is 100, the goal state cost matrix will be returned
    // Size of Q: number_states x number_states
    bool getStateCostMatrix(double percentage_horizon,
                            double time_horizon,
                            std::vector<std::vector<double>> & Q,
                            bool track_reference_trajectory = false);

    // Eigen overload
    bool getStateCostMatrix(double percentage_horizon,
                            double time_horizon,
                            Eigen::MatrixXd & Q,
                            bool track_reference_trajectory = false);

    // Returns the pure input cost matrix R.
    // Size of R: number_inputs x number_inputs
    bool getInputCostMatrix(std::vector<std::vector<double>> & R, double time_horizon);

    // Eigen overload
    bool getInputCostMatrix(Eigen::MatrixXd & R, double time_horizon);

    // Returns the state input cost matrix K.
    // Size of K: number_states x number_inputs
    bool getStateInputCostMatrix(std::vector<std::vector<double>> & K);

    // Eigen overload
    bool getStateInputCostMatrix(Eigen::MatrixXd & K);

    // Returns the gravity matrix given the arm state.
    // Size of G: number_arm_joints
    bool getArmGravityMatrix(std::vector<double> arm_positions, std::vector<double> & G);

    // Returns the inertia matrix given the arm state.
    // Size of I: number_arm_joints x number_arm_joints
    bool getArmInertiaMatrix(const std::vector<double> & arm_positions,
                             std::vector<std::vector<double>> & I);

    // Returns the coriolis matrix given the arm state.
    // Size of C: number_arm_joints x number_arm_joints
    bool getArmCoriolisMatrix(const std::vector<double> & arm_positions,
                              const std::vector<double> & arm_speeds,
                              std::vector<std::vector<double>> & C);

    // Returns the jacobian matrix given the arm state.
    // Size of J: 6DoF x number_arm_joints
    bool getArmJacobianMatrix(const std::vector<double> & arm_positions,
                              std::vector<std::vector<double>> & J);

    // Returns the base-to-joint_index transform matrix given the arm state,
    // using direct kinematics
    // Size of T: 4 x 4
    bool getDirectKinematicsTransform(const std::vector<double> & arm_positions,
                                      uint joint_index,
                                      std::vector<std::vector<double>> & T);

    // Returns the base-to-all-joints transform matrixes given the arm state,
    // using direct kinematics
    // Size of T: number_arm_joints x 4 x 4
    bool getDirectKinematicsTransform(const std::vector<double> & arm_positions,
                                      std::vector<std::vector<std::vector<double>>> & T);

    // Returns the cost due to obstacles obstacles_cost given a certain
    // robot pose and the characteristics of the map of the scenario
    // (resolution and gradient of the obstacles map in X and Y directions)
    // Size of obstacles_cost: number_states
    bool getObstaclesCost(const std::vector<double> & x,
                          double map_resolution,
                          const std::vector<std::vector<double>> & gradient_obstacles_map_X,
                          const std::vector<std::vector<double>> & gradient_obstacles_map_Y,
                          double time_horizon,
                          std::vector<double> & obstacles_cost);

    // Eigen overload
    bool getObstaclesCost(const Eigen::VectorXd & x,
                          double map_resolution,
                          const std::vector<std::vector<double>> & gradient_obstacles_map_X,
                          const std::vector<std::vector<double>> & gradient_obstacles_map_Y,
                          double time_horizon,
                          Eigen::VectorXd & obstacles_cost);

    // Returns the wheel inertia
    double getWheelInertia();

    // Returns the steering angle of the front right wheel in function of the front left one
    double getRightWheelSteer(double left_wheel_steer);

    // Returns the driving speed of the right wheels in function of the front left steering angle
    double getRightWheelsSpeed(double left_wheels_speed, double left_wheel_steer);

    // Returns the updated state after applying the control input u into the previous state x
    // over a time interval time_step.
    bool forwardIntegrateModel(std::vector<double> x,
                               std::vector<double> u,
                               double time_step,
                               std::vector<double> & xf);

    // Eigen overload
    bool forwardIntegrateModel(const Eigen::VectorXd & x,
                               const Eigen::VectorXd & u,
                               double time_step,
                               Eigen::VectorXd & xf);
};
}    // namespace StateSpaceModels

#endif
