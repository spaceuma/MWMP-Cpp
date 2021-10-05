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

/**
 *  Library containing state space models for different robots.
 *
 * CONSTRUCTORS:
 * StateSpaceModel * robot_ss_model = new StateSpaceModel(string robot_name);
 *
 * "string robot_name" should contain the name of the robot model to be used.
 *
 *
 * USAGE:
 * std::vector<std::vector<std::vector<double>>> A =
 *              robot_ss_model.getLinearizedMatrixA(std::vector<std::vector<double>> x,
 *                                                  std::vector<std::vector<double>> u,
 *                                                  double time_step);
 *
 * Returns the linearized state space model matrix A.
 * Size of A: number_states x number_states x number_time_steps.
 *
 * std::vector<std::vector<std::vector<double>>> B =
 *              robot_ss_model.getLinearizedMatrixB(std::vector<std::vector<double>> x,
 *                                                  std::vector<std::vector<double>> u,
 *                                                  double time_step);
 *
 * Returns the linearized state space model matrix B.
 * Size of B: number_states x number_inputs x number_time_steps.
 *
 *
 * int number_si_constraints = getNumberStateInputConstraints();
 *
 * Returns the number of state input constraints.
 *
 * std::vector<std::vector<std::vector<double>>> C = robot_ss_model.getConstraintsMatrixC();
 *
 * Returns the state input constraint matrix C.
 * Size of C: number_si_constraints x number_states x number_time_steps
 *
 * std::vector<std::vector<std::vector<double>>> D = robot_ss_model.getConstraintsMatrixD();
 *
 * Returns the state input constraint matrix D.
 * Size of D: number_si_constraints x number_inputs x number_time_steps
 *
 * std::vector<std::vector<double>> r = robot_ss_model.getConstraintsMatrixR();
 *
 * Returns the state input constraint matrix r.
 * Size of r: number_si_constraints x number_time_steps
 *
 *
 * int ps_constraints = getNumberPureStateConstraints();
 *
 * Returns the number of pure state constraints.
 *
 * std::vector<std::vector<std::vector<double>>> G = robot_ss_model.getConstraintsMatrixG();
 *
 * Returns the pure state constraint matrix G.
 * Size of G: number_ps_constraints x number_states x number_time_steps
 *
 * std::vector<std::vector<double>> h = robot_ss_model.getConstraintsMatrixH();
 *
 * Returns the pure state constraint matrix h.
 * Size of G: number_ps_constraints x number_time_steps
 *
 *
 * std::vector<std::vector<std::vector<double>>> Q = robot_ss_model.getStateCostMatrix();
 *
 * Returns the pure state cost matrix Q.
 * Size of Q: number_states x number_states x number_time_steps.
 *
 * std::vector<std::vector<std::vector<double>>> R = robot_ss_model.getInputCostMatrix();
 *
 * Returns the pure input cost matrix R.
 * Size of R: number_inputs x number_inputs x number_time_steps.
 *
 * std::vector<std::vector<std::vector<double>>> K = robot_ss_model.getStateInputCostMatrix();
 *
 * Returns the state input cost matrix K.
 * Size of K: number_states x number_inputs x number_time_steps.
 *
 *
 * std::vector<std::vector<double>> xf = forwardIntegrateModel(std::vector<std::vector<double>> x,
 *                                                             std::vector<std::vector<double>> u,
 *                                                             double time_step);
 *
 * Returns the updated state xf after applying the control input u into the initial state x.
 *
 *
 * std::vector<double> armG = getArmGravityMatrix(std::vector<double> x);
 *
 * Returns the gravity matrix given the arm state. Size of G: number_arm_joints
 *
 * std::vector<std::vector<double>> armB = getArmInertiaMatrix(std::vector<double> x);
 *
 * Returns the inertia matrix given the arm state. Size of B: number_arm_joints x number_arm_joints
 *
 * std::vector<std::vector<double>> armC = getArmCoriolisMatrix(std::vector<double> x);
 *
 * Returns the coriolis matrix given the arm state. Size of C: number_arm_joints x number_arm_joints
 *
 *
 * Parameters that configure each robot:
 *   - Indexes of the XY pose of the robot "std::vector<int> pose_indexes".
 *   - Position, velocity limits.
 *   - Arm shape and masses
 *   - Rover base shape and masses
 *   - Offset between rover center and arm base
 *   - Arm reachability distance
 *   - Wheels shape and masses
 *   - Robot mass
 *   - Ground rolling resistance
 *   - Gravitational acceleration
 *   - Average vehicle speed
 *   - Percentage of trajectory in which start recucing the base speed
 *   - Distance to obstacles considered risky
 *   - Distance to obstacles considered as obstacles for safety
 *   - State and input costs
 *   - Extra costs
 *
 **/

class StateSpaceModel
{

private:

    /**
    * Dependency classes
    */
 
    /**
     * Configurable Parameters
     */

    /**
     * Supporting Functions
     */

public:

    /**
     * Class Constructor.
     */
    StateSpaceModel();

    /**
     * Get Data
     */

    /*
     * Checking Functions
     */

};

#endif
