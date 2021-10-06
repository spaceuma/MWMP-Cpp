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

MobileManipulator::MobileManipulator(std::string robot_name)
{
    if(robot_name == "exoter")
    {
        //******************//
        // Model properties //
        //******************//
        number_states = 41;
        number_inputs = 7;

        pose_indexes = {10, 11};
        yaw_index = 12;

        base_speed_indexes = {13, 14, 15};

        state_constrained_indexes = {16,17,18,19,20,40,41};
        input_constrained_indexes = {1,2,3,4,5};

        goal_states_indexes = {1,2,3,16,8,20};
        whole_states_indexes = {10,11,12,31,32,33,34,35,40,41};

        whole_inputs_indexes = {1,2,3,4,5,6,7};

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

        state_limits = {90*pi/180, -10*pi/180, 160*pi/180, 250*pi/180, 160*pi/180, 2.85, 2.85,
                        -30*pi/180, -190*pi/180, -160*pi/180, -70*pi/180, -160*pi/180,-2.85,-2.85};
        input_limits = {0.01, 0.01, 0.01, 0.01, 0.01,
                        -0.01, -0.01, -0.01, -0.01, -0.01};

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
    }

}
