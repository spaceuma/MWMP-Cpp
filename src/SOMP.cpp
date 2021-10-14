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
using namespace MatrixOperations;

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
            << nocolor << std::endl;
        return false;
    }

    for(uint i = 1; i < number_time_steps; i++)
    {
        if(!robot_ss_model->forwardIntegrateModel(x[i - 1], u[i - 1], time_step, x[i]))
        {
            std::cout
                << red
                << "ERROR [MotionPlanner::updateHorizon]: Unable to forward integrate the model"
                << nocolor << std::endl;
            return false;
        }

        if(!robot_ss_model->getLinearizedMatrixA(x[i - 1], time_step, Ah[i]) ||
           !robot_ss_model->getLinearizedMatrixB(x[i - 1], u[i - 1], time_step, Bh[i]))
        {
            std::cout
                << red
                << "ERROR [MotionPlanner::updateHorizon]: Unable to linearize the state space model"
                << nocolor << std::endl;
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
                << nocolor << std::endl;
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
                      << nocolor << std::endl;
            return false;
        }
        if(!robot_ss_model->getConstraintsMatrixG(Gh[i]) ||
           !robot_ss_model->getConstraintsMatrixH(hh[i]))
        {
            std::cout << red
                      << "ERROR [MotionPlanner::updateHorizonConstraints]: Unable to compute the "
                         "pure state constraints"
                      << nocolor << std::endl;
            return false;
        }
    }

    return true;
}

bool MotionPlanner::computeObstaclesGradient(const std::vector<std::vector<uint>> & obst_map)
{
    uint m = obst_map.size();
    uint n = obst_map[0].size();

    // Obtaining the distance in X and Y axis to obstacles
    cv::Mat cv_obst_map(m, n, CV_32SC1);
    cv::Mat cv_x_distance = cv::Mat::zeros(m, n, CV_64FC1);
    cv::Mat cv_y_distance = cv::Mat::zeros(m, n, CV_64FC1);

    for(uint i = 0; i < m; i++)
    {
        for(uint j = 0; j < n; j++)
            cv_obst_map.at<int>(i, j) = 1 - obst_map[i][j];

        // Distance to closest 0
        cv::distanceTransform(cv_obst_map.row(i), cv_x_distance.row(i), cv::DIST_L2, 3);
        cv_x_distance.row(i) = 1 + cv_x_distance.row(i);
    }

    for(uint j = 0; j < n; j++)
    {
        // Distance to closest 0
        cv::distanceTransform(cv_obst_map.col(j), cv_y_distance.col(j), cv::DIST_L2, 3);
        cv_y_distance.col(j) = 1 + cv_y_distance.col(j);
    }

    // Computing the gradient
    cv::Mat cv_x_gradient = cv::Mat::zeros(m, n, CV_64FC1);
    cv::Mat cv_y_gradient = cv::Mat::zeros(m, n, CV_64FC1);

    for(uint i = 0; i < n; i++)
    {
        for(uint j = 0; j < m; j++)
        {
            if(i == 0)
            {
                cv_x_gradient.at<double>(j, 0) =
                    cv_x_distance.at<double>(j, 1) - cv_x_distance.at<double>(j, 0);
            }
            else
            {
                if(i == n - 1)
                {
                    cv_x_gradient.at<double>(j, i) =
                        cv_x_distance.at<double>(j, i) - cv_x_distance.at<double>(j, i - 1);
                }
                else
                {
                    if(cv_x_distance.at<double>(j, i + 1) == inf)
                    {
                        if(cv_x_distance.at<double>(j, i - 1) == inf)
                        { cv_x_gradient.at<double>(j, i) = 0; }
                        else
                        {
                            cv_x_gradient.at<double>(j, i) =
                                cv_x_distance.at<double>(j, i) - cv_x_distance.at<double>(j, i - 1);
                        }
                    }
                    else
                    {
                        if(cv_x_distance.at<double>(j, i - 1) == inf)
                        {
                            cv_x_gradient.at<double>(j, i) =
                                cv_x_distance.at<double>(j, i + 1) - cv_x_distance.at<double>(j, i);
                        }
                        else
                        {
                            cv_x_gradient.at<double>(j, i) = (cv_x_distance.at<double>(j, i + 1) -
                                                              cv_x_distance.at<double>(j, i - 1)) /
                                                             2;
                        }
                    }
                }
            }

            if(j == 0)
            {
                cv_y_gradient.at<double>(0, i) =
                    cv_y_distance.at<double>(1, i) - cv_y_distance.at<double>(0, i);
            }
            else
            {
                if(j == m - 1)
                {
                    cv_y_gradient.at<double>(j, i) =
                        cv_y_distance.at<double>(j, i) - cv_y_distance.at<double>(j - 1, i);
                }
                else
                {
                    if(cv_y_distance.at<double>(j + 1, i) == inf)
                    {
                        if(cv_y_distance.at<double>(j - 1, i) == inf)
                        { cv_y_gradient.at<double>(j, i) = 0; }
                        else
                        {
                            cv_y_gradient.at<double>(j, i) =
                                cv_y_distance.at<double>(j, i) - cv_y_distance.at<double>(j, i - 1);
                        }
                    }
                    else
                    {
                        if(cv_y_distance.at<double>(j - 1, i) == inf)
                        {
                            cv_y_gradient.at<double>(j, i) =
                                cv_y_distance.at<double>(j + 1, i) - cv_y_distance.at<double>(j, i);
                        }
                        else
                        {
                            cv_y_gradient.at<double>(j, i) = (cv_y_distance.at<double>(j + 1, i) -
                                                              cv_y_distance.at<double>(j - 1, i)) /
                                                             2;
                        }
                    }
                }
            }
        }
    }

    // Filtering the gradient
    // TODO check if this initialization is correct
    cv::Mat f10 = (cv::Mat_<float>(10, 10) << 0.01);
    cv::Mat f3 = (cv::Mat_<float>(3, 3) << 0.1111);

    cv::Mat cv_aux_x_gradient = cv::Mat::zeros(m, n, CV_64FC1);
    cv::Mat cv_aux_y_gradient = cv::Mat::zeros(m, n, CV_64FC1);

    cv::filter2D(
        cv_x_gradient, cv_aux_x_gradient, -1, f10, cv::Point(-1, -1), 0, cv::BORDER_CONSTANT);
    cv::filter2D(
        cv_aux_x_gradient, cv_x_gradient, -1, f3, cv::Point(-1, -1), 0, cv::BORDER_CONSTANT);

    cv::filter2D(
        cv_y_gradient, cv_aux_y_gradient, -1, f10, cv::Point(-1, -1), 0, cv::BORDER_CONSTANT);
    cv::filter2D(
        cv_aux_y_gradient, cv_y_gradient, -1, f3, cv::Point(-1, -1), 0, cv::BORDER_CONSTANT);

    // Generating the output matrixes
    for(uint i = 0; i < m; i++)
    {
        for(uint j = 0; j < n; j++)
        {
            gradient_obstacles_map_x[i][j] = cv_x_gradient.at<double>(i, j);
            gradient_obstacles_map_y[i][j] = cv_y_gradient.at<double>(i, j);
        }
    }

    return true;
}

bool MotionPlanner::dilateObstaclesMap(const std::vector<std::vector<uint>> & obst_map,
                                       double dilatation_distance,
                                       std::vector<std::vector<uint>> & dilated_map)
{
    uint m = obst_map.size();
    uint n = obst_map[0].size();

    if(dilated_map.size() != m || dilated_map[0].size() != n)
    {
        std::cout << red
                  << "ERROR [MotionPlanner::dilateObstaclesMap]: Wrong size of the output matrix"
                  << nocolor << std::endl;
        return false;
    }

    cv::Mat cv_obst_map(m, n, CV_32SC1);
    cv::Mat cv_dilated_map(m, n, CV_32SC1);

    for(uint i = 0; i < m; i++)
        for(uint j = 0; j < n; j++)
            cv_obst_map.at<int>(i, j) = obst_map[i][j];

    int dilatation_size = (int)dilatation_distance / map_resolution;

    cv::Mat sel =
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(dilatation_size, dilatation_size));

    cv::dilate(cv_obst_map, cv_dilated_map, sel);

    // Generating the output matrixes
    for(uint i = 0; i < m; i++)
        for(uint j = 0; j < n; j++)
            dilated_map[i][j] = cv_dilated_map.at<int>(i, j);

    return true;
}

bool MotionPlanner::computeLineSearch(std::vector<std::vector<double>> & x,
                                      const std::vector<std::vector<double>> & x0,
                                      std::vector<std::vector<double>> & u,
                                      const std::vector<std::vector<double>> & u0,
                                      const std::vector<std::vector<double>> & uh,
                                      const std::vector<std::vector<std::vector<double>>> & Qh,
                                      const std::vector<std::vector<std::vector<double>>> & Rh)
{
    double min_cost = inf;
    std::vector<std::vector<double>> current_cost(1, std::vector<double>(1, 0));

    std::vector<std::vector<double>> x_temp = x;
    std::vector<std::vector<double>> uk = u;
    std::vector<std::vector<double>> u_temp = u;

    std::vector<std::vector<double>> diff_states(1, std::vector<double>(number_states, 0));
    std::vector<std::vector<double>> diff_controls(1, std::vector<double>(number_inputs, 0));

    for(double alfa = 1; alfa > 0; alfa -= line_search_step)
    {
        getSum(uk, dot(alfa, uh), u_temp);

        for(uint i = 0; i < number_time_steps - 1; i++)
        {
            if(!robot_ss_model->forwardIntegrateModel(
                   x_temp[i], u_temp[i], time_step, x_temp[i + 1]))
            {
                std::cout << red
                          << "ERROR [MotionPlanner::computeLineSearch]: Unable to forward "
                             "integrate the model"
                          << nocolor << std::endl;
                return false;
            }

            getDifference(x_temp[i], x0[i], diff_states[0]);
            getDifference(u_temp[i], u0[i], diff_controls[0]);

            // TODO if performance is not good, divide the operations to pass-by-reference functions
            current_cost =
                getSum(current_cost,
                       getSum(dot(getTranspose(diff_states), dot(Qh[i], diff_states)),
                              dot(getTranspose(diff_controls), dot(Rh[i], diff_controls))));
        }

        getDifference(x_temp[number_time_steps - 1], x0[number_time_steps - 1], diff_states[0]);
        getDifference(u_temp[number_time_steps - 1], u0[number_time_steps - 1], diff_controls[0]);

        current_cost = getSum(
            current_cost,
            getSum(
                dot(getTranspose(diff_states), dot(Qh[number_time_steps - 1], diff_states)),
                dot(getTranspose(diff_controls), dot(Rh[number_time_steps - 1], diff_controls))));

        current_cost[0][0] /= 2;

        if(current_cost[0][0] < min_cost)
        {
            min_cost = current_cost[0][0];
            x = x_temp;
            u = u_temp;
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

    if(line_search_step < 0 || line_search_step >= 1)
    {
        throw std::domain_error(red +
                                std::string("ERROR [MotionPlanner::MotionPlanner]: The provided "
                                            "line search step is not inside the interval (0,1)") +
                                nocolor);
    }

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

    if(line_search_step < 0 || line_search_step >= 1)
    {
        throw std::domain_error(red +
                                std::string("ERROR [MotionPlanner::MotionPlanner]: The provided "
                                            "line search step is not inside the interval (0,1)") +
                                nocolor);
    }

    check_distance = config.check_distance;
    distance_indexes = robot_ss_model->getIndexesGoalDistance();
    distance_threshold = robot_ss_model->getThresholdGoalDistance();

    check_orientation = config.check_orientation;
    orientation_indexes = robot_ss_model->getIndexesGoalOrientation();
    orientation_threshold = robot_ss_model->getThresholdGoalOrientation();

    // Process the map info to obtain a safer representation of the scenario
    check_safety = true;
    pose_indexes = robot_ss_model->getIndexesRobotPose();
    map_resolution = map_info.map_resolution;
    obstacles_map = map_info.obstacles_map;

    if(!dilateObstaclesMap(
           obstacles_map, robot_ss_model->getRiskDistance(), dilated_obstacles_map) ||
       !dilateObstaclesMap(
           obstacles_map, robot_ss_model->getSafetyDistance(), safety_obstacles_map) ||
       // The gradient of the obstacles map is computed,
       // it will be used later to obtain repulsive costs in the neighbourhood of obstacles
       !computeObstaclesGradient(safety_obstacles_map))
    {
        throw std::domain_error(red +
                                std::string("ERROR [MotionPlanner::MotionPlanner]: Failure while "
                                            "processing the input obstacles map") +
                                nocolor);
    }

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
                  << nocolor << std::endl;
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
                  << nocolor << std::endl;
        return false;
    }
    return true;
}

int MotionPlanner::generateUnconstrainedMotionPlan(std::vector<double> x_ini,
                                                   std::vector<std::vector<double>> x0,
                                                   std::vector<double> u_ini,
                                                   std::vector<std::vector<double>> u0,
                                                   uint max_iter)
{
    // State and input along the whole time horizon
    std::vector<std::vector<double>> x(number_time_steps, std::vector<double>(number_states, 0));
    std::vector<std::vector<double>> u(number_time_steps, std::vector<double>(number_inputs, 0));

    // The initial states and inputs are the ones provided
    x[0] = x_ini;
    u[0] = u_ini;

    // Initializing the dynamics matrixes and cost matrixes along the whole time horizon
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

    // Initializing variables to control the status of the algorithm
    uint number_iterations = 0;
    int convergence_status = 0;

    // Initializing reference trajectories
    std::vector<std::vector<double>> xh0(number_time_steps, std::vector<double>(number_states, 0));
    std::vector<std::vector<double>> uh0(number_time_steps, std::vector<double>(number_inputs, 0));

    // Starting main loop
    while(true)
    {
        // Update states, linearization and costs
        if(!updateHorizon(x, u, Ah, Bh, Qh, Rh, Kh))
        {
            std::cout << red
                      << "ERROR [MotionPlanner::generateUnconstrainedMotionPlan]: Unable to update "
                         "the system state, linear matrixes and costs"
                      << nocolor << std::endl;
            return 0;
        }

        // Generate reference trajectories
        xh0 = getDifference(x0, x);
        uh0 = getDifference(u0, u);

        // If checking safety, generate obstacles repulsive cost
        std::vector<std::vector<double>> obstacles_repulsive_cost(
            number_time_steps, std::vector<double>(number_states, 0));
        if(check_safety)
        {
            for(uint i = 0; i < number_time_steps; i++)
            {
                robot_ss_model->getObstaclesCost(x[i],
                                                 map_resolution,
                                                 gradient_obstacles_map_x,
                                                 gradient_obstacles_map_y,
                                                 obstacles_repulsive_cost[i]);
            }
        }

        // LQR problem solution
        std::vector<std::vector<std::vector<double>>> M(
            number_time_steps,
            std::vector<std::vector<double>>(number_states, std::vector<double>(number_states, 0)));
        std::vector<std::vector<std::vector<double>>> P(
            number_time_steps,
            std::vector<std::vector<double>>(number_states, std::vector<double>(number_states, 0)));
        std::vector<std::vector<double>> s(number_time_steps,
                                           std::vector<double>(number_states, 0));

        // TODO if performance is not good, divide the operations to pass-by-reference functions
        P[number_time_steps - 1] = Qh[number_time_steps - 1];
        s[number_time_steps - 1] =
            getDifference(obstacles_repulsive_cost[number_time_steps - 1],
                          dot(Qh[number_time_steps - 1], xh0[number_time_steps - 1]));

        std::vector<std::vector<double>> xh(number_time_steps,
                                            std::vector<double>(number_states, 0));
        std::vector<std::vector<double>> uh(number_time_steps,
                                            std::vector<double>(number_inputs, 0));
        std::vector<std::vector<double>> v(number_time_steps,
                                           std::vector<double>(number_states, 0));
        std::vector<std::vector<double>> lambdah(number_time_steps,
                                                 std::vector<double>(number_states, 0));

        // Solve backwards
        // TODO if performance is not good, divide the operations to pass-by-reference functions
        for(uint i = number_time_steps - 2; i >= 0; i--)
        {
            M[i] = getSum(getIdentity(number_states),
                          dot(Bh[i], dot(getInverse(Rh[i]), dot(getTranspose(Bh[i]), P[i]))));
            P[i] = getSum(Qh[i], dot(getTranspose(Ah[i]), dot(P[i], dot(M[i], Ah[i]))));
            s[i] = getDifference(
                getSum(
                    dot(getTranspose(Ah[i]),
                        dot(getDifference(
                                getIdentity(number_states),
                                dot(P[i + 1],
                                    dot(M[i],
                                        dot(Bh[i], dot(getInverse(Rh[i]), getTranspose(Bh[i])))))),
                            s[i + 1])),
                    dot(getTranspose(Ah[i]), dot(P[i + 1], dot(M[i], dot(Bh[i], uh0[i]))))),
                getSum(dot(Qh[i], xh0[i]), obstacles_repulsive_cost[i]));
        }

        // Solve forwards
        for(uint i = 0; i < number_time_steps - 1; i++)
        {
            v[i] =
                dot(M[i],
                    dot(Bh[i],
                        getDifference(uh0[i],
                                      dot(getInverse(Rh[i]), dot(getTranspose(Bh[i]), s[i + 1])))));
            xh[i + 1] = getSum(dot(M[i], dot(Ah[i], xh[i])), v[i]);
            lambdah[i + 1] = getSum(dot(P[i + 1], xh[i + 1]), s[i + 1]);
            uh[i] = getDifference(uh0[i],
                                  dot(getInverse(Rh[i]), dot(getTranspose(Bh[i]), lambdah[i + 1])));
        }

        // Checking termination conditions
        bool convergence_condition = true;
        double distance_to_goal;
        double orientation_to_goal;
        number_iterations++;
        std::cout << std::endl
                  << "\r[MotionPlanner::generateUnconstrainedMotionPlan]: Iteration number "
                  << number_iterations << "...";

        // Check if the control is not changing
        for(uint i = 0; i < number_time_steps; i++)
            convergence_condition &= (getNorm(uh[i]) <= control_threshold * getNorm(u[i]));
        if(!convergence_condition) convergence_status = -2;

        // Check if the distance objective has been reached
        if(convergence_condition)
        {
            if(check_distance && convergence_condition)
            {
                std::vector<double> termination_state(distance_indexes.size(), 0);
                std::vector<double> termination_goal(distance_indexes.size(), 0);
                for(uint i = 0; i < distance_indexes.size(); i++)
                {
                    termination_state[i] = x[number_time_steps - 1][distance_indexes[i]];
                    termination_goal[i] = x0[number_time_steps - 1][distance_indexes[i]];
                }

                distance_to_goal = getNorm(getDifference(termination_state, termination_goal));

                if(distance_to_goal < distance_threshold)
                {
                    convergence_condition = true;

                    for(uint i = 0; i < number_time_steps; i++)
                        convergence_condition &=
                            (getNorm(uh[i]) <= 20 * control_threshold * getNorm(u[i]));
                }

                if(!convergence_condition) convergence_status = -1;
            }

            // Check if the orientation objective has been reached
            if(check_orientation && convergence_condition)
            {
                std::vector<double> termination_state(orientation_indexes.size(), 0);
                std::vector<double> termination_goal(orientation_indexes.size(), 0);
                for(uint i = 0; i < orientation_indexes.size(); i++)
                {
                    termination_state[i] = x[number_time_steps - 1][orientation_indexes[i]];
                    termination_goal[i] = x0[number_time_steps - 1][orientation_indexes[i]];
                }

                orientation_to_goal = getNorm(getDifference(termination_state, termination_goal));

                convergence_condition &= (orientation_to_goal < orientation_threshold);

                if(!convergence_condition) convergence_status = -1;
            }

            // Check if the planned motion is safe
            if(check_safety && convergence_condition)
            {
                for(uint i = 0; i < number_time_steps; i++)
                {
                    std::vector<double> pose = {x[i][pose_indexes[0]], x[i][pose_indexes[1]]};
                    uint ix = (uint)pose[0] / map_resolution;
                    uint iy = (uint)pose[1] / map_resolution;

                    if(ix >= 0 && ix < dilated_obstacles_map[0].size() && iy >= 0 &&
                       iy < dilated_obstacles_map.size())
                    { convergence_condition &= !dilated_obstacles_map[iy][ix]; }
                    else
                        convergence_condition = false;
                }
                if(!convergence_condition) convergence_status = -3;
            }
        }

        // If the algorithm has converged!!
        if(convergence_condition)
        {
            std::cout << std::endl;
            std::cout << green
                      << "[MotionPlanner::generateUnconstrainedMotionPlan]: The motion planner "
                         "found a solution!"
                      << nocolor << std::endl;
            if(check_distance)
                std::cout << "[MotionPlanner::generateUnconstrainedMotionPlan]: Distance to "
                             "goalposition: "
                          << distance_to_goal << std::endl;
            if(check_orientation)
                std::cout << "[MotionPlanner::generateUnconstrainedMotionPlan]: Distance to "
                             "goalorientation: "
                          << orientation_to_goal << std::endl;

            planned_state = x;
            planned_control = u;

            return 1;
        }
        else
        {
            // Decide the best way to apply the last obtained state and control
            // steps (xh and uh)
            computeLineSearch(x, x0, u, u0, uh, Qh, Rh);
        }

        // If the algorithm finally failed to converge
        if(number_iterations > max_iter)
        {
            std::cout << red
                      << "ERROR [MotionPlanner::generateUnconstrainedMotionPlan]: The motion "
                         "planner failed to find a solution";

            switch(convergence_status)
            {
                case -1:
                    std::cout << "The goal distance threshold was not satisfied";
                    break;
                case -2:
                    std::cout << "The goal control threshold was not satisfied";
                    break;
                case -3:
                    std::cout << "The generated solution was not safe";
                    break;
                default:
                    std::cout << "Something unexpected happened";
                    break;
            }

            std::cout << nocolor << std::endl;
            return convergence_status;
        }
    }

    return 1;
}

int MotionPlanner::generateConstrainedMotionPlan(std::vector<double> x,
                                                 std::vector<std::vector<double>> x0,
                                                 std::vector<double> u,
                                                 std::vector<std::vector<double>> u0,
                                                 uint max_iter)
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
                  << "ERROR [MotionPlanner::generateConstrainedMotionPlan]: Unable to initialize "
                     "the constraints"
                  << nocolor << std::endl;
        return 0;
    }

    return 1;
}

int MotionPlanner::generateSteppedMotionPlan(std::vector<double> x,
                                             std::vector<std::vector<double>> x0,
                                             std::vector<double> u,
                                             std::vector<std::vector<double>> u0)
{
    return 1;
}

bool MotionPlanner::getPlannedState(std::vector<std::vector<double>> & x)
{
    if(!is_motion_planned)
    {
        std::cout << red
                  << "ERROR [MotionPlanner::getPlannedState]: No motion plan has yet been generated"
                  << nocolor << std::endl;
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
            << nocolor << std::endl;
        return false;
    }
    else
        u = planned_control;

    return true;
}
