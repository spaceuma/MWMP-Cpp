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
#include <ctime>

using namespace SOMP;
using namespace StateSpaceModels;
using namespace MatrixOperations;

bool MotionPlanner::generateHorizon(std::vector<Eigen::VectorXd> & x,
                                    const std::vector<Eigen::VectorXd> & u,
                                    std::vector<Eigen::MatrixXd> & Ah,
                                    std::vector<Eigen::MatrixXd> & Bh,
                                    std::vector<Eigen::MatrixXd> & Qh,
                                    std::vector<Eigen::MatrixXd> & Rh,
                                    std::vector<Eigen::MatrixXd> & Kh)
{
    if(!robot_ss_model->getLinearizedMatrixA(x[0], time_step, Ah[0]) ||
       !robot_ss_model->getLinearizedMatrixB(x[0], u[0], time_step, Bh[0]))
    {
        std::cout
            << red
            << "ERROR [MotionPlanner::generateHorizon]: Unable to linearize the state space model"
            << nocolor << std::endl;
        return false;
    }

    if(!robot_ss_model->getStateCostMatrix(0, time_horizon, Qh[0]) ||
       !robot_ss_model->getInputCostMatrix(Rh[0], time_horizon) ||
       !robot_ss_model->getStateInputCostMatrix(Kh[0]))
    {
        std::cout
            << red
            << "ERROR [MotionPlanner::generateHorizon]: Unable to compute the quadratized costs"
            << nocolor << std::endl;
        return false;
    }

    for(uint i = 1; i < number_time_steps; i++)
    {
        if(!robot_ss_model->forwardIntegrateModel(x[i - 1], u[i - 1], time_step, x[i]))
        {
            std::cout
                << red
                << "ERROR [MotionPlanner::generateHorizon]: Unable to forward integrate the model"
                << nocolor << std::endl;
            return false;
        }

        if(!robot_ss_model->getLinearizedMatrixA(x[i - 1], time_step, Ah[i]) ||
           !robot_ss_model->getLinearizedMatrixB(x[i - 1], u[i - 1], time_step, Bh[i]))
        {
            std::cout << red
                      << "ERROR [MotionPlanner::generateHorizon]: Unable to linearize the state "
                         "space model"
                      << nocolor << std::endl;
            return false;
        }

        double percentage_horizon = 100 * ((double)i + 1) / (double)number_time_steps;

        if(!robot_ss_model->getStateCostMatrix(percentage_horizon, time_horizon, Qh[i]) ||
           !robot_ss_model->getInputCostMatrix(Rh[i], time_horizon) ||
           !robot_ss_model->getStateInputCostMatrix(Kh[i]))
        {
            std::cout
                << red
                << "ERROR [MotionPlanner::generateHorizon]: Unable to compute the quadratized costs"
                << nocolor << std::endl;
            return false;
        }
    }

    return true;
}

bool MotionPlanner::generateHorizon(std::vector<std::vector<double>> & x,
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
            << "ERROR [MotionPlanner::generateHorizon]: Unable to linearize the state space model"
            << nocolor << std::endl;
        return false;
    }

    if(!robot_ss_model->getStateCostMatrix(0, time_horizon, Qh[0]) ||
       !robot_ss_model->getInputCostMatrix(Rh[0], time_horizon) ||
       !robot_ss_model->getStateInputCostMatrix(Kh[0]))
    {
        std::cout
            << red
            << "ERROR [MotionPlanner::generateHorizon]: Unable to compute the quadratized costs"
            << nocolor << std::endl;
        return false;
    }

    for(uint i = 1; i < number_time_steps; i++)
    {
        if(!robot_ss_model->forwardIntegrateModel(x[i - 1], u[i - 1], time_step, x[i]))
        {
            std::cout
                << red
                << "ERROR [MotionPlanner::generateHorizon]: Unable to forward integrate the model"
                << nocolor << std::endl;
            return false;
        }

        if(!robot_ss_model->getLinearizedMatrixA(x[i - 1], time_step, Ah[i]) ||
           !robot_ss_model->getLinearizedMatrixB(x[i - 1], u[i - 1], time_step, Bh[i]))
        {
            std::cout << red
                      << "ERROR [MotionPlanner::generateHorizon]: Unable to linearize the state "
                         "space model"
                      << nocolor << std::endl;
            return false;
        }

        double percentage_horizon = 100 * ((double)i + 1) / (double)number_time_steps;

        if(!robot_ss_model->getStateCostMatrix(percentage_horizon, time_horizon, Qh[i]) ||
           !robot_ss_model->getInputCostMatrix(Rh[i], time_horizon) ||
           !robot_ss_model->getStateInputCostMatrix(Kh[i]))
        {
            std::cout
                << red
                << "ERROR [MotionPlanner::generateHorizon]: Unable to compute the quadratized costs"
                << nocolor << std::endl;
            return false;
        }
    }

    return true;
}

bool MotionPlanner::updateLinearModel(const std::vector<Eigen::VectorXd> & x,
                                      const std::vector<Eigen::VectorXd> & u,
                                      std::vector<Eigen::MatrixXd> & Ah,
                                      std::vector<Eigen::MatrixXd> & Bh)
{
    for(uint i = 1; i < number_time_steps; i++)
    {
        if(!robot_ss_model->getLinearizedMatrixA(x[i - 1], time_step, Ah[i]) ||
           !robot_ss_model->getLinearizedMatrixB(x[i - 1], u[i - 1], time_step, Bh[i]))
        {
            std::cout << red
                      << "ERROR [MotionPlanner::updateLinearModel]: Unable to linearize the state "
                         "space model"
                      << nocolor << std::endl;
            return false;
        }
    }

    return true;
}

bool MotionPlanner::updateLinearModel(const std::vector<std::vector<double>> & x,
                                      const std::vector<std::vector<double>> & u,
                                      std::vector<std::vector<std::vector<double>>> & Ah,
                                      std::vector<std::vector<std::vector<double>>> & Bh)
{
    for(uint i = 1; i < number_time_steps; i++)
    {
        if(!robot_ss_model->getLinearizedMatrixA(x[i - 1], time_step, Ah[i]) ||
           !robot_ss_model->getLinearizedMatrixB(x[i - 1], u[i - 1], time_step, Bh[i]))
        {
            std::cout << red
                      << "ERROR [MotionPlanner::updateLinearModel]: Unable to linearize the state "
                         "space model"
                      << nocolor << std::endl;
            return false;
        }
    }

    return true;
}

bool MotionPlanner::generateHorizonConstraints(std::vector<Eigen::MatrixXd> & Ch,
                                               std::vector<Eigen::MatrixXd> & Dh,
                                               std::vector<Eigen::VectorXd> & rh,
                                               std::vector<Eigen::MatrixXd> & Gh,
                                               std::vector<Eigen::VectorXd> & hh)
{
    for(uint i = 0; i < number_time_steps; i++)
    {
        if(!robot_ss_model->getConstraintsMatrixC(Ch[i]) ||
           !robot_ss_model->getConstraintsMatrixD(Dh[i]) ||
           !robot_ss_model->getConstraintsMatrixR(rh[i]))
        {
            std::cout << red
                      << "ERROR [MotionPlanner::generateHorizonConstraints]: Unable to compute the "
                         "state input constraints"
                      << nocolor << std::endl;
            return false;
        }
        if(!robot_ss_model->getConstraintsMatrixG(Gh[i]) ||
           !robot_ss_model->getConstraintsMatrixH(hh[i]))
        {
            std::cout << red
                      << "ERROR [MotionPlanner::generateHorizonConstraints]: Unable to compute the "
                         "pure state constraints"
                      << nocolor << std::endl;
            return false;
        }
    }

    return true;
}

bool MotionPlanner::generateHorizonConstraints(std::vector<std::vector<std::vector<double>>> & Ch,
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
                      << "ERROR [MotionPlanner::generateHorizonConstraints]: Unable to compute the "
                         "state input constraints"
                      << nocolor << std::endl;
            return false;
        }
        if(!robot_ss_model->getConstraintsMatrixG(Gh[i]) ||
           !robot_ss_model->getConstraintsMatrixH(hh[i]))
        {
            std::cout << red
                      << "ERROR [MotionPlanner::generateHorizonConstraints]: Unable to compute the "
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
    cv::Mat cv_obst_map = cv::Mat::ones(m, n, CV_8UC1);
    cv::Mat cv_distance_x = cv::Mat::zeros(m, n, CV_32FC1);
    cv::Mat cv_distance_y = cv::Mat::zeros(m, n, CV_32FC1);
    cv::Mat cv_distance_aux = cv::Mat::zeros(m, n, CV_32FC1);

    // Computing X distance row by row
    for(uint i = 0; i < m; i++)
    {
        for(uint j = 0; j < n; j++)
            cv_obst_map.at<uchar>(i, j) = obst_map[i][j];

        cv::distanceTransform(cv_obst_map, cv_distance_aux, cv::DIST_L2, 5);
        cv_distance_aux.row(i).copyTo(cv_distance_x.row(i));

        cv_obst_map = cv::Mat::ones(m, n, CV_8UC1);
    }

    // Computing Y distance column by column
    for(uint j = 0; j < m; j++)
    {
        for(uint i = 0; i < n; i++)
            cv_obst_map.at<uchar>(i, j) = obst_map[i][j];

        cv::distanceTransform(cv_obst_map, cv_distance_aux, cv::DIST_L2, 5);
        cv_distance_aux.col(j).copyTo(cv_distance_y.col(j));

        cv_obst_map = cv::Mat::ones(m, n, CV_8UC1);
    }

    cv_distance_x += 1;
    cv_distance_y += 1;

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
                    cv_distance_x.at<float>(j, 1) - cv_distance_x.at<float>(j, 0);
            }
            else
            {
                if(i == n - 1)
                {
                    cv_x_gradient.at<double>(j, i) =
                        cv_distance_x.at<float>(j, i) - cv_distance_x.at<float>(j, i - 1);
                }
                else
                {
                    if(cv_distance_x.at<float>(j, i + 1) == 8193)
                    {
                        if(cv_distance_x.at<float>(j, i - 1) == 8193)
                        { cv_x_gradient.at<double>(j, i) = 0; }
                        else
                        {
                            cv_x_gradient.at<double>(j, i) =
                                cv_distance_x.at<float>(j, i) - cv_distance_x.at<float>(j, i - 1);
                        }
                    }
                    else
                    {
                        if(cv_distance_x.at<float>(j, i - 1) == 8193)
                        {
                            cv_x_gradient.at<double>(j, i) =
                                cv_distance_x.at<float>(j, i + 1) - cv_distance_x.at<float>(j, i);
                        }
                        else
                        {
                            cv_x_gradient.at<double>(j, i) = (cv_distance_x.at<float>(j, i + 1) -
                                                              cv_distance_x.at<float>(j, i - 1)) /
                                                             2;
                        }
                    }
                }
            }

            if(j == 0)
            {
                cv_y_gradient.at<double>(0, i) =
                    cv_distance_y.at<float>(1, i) - cv_distance_y.at<float>(0, i);
            }
            else
            {
                if(j == m - 1)
                {
                    cv_y_gradient.at<double>(j, i) =
                        cv_distance_y.at<float>(j, i) - cv_distance_y.at<float>(j - 1, i);
                }
                else
                {
                    if(cv_distance_y.at<float>(j + 1, i) == 8193)
                    {
                        if(cv_distance_y.at<float>(j - 1, i) == 8193)
                        { cv_y_gradient.at<double>(j, i) = 0; }
                        else
                        {
                            cv_y_gradient.at<double>(j, i) =
                                cv_distance_y.at<float>(j, i) - cv_distance_y.at<float>(j, i - 1);
                        }
                    }
                    else
                    {
                        if(cv_distance_y.at<float>(j - 1, i) == 8193)
                        {
                            cv_y_gradient.at<double>(j, i) =
                                cv_distance_y.at<float>(j + 1, i) - cv_distance_y.at<float>(j, i);
                        }
                        else
                        {
                            cv_y_gradient.at<double>(j, i) = (cv_distance_y.at<float>(j + 1, i) -
                                                              cv_distance_y.at<float>(j - 1, i)) /
                                                             2;
                        }
                    }
                }
            }
        }
    }

    // Filtering the gradient
    cv::Mat f10(cv::Size(10, 10), CV_32FC1, cv::Scalar(0.01));
    cv::Mat f3(cv::Size(3, 3), CV_32FC1, cv::Scalar(0.1111));

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

    cv::Mat cv_obst_map(m, n, CV_16SC1);
    cv::Mat cv_dilated_map(m, n, CV_16SC1);

    for(uint i = 0; i < m; i++)
        for(uint j = 0; j < n; j++)
            cv_obst_map.at<short>(i, j) = obst_map[i][j];

    int dilatation_size = (int)(dilatation_distance / map_resolution);

    cv::Mat sel =
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(dilatation_size, dilatation_size));

    cv::dilate(cv_obst_map, cv_dilated_map, sel);

    // Generating the output matrixes
    for(uint i = 0; i < m; i++)
        for(uint j = 0; j < n; j++)
            dilated_map[i][j] = cv_dilated_map.at<short>(i, j);

    return true;
}

bool MotionPlanner::computeLineSearch(std::vector<Eigen::VectorXd> & x,
                                      const std::vector<Eigen::VectorXd> & x0,
                                      std::vector<Eigen::VectorXd> & u,
                                      const std::vector<Eigen::VectorXd> & u0,
                                      const std::vector<Eigen::VectorXd> & uh,
                                      const std::vector<Eigen::MatrixXd> & Qh,
                                      const std::vector<Eigen::MatrixXd> & Rh)
{
    double min_cost = inf;

    std::vector<Eigen::VectorXd> x_temp = x;
    std::vector<Eigen::VectorXd> uk = u;
    std::vector<Eigen::VectorXd> u_temp = u;

    Eigen::VectorXd diff_states(number_states);
    Eigen::VectorXd diff_controls(number_inputs);

    for(double alfa = 1; alfa > 0; alfa -= line_search_step)
    {
        Eigen::VectorXd current_cost(1);
        current_cost << 0;

        for(uint i = 0; i < number_time_steps - 1; i++)
        {
            u_temp[i] = uk[i] + alfa * uh[i];
            if(!robot_ss_model->forwardIntegrateModel(
                   x_temp[i], u_temp[i], time_step, x_temp[i + 1]))
            {
                std::cout << red
                          << "ERROR [MotionPlanner::computeLineSearch]: Unable to forward "
                             "integrate the model"
                          << nocolor << std::endl;
                return false;
            }

            diff_states = x_temp[i] - x0[i];
            diff_controls = u_temp[i] - u0[i];

            current_cost += (diff_states.transpose() * Qh[i] * diff_states +
                             diff_controls.transpose() * Rh[i] * diff_controls) /
                            2;
        }

        diff_states = x_temp[number_time_steps - 1] - x0[number_time_steps - 1];

        current_cost += (diff_states.transpose() * Qh[number_time_steps - 1] * diff_states) / 2;

        if(current_cost(0) < min_cost)
        {
            min_cost = current_cost(0);
            x = x_temp;
            u = u_temp;
        }
    }

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

    std::vector<std::vector<double>> x_temp = x;
    std::vector<std::vector<double>> uk = u;
    std::vector<std::vector<double>> u_temp = u;

    std::vector<double> diff_states(number_states);
    std::vector<double> diff_controls(number_inputs);

    for(double alfa = 1; alfa > 0; alfa -= line_search_step)
    {
        double current_cost = 0;

        for(uint i = 0; i < number_time_steps - 1; i++)
        {
            u_temp[i] = getSum(uk[i], dot(alfa, uh[i]));
            if(!robot_ss_model->forwardIntegrateModel(
                   x_temp[i], u_temp[i], time_step, x_temp[i + 1]))
            {
                std::cout << red
                          << "ERROR [MotionPlanner::computeLineSearch]: Unable to forward "
                             "integrate the model"
                          << nocolor << std::endl;
                return false;
            }

            diff_states = getDifference(x_temp[i], x0[i]);
            diff_controls = getDifference(u_temp[i], u0[i]);

            current_cost += dot(diff_states, dot(Qh[i], diff_states)) +
                            dot(diff_controls, dot(Rh[i], diff_controls));
        }

        diff_states = getDifference(x_temp[number_time_steps - 1], x0[number_time_steps - 1]);

        current_cost += dot(diff_states, dot(Qh[number_time_steps - 1], diff_states));
        current_cost /= 2;

        if(current_cost < min_cost)
        {
            min_cost = current_cost;
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

    number_time_steps = (uint)(time_horizon / time_step) + 1;

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

    number_time_steps = (uint)(time_horizon / time_step) + 1;

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

    // Creating an obstacle below the goal to avoid stepping into it
    uint x_index_goal = (uint)(map_info.goal_pose[0] / map_resolution);
    uint y_index_goal = (uint)(map_info.goal_pose[1] / map_resolution);
    obstacles_map[y_index_goal][x_index_goal] = 1;

    dilated_obstacles_map = obstacles_map;
    safety_obstacles_map = obstacles_map;

    gradient_obstacles_map_x = std::vector<std::vector<double>>(
        obstacles_map.size(), std::vector<double>(obstacles_map[0].size(), 0));
    gradient_obstacles_map_y = std::vector<std::vector<double>>(
        obstacles_map.size(), std::vector<double>(obstacles_map[0].size(), 0));

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

int MotionPlanner::generateUnconstrainedMotionPlan(const Eigen::VectorXd & x_ini,
                                                   const std::vector<Eigen::VectorXd> & x0,
                                                   const Eigen::VectorXd & u_ini,
                                                   const std::vector<Eigen::VectorXd> & u0,
                                                   uint max_iter)
{
    if(x0.size() != number_time_steps)
    {
        std::cout << red
                  << "ERROR [MotionPlanner::generateUnconstrainedMotionPlan]: The provided goal "
                     "state matrix has a wrong number of time steps"
                  << nocolor << std::endl;
        return -1;
    }

    if(u0.size() != number_time_steps)
    {
        std::cout << red
                  << "ERROR [MotionPlanner::generateUnconstrainedMotionPlan]: The provided goal "
                     "input matrix has a wrong number of time steps"
                  << nocolor << std::endl;
        return -1;
    }

    // State and input along the whole time horizon
    std::vector<Eigen::VectorXd> x(number_time_steps, Eigen::VectorXd::Zero(number_states));
    std::vector<Eigen::VectorXd> u(number_time_steps, Eigen::VectorXd::Zero(number_inputs));

    // The initial states and inputs are the ones provided
    x[0] = x_ini;
    u[0] = u_ini;

    // Initializing the dynamics matrixes and cost matrixes along the whole time horizon
    std::vector<Eigen::MatrixXd> Ah(number_time_steps,
                                    Eigen::MatrixXd::Zero(number_states, number_states));
    std::vector<Eigen::MatrixXd> Bh(number_time_steps,
                                    Eigen::MatrixXd::Zero(number_states, number_inputs));

    std::vector<Eigen::MatrixXd> Qh(number_time_steps,
                                    Eigen::MatrixXd::Zero(number_states, number_states));
    std::vector<Eigen::MatrixXd> Rh(number_time_steps,
                                    Eigen::MatrixXd::Zero(number_inputs, number_inputs));
    std::vector<Eigen::MatrixXd> Kh(number_time_steps,
                                    Eigen::MatrixXd::Zero(number_states, number_inputs));

    // Initializing variables to control the status of the algorithm
    uint number_iterations = 0;
    int convergence_status = 0;

    // Generating the state, linearized matrixes and cost matrixes for the whole time horizon
    if(!generateHorizon(x, u, Ah, Bh, Qh, Rh, Kh))
    {
        std::cout << red
                  << "ERROR [MotionPlanner::generateUnconstrainedMotionPlan]: Unable to generate "
                     "the system state, linear matrixes and costs for the time horizon"
                  << nocolor << std::endl;
        return 0;
    }

    // Initializing reference trajectories
    std::vector<Eigen::VectorXd> xh0(number_time_steps, Eigen::VectorXd::Zero(number_states));
    std::vector<Eigen::VectorXd> uh0(number_time_steps, Eigen::VectorXd::Zero(number_inputs));

    // Auxiliary variables
    Eigen::MatrixXd I_states = Eigen::MatrixXd::Identity(number_states, number_states);
    Eigen::MatrixXd I_inputs = Eigen::MatrixXd::Identity(number_inputs, number_inputs);
    Eigen::MatrixXd Rh_inv = Rh[0].partialPivLu().solve(I_inputs);

    // Starting main loop
    while(true)
    {
        double it_time = clock();
        // Generate reference trajectories
        for(uint i = 0; i < number_time_steps; i++)
        {
            xh0[i] = x0[i] - x[i];
            uh0[i] = u0[i] - u[i];
        }

        // If checking safety, generate obstacles repulsive cost
        std::vector<Eigen::VectorXd> obstacles_repulsive_cost(number_time_steps,
                                                              Eigen::VectorXd::Zero(number_states));

        if(check_safety)
        {
            for(uint i = 0; i < number_time_steps; i++)
            {
                robot_ss_model->getObstaclesCost(x[i],
                                                 map_resolution,
                                                 gradient_obstacles_map_x,
                                                 gradient_obstacles_map_y,
                                                 time_horizon,
                                                 obstacles_repulsive_cost[i]);
            }
        }

        // LQR problem solution
        std::vector<Eigen::MatrixXd> M(number_time_steps, Eigen::MatrixXd::Identity(number_states,number_states));
        std::vector<Eigen::MatrixXd> P = Qh;
        std::vector<Eigen::VectorXd> s = obstacles_repulsive_cost;

        P = Qh;
        s = obstacles_repulsive_cost;
        s[number_time_steps - 1].noalias() -= Qh[number_time_steps - 1] * xh0[number_time_steps - 1];

        // Solve backwards
        std::vector<Eigen::MatrixXd> Rh_inv_Bh_t(number_time_steps);
        std::vector<Eigen::MatrixXd> Rh_inv_Bh_t_s1(number_time_steps);

        for(int i = number_time_steps - 2; i >= 0; i--)
        {
            Eigen::MatrixXd Ah_trans = Ah[i].transpose();
            Rh_inv_Bh_t[i] = Rh_inv * Bh[i].transpose();

            M[i].noalias() += Bh[i] * Rh_inv_Bh_t[i] * P[i + 1];
            M[i].noalias() = M[i].partialPivLu().solve(I_states);

            Eigen::MatrixXd Ah_trans_P1_M = Ah_trans * P[i+1] * M[i];

            P[i].noalias() += Ah_trans_P1_M * Ah[i];

            Eigen::MatrixXd Ah_trans_P1_M_Bh = Ah_trans_P1_M * Bh[i];
            Rh_inv_Bh_t_s1[i] = Rh_inv_Bh_t[i] * s[i + 1];
            s[i].noalias() += Ah_trans * s[i + 1];
            s[i].noalias() -= Ah_trans_P1_M_Bh * Rh_inv_Bh_t_s1[i];
            s[i].noalias() += Ah_trans_P1_M_Bh * uh0[i];
            s[i].noalias() -= Qh[i] * xh0[i];
        }

        std::vector<Eigen::VectorXd> v(number_time_steps, Eigen::VectorXd::Zero(number_states));
        std::vector<Eigen::VectorXd> xh(number_time_steps, Eigen::VectorXd::Zero(number_states));
        std::vector<Eigen::VectorXd> lambdah = s;
        std::vector<Eigen::VectorXd> uh = uh0;

        // Solve forwards
        for(uint i = 0; i < number_time_steps - 1; i++)
        {
            v[i] = M[i]*Bh[i] * (uh0[i] - Rh_inv_Bh_t_s1[i]);
            xh[i + 1] = v[i];
            xh[i + 1].noalias() += M[i] * (Ah[i] * xh[i]);
            lambdah[i + 1].noalias() += P[i + 1] * xh[i + 1];
            uh[i].noalias() -= Rh_inv_Bh_t[i] * lambdah[i + 1];
        }

        // Decide the best way to apply the last obtained state and control
        // steps (xh and uh)
        computeLineSearch(x, x0, u, u0, uh, Qh, Rh);

        // Checking termination conditions
        bool convergence_condition = true;
        double distance_to_goal;
        double orientation_to_goal;
        std::cout << blue << "[MotionPlanner::generateUnconstrainedMotionPlan]: Iteration number "
                  << number_iterations <<nocolor<< std::endl;
        std::cout << yellow
                  << "[MotionPlanner::generateUnconstrainedMotionPlan]: Elapsed iteration time: " << (double)(clock() - it_time) / CLOCKS_PER_SEC
                  << nocolor << std::endl;
        number_iterations++;

        // Check if the control is not changing
        for(uint i = 0; i < number_time_steps; i++)
            convergence_condition &= (uh[i].norm() <= control_threshold * u[i].norm());

        // If the control can be improved, check if a suitable solution is already planned
        if(!convergence_condition)
        {
            // Check if the distance objective has been reached
            if(check_distance)
            {
                Eigen::VectorXd termination_state(distance_indexes.size());
                Eigen::VectorXd termination_goal(distance_indexes.size());

                for(uint i = 0; i < distance_indexes.size(); i++)
                {
                    termination_state(i) = x[number_time_steps - 1](distance_indexes[i]);
                    termination_goal(i) = x0[number_time_steps - 1](distance_indexes[i]);
                }
                distance_to_goal = (termination_state - termination_goal).norm();

                if(distance_to_goal < distance_threshold)
                {
                    convergence_condition = true;

                    for(uint i = 0; i < number_time_steps; i++)
                        convergence_condition &=
                            (uh[i].norm() <= (20 * control_threshold * u[i].norm()));
                }
                std::cout << blue << "[MotionPlanner::generateUnconstrainedMotionPlan]: Distance to "
                             "goal position: "
                          << distance_to_goal <<nocolor<< std::endl;

                if(!convergence_condition) convergence_status = -1;
            }

            // Check if the orientation objective has been reached
            if(check_orientation && convergence_condition)
            {
                Eigen::VectorXd termination_state(orientation_indexes.size());
                Eigen::VectorXd termination_goal(orientation_indexes.size());
                for(uint i = 0; i < orientation_indexes.size(); i++)
                {
                    termination_state(i) = x[number_time_steps - 1](orientation_indexes[i]);
                    termination_goal(i) = x0[number_time_steps - 1](orientation_indexes[i]);
                }

                orientation_to_goal = (termination_state - termination_goal).norm();

                convergence_condition &= (orientation_to_goal < orientation_threshold);

                std::cout << blue << "[MotionPlanner::generateUnconstrainedMotionPlan]: Distance to "
                             "goal orientation: "
                          << orientation_to_goal << nocolor <<std::endl;

                if(!convergence_condition) convergence_status = -1;
            }

            // Check if the planned motion is safe
            if(check_safety && convergence_condition)
            {
                for(uint i = 0; i < number_time_steps; i++)
                {
                    std::vector<double> pose = {x[i](pose_indexes[0]), x[i](pose_indexes[1])};
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
            std::cout << green
                      << "[MotionPlanner::generateUnconstrainedMotionPlan]: The motion planner "
                         "found a solution!"
                      << nocolor << std::endl;

            planned_state = x;
            planned_control = u;
            is_motion_planned = true;

            return 1;
        }
        else
        {
            // Update linearization
            if(!updateLinearModel(x, u, Ah, Bh))
            {
                std::cout
                    << red
                    << "ERROR [MotionPlanner::generateUnconstrainedMotionPlan]: Unable to update "
                       "the system linear matrixes"
                    << nocolor << std::endl;
                return 0;
            }
        }

        // If the algorithm finally failed to converge
        if(number_iterations > max_iter)
        {
            std::cout << red
                      << "ERROR [MotionPlanner::generateUnconstrainedMotionPlan]: The motion "
                         "planner failed to find a solution, ";

            switch(convergence_status)
            {
                case -1:
                    std::cout << "the goal distance threshold was not satisfied";
                    break;
                case -2:
                    std::cout << "the goal control threshold was not satisfied";
                    break;
                case -3:
                    std::cout << "the generated solution was not safe";
                    break;
                default:
                    std::cout << "something unexpected happened";
                    break;
            }

            std::cout << nocolor << std::endl;
            return convergence_status;
        }

    }

    return 1;
}

int MotionPlanner::generateUnconstrainedMotionPlan(const std::vector<double> & x_ini,
                                                   const std::vector<std::vector<double>> & x0,
                                                   const std::vector<double> & u_ini,
                                                   const std::vector<std::vector<double>> & u0,
                                                   uint max_iter)
{
    if(x0.size() != number_time_steps)
    {
        std::cout << red
                  << "ERROR [MotionPlanner::generateUnconstrainedMotionPlan]: The provided goal "
                     "state matrix has a wrong number of time steps"
                  << nocolor << std::endl;
        return -1;
    }

    if(u0.size() != number_time_steps)
    {
        std::cout << red
                  << "ERROR [MotionPlanner::generateUnconstrainedMotionPlan]: The provided goal "
                     "input matrix has a wrong number of time steps"
                  << nocolor << std::endl;
        return -1;
    }

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

    // Generating the state, linearized matrixes and cost matrixes for the whole time horizon
    if(!generateHorizon(x, u, Ah, Bh, Qh, Rh, Kh))
    {
        std::cout << red
                  << "ERROR [MotionPlanner::generateUnconstrainedMotionPlan]: Unable to generate "
                     "the system state, linear matrixes and costs for the time horizon"
                  << nocolor << std::endl;
        return 0;
    }

    // Initializing reference trajectories
    std::vector<std::vector<double>> xh0(number_time_steps, std::vector<double>(number_states, 0));
    std::vector<std::vector<double>> uh0(number_time_steps, std::vector<double>(number_inputs, 0));

    // Auxiliary variables
    std::vector<std::vector<double>> I_states = getIdentity(number_states);
    std::vector<std::vector<double>> I_inputs = getIdentity(number_inputs);
    Eigen::MatrixXd I_eig = Eigen::MatrixXd::Identity(number_states, number_states);
    Eigen::MatrixXd Rh_eig = getEigenMatrix(Rh[0]);
    Eigen::MatrixXd Rh_inv_eig = Rh_eig.inverse();
    std::vector<std::vector<double>> Rh_inv = getMatrixFromEigen(Rh_inv_eig);

    // Starting main loop
    while(true)
    {
        double it_time = clock();
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
                                                 time_horizon,
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
        for(int i = number_time_steps - 2; i >= 0; i--)
        {
            std::vector<std::vector<double>> Ah_trans = getTranspose(Ah[i]);
            std::vector<std::vector<double>> Bh_trans = getTranspose(Bh[i]);
            std::vector<std::vector<double>> Rh_inv_Bh_t = dot(Rh_inv, Bh_trans);

            M[i] = getSum(I_states, dot(Bh[i], dot(Rh_inv_Bh_t, P[i + 1])));
            Eigen::MatrixXd M_aux = getEigenMatrix(M[i]).inverse();
            M[i] = getMatrixFromEigen(M_aux);

            P[i] = getSum(Qh[i], dot(Ah_trans, dot(P[i + 1], dot(M[i], Ah[i]))));

            s[i] = getSum(
                getSum(dot(getTranspose(Ah[i]),
                           dot(getDifference(getIdentity(number_states),
                                             dot(P[i + 1], dot(M[i], dot(Bh[i], Rh_inv_Bh_t)))),
                               s[i + 1])),
                       dot(getTranspose(Ah[i]), dot(P[i + 1], dot(M[i], dot(Bh[i], uh0[i]))))),
                getDifference(obstacles_repulsive_cost[i], dot(Qh[i], xh0[i])));
        }

        // Solve forwards
        for(uint i = 0; i < number_time_steps - 1; i++)
        {
            std::vector<std::vector<double>> Bh_trans = getTranspose(Bh[i]);
            std::vector<std::vector<double>> Rh_inv_Bh_t = dot(Rh_inv, Bh_trans);

            v[i] = dot(M[i], dot(Bh[i], getDifference(uh0[i], dot(Rh_inv_Bh_t, s[i + 1]))));
            xh[i + 1] = getSum(dot(M[i], dot(Ah[i], xh[i])), v[i]);
            lambdah[i + 1] = getSum(dot(P[i + 1], xh[i + 1]), s[i + 1]);
            uh[i] = getDifference(uh0[i], dot(Rh_inv_Bh_t, lambdah[i + 1]));
        }

        // Decide the best way to apply the last obtained state and control
        // steps (xh and uh)
        computeLineSearch(x, x0, u, u0, uh, Qh, Rh);

        // Checking termination conditions
        bool convergence_condition = true;
        double distance_to_goal;
        double orientation_to_goal;
        std::cout << "[MotionPlanner::generateUnconstrainedMotionPlan]: Iteration number "
                  << number_iterations << std::endl;
        number_iterations++;

        // Check if the control is not changing
        for(uint i = 0; i < number_time_steps; i++)
            convergence_condition &= (getNorm(uh[i]) <= control_threshold * getNorm(u[i]));

        // If the control can be improved, check if a suitable solution is already planned
        if(!convergence_condition)
        {
            // Check if the distance objective has been reached
            if(check_distance)
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
                            (getNorm(uh[i]) <= (20 * control_threshold * getNorm(u[i])));
                }
                std::cout << "[MotionPlanner::generateUnconstrainedMotionPlan]: Distance to "
                             "goal position: "
                          << distance_to_goal << std::endl;

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

                std::cout << "[MotionPlanner::generateUnconstrainedMotionPlan]: Distance to "
                             "goal orientation: "
                          << orientation_to_goal << std::endl;

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
            std::cout << green
                      << "[MotionPlanner::generateUnconstrainedMotionPlan]: The motion planner "
                         "found a solution!"
                      << nocolor << std::endl;

            planned_state_vector = x;
            planned_control_vector = u;
            is_motion_planned = true;

            return 1;
        }
        else
        {
            // Update linearization
            if(!updateLinearModel(x, u, Ah, Bh))
            {
                std::cout
                    << red
                    << "ERROR [MotionPlanner::generateUnconstrainedMotionPlan]: Unable to update "
                       "the system linear matrixes"
                    << nocolor << std::endl;
                return 0;
            }
        }

        // If the algorithm finally failed to converge
        if(number_iterations > max_iter)
        {
            std::cout << red
                      << "ERROR [MotionPlanner::generateUnconstrainedMotionPlan]: The motion "
                         "planner failed to find a solution, ";

            switch(convergence_status)
            {
                case -1:
                    std::cout << "the goal distance threshold was not satisfied";
                    break;
                case -2:
                    std::cout << "the goal control threshold was not satisfied";
                    break;
                case -3:
                    std::cout << "the generated solution was not safe";
                    break;
                default:
                    std::cout << "something unexpected happened";
                    break;
            }

            std::cout << nocolor << std::endl;
            return convergence_status;
        }

        std::cout << yellow
                  << "Elapsed iteration time: " << (double)(clock() - it_time) / CLOCKS_PER_SEC
                  << nocolor << std::endl;
    }

    return 1;
}

int MotionPlanner::generateConstrainedMotionPlan(const Eigen::VectorXd & x_ini,
                                                 const std::vector<Eigen::VectorXd> & x0,
                                                 const Eigen::VectorXd & u_ini,
                                                 const std::vector<Eigen::VectorXd> & u0,
                                                 uint max_iter)
{
    return 1;
}

int MotionPlanner::generateSteppedMotionPlan(const Eigen::VectorXd & x_ini,
                                             const std::vector<Eigen::VectorXd> & x0,
                                             const Eigen::VectorXd & u_ini,
                                             const std::vector<Eigen::VectorXd> & u0)
{
    return 1;
}

bool MotionPlanner::getPlannedState(std::vector<Eigen::VectorXd> & x)
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
        x = planned_state_vector;

    return true;
}

bool MotionPlanner::getPlannedControl(std::vector<Eigen::VectorXd> & u)
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
    {
        u = planned_control;
    }

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
        u = planned_control_vector;

    return true;
}
