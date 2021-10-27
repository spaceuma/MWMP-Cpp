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

bool MotionPlanner::generateHorizonLinearization(std::vector<Eigen::VectorXd> & x,
                                                 const std::vector<Eigen::VectorXd> & u,
                                                 std::vector<Eigen::MatrixXd> & Ath,
                                                 std::vector<Eigen::MatrixXd> & Bth)
{
    if(!robot_ss_model->getLinearizedMatrixA(x[0], time_step, Ath[0]) ||
       !robot_ss_model->getLinearizedMatrixB(x[0], u[0], time_step, Bth[0]))
    {
        std::cout << red
                  << "ERROR [MotionPlanner::generateHorizonLinearization]: Unable to linearize the "
                     "state space model"
                  << nocolor << std::endl;
        return false;
    }

    for(uint i = 1; i < number_time_steps; i++)
    {
        if(!robot_ss_model->forwardIntegrateModel(x[i - 1], u[i - 1], time_step, x[i]))
        {
            std::cout << red
                      << "ERROR [MotionPlanner::generateHorizonLinearization]: Unable to forward "
                         "integrate the model"
                      << nocolor << std::endl;
            return false;
        }

        if(!robot_ss_model->getLinearizedMatrixA(x[i - 1], time_step, Ath[i]) ||
           !robot_ss_model->getLinearizedMatrixB(x[i - 1], u[i - 1], time_step, Bth[i]))
        {
            std::cout << red
                      << "ERROR [MotionPlanner::generateHorizonLinearization]: Unable to linearize "
                         "the state "
                         "space model"
                      << nocolor << std::endl;
            return false;
        }
    }

    return true;
}

bool MotionPlanner::generateHorizonLinearization(
    std::vector<std::vector<double>> & x,
    const std::vector<std::vector<double>> & u,
    std::vector<std::vector<std::vector<double>>> & Ath,
    std::vector<std::vector<std::vector<double>>> & Bth)
{
    if(!robot_ss_model->getLinearizedMatrixA(x[0], time_step, Ath[0]) ||
       !robot_ss_model->getLinearizedMatrixB(x[0], u[0], time_step, Bth[0]))
    {
        std::cout << red
                  << "ERROR [MotionPlanner::generateHorizonLinearization]: Unable to linearize the "
                     "state space model"
                  << nocolor << std::endl;
        return false;
    }

    for(uint i = 1; i < number_time_steps; i++)
    {
        if(!robot_ss_model->forwardIntegrateModel(x[i - 1], u[i - 1], time_step, x[i]))
        {
            std::cout << red
                      << "ERROR [MotionPlanner::generateHorizonLinearization]: Unable to forward "
                         "integrate the model"
                      << nocolor << std::endl;
            return false;
        }

        if(!robot_ss_model->getLinearizedMatrixA(x[i - 1], time_step, Ath[i]) ||
           !robot_ss_model->getLinearizedMatrixB(x[i - 1], u[i - 1], time_step, Bth[i]))
        {
            std::cout << red
                      << "ERROR [MotionPlanner::generateHorizonLinearization]: Unable to linearize "
                         "the state "
                         "space model"
                      << nocolor << std::endl;
            return false;
        }
    }

    return true;
}

bool MotionPlanner::generateHorizonCosts(std::vector<Eigen::MatrixXd> & Qth,
                                         std::vector<Eigen::MatrixXd> & Rth,
                                         std::vector<Eigen::MatrixXd> & Kth)
{
    if(!robot_ss_model->getStateCostMatrix(0, time_horizon, Qth[0], track_reference_trajectory) ||
       !robot_ss_model->getInputCostMatrix(Rth[0], time_horizon) ||
       !robot_ss_model->getStateInputCostMatrix(Kth[0]))
    {
        std::cout << red
                  << "ERROR [MotionPlanner::generateHorizonCosts]: Unable to compute the "
                     "quadratized costs"
                  << nocolor << std::endl;
        return false;
    }

    for(uint i = 1; i < number_time_steps; i++)
    {
        double percentage_horizon = 100 * ((double)i + 1) / (double)number_time_steps;

        if(!robot_ss_model->getStateCostMatrix(
               percentage_horizon, time_horizon, Qth[i], track_reference_trajectory) ||
           !robot_ss_model->getInputCostMatrix(Rth[i], time_horizon) ||
           !robot_ss_model->getStateInputCostMatrix(Kth[i]))
        {
            std::cout << red
                      << "ERROR [MotionPlanner::generateHorizonCosts]: Unable to compute the "
                         "quadratized costs"
                      << nocolor << std::endl;
            return false;
        }
    }

    return true;
}

bool MotionPlanner::generateHorizonCosts(std::vector<std::vector<std::vector<double>>> & Qth,
                                         std::vector<std::vector<std::vector<double>>> & Rth,
                                         std::vector<std::vector<std::vector<double>>> & Kth)
{
    if(!robot_ss_model->getStateCostMatrix(0, time_horizon, Qth[0]) ||
       !robot_ss_model->getInputCostMatrix(Rth[0], time_horizon) ||
       !robot_ss_model->getStateInputCostMatrix(Kth[0]))
    {
        std::cout << red
                  << "ERROR [MotionPlanner::generateHorizonCosts]: Unable to compute the "
                     "quadratized costs"
                  << nocolor << std::endl;
        return false;
    }

    for(uint i = 1; i < number_time_steps; i++)
    {
        double percentage_horizon = 100 * ((double)i + 1) / (double)number_time_steps;

        if(!robot_ss_model->getStateCostMatrix(percentage_horizon, time_horizon, Qth[i]) ||
           !robot_ss_model->getInputCostMatrix(Rth[i], time_horizon) ||
           !robot_ss_model->getStateInputCostMatrix(Kth[i]))
        {
            std::cout << red
                      << "ERROR [MotionPlanner::generateHorizonCosts]: Unable to compute the "
                         "quadratized costs"
                      << nocolor << std::endl;
            return false;
        }
    }

    return true;
}

bool MotionPlanner::updateLinearModel(const std::vector<Eigen::VectorXd> & x,
                                      const std::vector<Eigen::VectorXd> & u,
                                      std::vector<Eigen::MatrixXd> & Ath,
                                      std::vector<Eigen::MatrixXd> & Bth)
{
    for(uint i = 1; i < number_time_steps; i++)
    {
        if(!robot_ss_model->getLinearizedMatrixA(x[i - 1], time_step, Ath[i]) ||
           !robot_ss_model->getLinearizedMatrixB(x[i - 1], u[i - 1], time_step, Bth[i]))
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
                                      std::vector<std::vector<std::vector<double>>> & Ath,
                                      std::vector<std::vector<std::vector<double>>> & Bth)
{
    for(uint i = 1; i < number_time_steps; i++)
    {
        if(!robot_ss_model->getLinearizedMatrixA(x[i - 1], time_step, Ath[i]) ||
           !robot_ss_model->getLinearizedMatrixB(x[i - 1], u[i - 1], time_step, Bth[i]))
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

bool MotionPlanner::generateHorizonConstraints(std::vector<Eigen::MatrixXd> & Cth,
                                               std::vector<Eigen::MatrixXd> & Dth,
                                               std::vector<Eigen::VectorXd> & rth,
                                               std::vector<Eigen::MatrixXd> & Gth,
                                               std::vector<Eigen::VectorXd> & hth)
{
    for(uint i = 0; i < number_time_steps; i++)
    {
        if(!robot_ss_model->getConstraintsMatrixC(Cth[i]) ||
           !robot_ss_model->getConstraintsMatrixD(Dth[i]) ||
           !robot_ss_model->getConstraintsMatrixR(rth[i]))
        {
            std::cout << red
                      << "ERROR [MotionPlanner::generateHorizonConstraints]: Unable to compute the "
                         "state input constraints"
                      << nocolor << std::endl;
            return false;
        }
        if(!robot_ss_model->getConstraintsMatrixG(Gth[i]) ||
           !robot_ss_model->getConstraintsMatrixH(hth[i]))
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

bool MotionPlanner::generateHorizonConstraints(std::vector<std::vector<std::vector<double>>> & Cth,
                                               std::vector<std::vector<std::vector<double>>> & Dth,
                                               std::vector<std::vector<double>> & rth,
                                               std::vector<std::vector<std::vector<double>>> & Gth,
                                               std::vector<std::vector<double>> & hth)
{
    for(uint i = 0; i < number_time_steps; i++)
    {
        if(!robot_ss_model->getConstraintsMatrixC(Cth[i]) ||
           !robot_ss_model->getConstraintsMatrixD(Dth[i]) ||
           !robot_ss_model->getConstraintsMatrixR(rth[i]))
        {
            std::cout << red
                      << "ERROR [MotionPlanner::generateHorizonConstraints]: Unable to compute the "
                         "state input constraints"
                      << nocolor << std::endl;
            return false;
        }
        if(!robot_ss_model->getConstraintsMatrixG(Gth[i]) ||
           !robot_ss_model->getConstraintsMatrixH(hth[i]))
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

bool MotionPlanner::computeCostMap(const std::vector<std::vector<uint>> & obst_map,
                                   const std::vector<std::vector<uint>> & safe_obst_map)
{
    uint m = obst_map.size();
    uint n = obst_map[0].size();

    // Obtaining the distance to obstacles
    cv::Mat cv_obst_map = cv::Mat::ones(m, n, CV_8UC1);
    cv::Mat cv_distance = cv::Mat::zeros(m, n, CV_32FC1);

    // Computing distance
    for(uint i = 0; i < m; i++)
        for(uint j = 0; j < n; j++)
            cv_obst_map.at<uchar>(i, j) = 1 - obst_map[i][j];

    cv::distanceTransform(cv_obst_map, cv_distance, cv::DIST_L2, 5);
    cv::Mat cv_aux_map = 1 / cv_distance;
    double min;
    cv::minMaxLoc(cv_aux_map, &min);

    for(uint i = 0; i < m; i++)
        for(uint j = 0; j < n; j++)
        {
            if(safe_obst_map[i][j]) cost_map[i][j] = (1 + cv_aux_map.at<float>(i, j) / min) / 2;
            if(obst_map[i][j]) cost_map[i][j] = inf;
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

bool MotionPlanner::checkConstraints(bool & constraints_satisfied)
{
    if(!is_motion_planned)
    {
        std::cout << red
                  << "ERROR [MotionPlanner::checkConstraints]: The motion planner has not yet been "
                     "generated"
                  << nocolor << std::endl;
        return false;
    }

    // Checking the constraints in the planned state
    for(uint n = 0; n < number_time_steps; n++)
    {
        for(uint i = 0; i < number_si_constraints; i++)
        {
            double rhoi = Ch[n].row(i) * planned_state[n];
            rhoi += Dh[n].row(i) * planned_control[n];
            rhoi += rh[n](i);
            if(rhoi > 1e-4)
            {
                constraints_satisfied = false;
                return true;
            }
        }
        for(uint j = 0; j < number_ps_constraints; j++)
        {
            double rhoj = Gh[n].row(j) * planned_state[n] + hh[n](j);
            if(rhoj > 1e-4)
            {
                constraints_satisfied = false;
                return true;
            }
        }
    }

    constraints_satisfied = true;
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

    std::vector<Eigen::MatrixXd> Qh_aux(number_time_steps,
                                        Eigen::MatrixXd::Zero(number_states, number_states));
    std::vector<Eigen::MatrixXd> Rh_aux(number_time_steps,
                                        Eigen::MatrixXd::Zero(number_inputs, number_inputs));
    std::vector<Eigen::MatrixXd> Kh_aux(number_time_steps,
                                        Eigen::MatrixXd::Zero(number_states, number_inputs));

    if(!generateHorizonCosts(Qh_aux, Rh_aux, Kh_aux))
    {
        throw std::domain_error(
            red +
            std::string(
                "ERROR [MotionPlanner::MotionPlanner]: Failure generating the cost matrixes") +
            nocolor);
    }

    Qh = Qh_aux;
    Rh = Rh_aux;
    Kh = Kh_aux;

    std::vector<Eigen::MatrixXd> Ch_aux(
        number_time_steps, Eigen::MatrixXd::Zero(number_si_constraints, number_states));
    std::vector<Eigen::MatrixXd> Dh_aux(
        number_time_steps, Eigen::MatrixXd::Zero(number_si_constraints, number_inputs));
    std::vector<Eigen::VectorXd> rh_aux(number_time_steps,
                                        Eigen::VectorXd::Zero(number_si_constraints));
    std::vector<Eigen::MatrixXd> Gh_aux(
        number_time_steps, Eigen::MatrixXd::Zero(number_ps_constraints, number_states));
    std::vector<Eigen::VectorXd> hh_aux(number_time_steps,
                                        Eigen::VectorXd::Zero(number_ps_constraints));

    if(!generateHorizonConstraints(Ch_aux, Dh_aux, rh_aux, Gh_aux, hh_aux))
    {
        throw std::domain_error(red +
                                std::string("ERROR [MotionPlanner::MotionPlanner]: Failure "
                                            "generating the constraints matrixes") +
                                nocolor);
    }

    Ch = Ch_aux;
    Dh = Dh_aux;
    rh = rh_aux;
    Gh = Gh_aux;
    hh = hh_aux;
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
    track_reference_trajectory = false;

    number_states = robot_ss_model->getNumberStates();
    number_inputs = robot_ss_model->getNumberInputs();
    number_si_constraints = robot_ss_model->getNumberStateInputConstraints();
    number_ps_constraints = robot_ss_model->getNumberPureStateConstraints();

    std::vector<Eigen::MatrixXd> Qh_aux(number_time_steps,
                                        Eigen::MatrixXd::Zero(number_states, number_states));
    std::vector<Eigen::MatrixXd> Rh_aux(number_time_steps,
                                        Eigen::MatrixXd::Zero(number_inputs, number_inputs));
    std::vector<Eigen::MatrixXd> Kh_aux(number_time_steps,
                                        Eigen::MatrixXd::Zero(number_states, number_inputs));

    if(!generateHorizonCosts(Qh_aux, Rh_aux, Kh_aux))
    {
        throw std::domain_error(
            red +
            std::string(
                "ERROR [MotionPlanner::MotionPlanner]: Failure generating the cost matrixes") +
            nocolor);
    }

    Qh = Qh_aux;
    Rh = Rh_aux;
    Kh = Kh_aux;

    std::vector<Eigen::MatrixXd> Ch_aux(
        number_time_steps, Eigen::MatrixXd::Zero(number_si_constraints, number_states));
    std::vector<Eigen::MatrixXd> Dh_aux(
        number_time_steps, Eigen::MatrixXd::Zero(number_si_constraints, number_inputs));
    std::vector<Eigen::VectorXd> rh_aux(number_time_steps,
                                        Eigen::VectorXd::Zero(number_si_constraints));
    std::vector<Eigen::MatrixXd> Gh_aux(
        number_time_steps, Eigen::MatrixXd::Zero(number_ps_constraints, number_states));
    std::vector<Eigen::VectorXd> hh_aux(number_time_steps,
                                        Eigen::VectorXd::Zero(number_ps_constraints));

    if(!generateHorizonConstraints(Ch_aux, Dh_aux, rh_aux, Gh_aux, hh_aux))
    {
        throw std::domain_error(red +
                                std::string("ERROR [MotionPlanner::MotionPlanner]: Failure "
                                            "generating the constraints matrixes") +
                                nocolor);
    }

    Ch = Ch_aux;
    Dh = Dh_aux;
    rh = rh_aux;
    Gh = Gh_aux;
    hh = hh_aux;
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
    track_reference_trajectory = config.track_reference_trajectory;
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

    if(track_reference_trajectory)
    {
        cost_map = std::vector<std::vector<double>>(
            obstacles_map.size(), std::vector<double>(obstacles_map[0].size(), 1));
        if(!computeCostMap(obstacles_map, safety_obstacles_map))
        {
            throw std::domain_error(
                red +
                std::string("ERROR [MotionPlanner::MotionPlanner]: Failure while "
                            "generating the path planner cost map") +
                nocolor);
        }
    }

    number_states = robot_ss_model->getNumberStates();
    number_inputs = robot_ss_model->getNumberInputs();
    number_si_constraints = robot_ss_model->getNumberStateInputConstraints();
    number_ps_constraints = robot_ss_model->getNumberPureStateConstraints();

    std::vector<Eigen::MatrixXd> Qh_aux(number_time_steps,
                                        Eigen::MatrixXd::Zero(number_states, number_states));
    std::vector<Eigen::MatrixXd> Rh_aux(number_time_steps,
                                        Eigen::MatrixXd::Zero(number_inputs, number_inputs));
    std::vector<Eigen::MatrixXd> Kh_aux(number_time_steps,
                                        Eigen::MatrixXd::Zero(number_states, number_inputs));

    if(!generateHorizonCosts(Qh_aux, Rh_aux, Kh_aux))
    {
        throw std::domain_error(
            red +
            std::string(
                "ERROR [MotionPlanner::MotionPlanner]: Failure generating the cost matrixes") +
            nocolor);
    }

    Qh = Qh_aux;
    Rh = Rh_aux;
    Kh = Kh_aux;

    std::vector<Eigen::MatrixXd> Ch_aux(
        number_time_steps, Eigen::MatrixXd::Zero(number_si_constraints, number_states));
    std::vector<Eigen::MatrixXd> Dh_aux(
        number_time_steps, Eigen::MatrixXd::Zero(number_si_constraints, number_inputs));
    std::vector<Eigen::VectorXd> rh_aux(number_time_steps,
                                        Eigen::VectorXd::Zero(number_si_constraints));
    std::vector<Eigen::MatrixXd> Gh_aux(
        number_time_steps, Eigen::MatrixXd::Zero(number_ps_constraints, number_states));
    std::vector<Eigen::VectorXd> hh_aux(number_time_steps,
                                        Eigen::VectorXd::Zero(number_ps_constraints));

    if(!generateHorizonConstraints(Ch_aux, Dh_aux, rh_aux, Gh_aux, hh_aux))
    {
        throw std::domain_error(red +
                                std::string("ERROR [MotionPlanner::MotionPlanner]: Failure "
                                            "generating the constraints matrixes") +
                                nocolor);
    }

    Ch = Ch_aux;
    Dh = Dh_aux;
    rh = rh_aux;
    Gh = Gh_aux;
    hh = hh_aux;
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
    reference_state = x0;
    reference_control = u0;

    if(reference_state.size() != number_time_steps)
    {
        std::cout << red
                  << "ERROR [MotionPlanner::generateUnconstrainedMotionPlan]: The provided goal "
                     "state matrix has a wrong number of time steps"
                  << nocolor << std::endl;
        return -1;
    }

    if(reference_control.size() != number_time_steps)
    {
        std::cout << red
                  << "ERROR [MotionPlanner::generateUnconstrainedMotionPlan]: The provided goal "
                     "input matrix has a wrong number of time steps"
                  << nocolor << std::endl;
        return -1;
    }

    if(track_reference_trajectory)
    {
        FastMarching::PathPlanner * path_planner = new FastMarching::PathPlanner();
        std::vector<double> ini_pose = {x_ini(pose_indexes[0]), x_ini(pose_indexes[1])};
        std::vector<double> goal_pose = {
            reference_state[number_time_steps - 1](distance_indexes[0]),
            reference_state[number_time_steps - 1](distance_indexes[1])};
        std::vector<std::vector<double>> reference_path;

        if(!path_planner->planPath(&cost_map, map_resolution, ini_pose, goal_pose, &reference_path))
        {
            std::cout << red
                      << "ERROR [MotionPlanner::generateUnconstrainedMotionPlan]: Failure while "
                         "computing the reference trajectory"
                      << nocolor << std::endl;
            return -1;
        }

        uint path_size = reference_path.size();
        for(uint i = 0; i < number_time_steps; i++)
        {
            uint path_index = (uint)(path_size * i / number_time_steps);
            reference_state[i](pose_indexes[0]) = reference_path[path_index][0];
            reference_state[i](pose_indexes[1]) = reference_path[path_index][1];
            reference_state[i](pose_indexes[2]) = reference_path[path_index][2];
        }
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

    // Initializing variables to control the status of the algorithm
    uint number_iterations = 0;
    int convergence_status = 0;

    // Generating the state and linearized matrixes for the whole time horizon
    if(!generateHorizonLinearization(x, u, Ah, Bh))
    {
        std::cout << red
                  << "ERROR [MotionPlanner::generateUnconstrainedMotionPlan]: Unable to generate "
                     "the system state and linear matrixes for the time horizon"
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
            xh0[i] = reference_state[i] - x[i];
            uh0[i] = reference_control[i] - u[i];
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
        std::vector<Eigen::MatrixXd> M(number_time_steps,
                                       Eigen::MatrixXd::Identity(number_states, number_states));
        std::vector<Eigen::MatrixXd> P = Qh;
        std::vector<Eigen::VectorXd> s = obstacles_repulsive_cost;

        P = Qh;
        s = obstacles_repulsive_cost;
        s[number_time_steps - 1].noalias() -=
            Qh[number_time_steps - 1] * xh0[number_time_steps - 1];

        // Solve backwards
        std::vector<Eigen::MatrixXd> Rh_inv_Bh_t(number_time_steps);
        std::vector<Eigen::MatrixXd> Rh_inv_Bh_t_s1(number_time_steps);

        for(int i = number_time_steps - 2; i >= 0; i--)
        {
            Eigen::MatrixXd Ah_trans = Ah[i].transpose();
            Rh_inv_Bh_t[i] = Rh_inv * Bh[i].transpose();

            M[i].noalias() += Bh[i] * Rh_inv_Bh_t[i] * P[i + 1];
            M[i].noalias() = M[i].partialPivLu().solve(I_states);

            Eigen::MatrixXd Ah_trans_P1_M = Ah_trans * P[i + 1] * M[i];

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
            v[i] = M[i] * Bh[i] * (uh0[i] - Rh_inv_Bh_t_s1[i]);
            xh[i + 1] = v[i];
            xh[i + 1].noalias() += M[i] * (Ah[i] * xh[i]);
            lambdah[i + 1].noalias() += P[i + 1] * xh[i + 1];
            uh[i].noalias() -= Rh_inv_Bh_t[i] * lambdah[i + 1];
        }

        // Decide the best way to apply the last obtained state and control
        // steps (xh and uh)
        computeLineSearch(x, reference_state, u, reference_control, uh, Qh, Rh);

        // Checking termination conditions
        bool convergence_condition = true;
        double distance_to_goal;
        double orientation_to_goal;
        std::cout << blue << "[MotionPlanner::generateUnconstrainedMotionPlan]: Iteration number "
                  << number_iterations << nocolor << std::endl;
        std::cout << yellow
                  << "[MotionPlanner::generateUnconstrainedMotionPlan]: Elapsed iteration time: "
                  << (double)(clock() - it_time) / CLOCKS_PER_SEC << nocolor << std::endl;
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
                    termination_goal(i) =
                        reference_state[number_time_steps - 1](distance_indexes[i]);
                }
                distance_to_goal = (termination_state - termination_goal).norm();

                if(distance_to_goal < distance_threshold)
                {
                    convergence_condition = true;

                    for(uint i = 0; i < number_time_steps; i++)
                        convergence_condition &=
                            (uh[i].norm() <= (20 * control_threshold * u[i].norm()));
                }
                std::cout << blue
                          << "[MotionPlanner::generateUnconstrainedMotionPlan]: Distance to "
                             "goal position: "
                          << distance_to_goal << nocolor << std::endl;

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
                    termination_goal(i) =
                        reference_state[number_time_steps - 1](orientation_indexes[i]);
                }

                orientation_to_goal = (termination_state - termination_goal).norm();

                convergence_condition &= (orientation_to_goal < orientation_threshold);

                std::cout << blue
                          << "[MotionPlanner::generateUnconstrainedMotionPlan]: Distance to "
                             "goal orientation: "
                          << orientation_to_goal << nocolor << std::endl;

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

int MotionPlanner::generateConstrainedMotionPlan(const Eigen::VectorXd & x_ini,
                                                 const std::vector<Eigen::VectorXd> & x0,
                                                 const std::vector<Eigen::VectorXd> & xs,
                                                 const Eigen::VectorXd & u_ini,
                                                 const std::vector<Eigen::VectorXd> & u0,
                                                 const std::vector<Eigen::VectorXd> & us,
                                                 uint max_iter)
{
    return 1;
}

int MotionPlanner::generateSteppedMotionPlan(const Eigen::VectorXd & x_ini,
                                             const std::vector<Eigen::VectorXd> & x0,
                                             const Eigen::VectorXd & u_ini,
                                             const std::vector<Eigen::VectorXd> & u0)
{
    if(generateUnconstrainedMotionPlan(x_ini, x0, u_ini, u0, (uint)(max_iterations / 2 + 0.4)) != 1)
    {
        std::cout << red
                  << "ERROR [MotionPlanner::generateSteppedMotionPlan]: Failed to generate the "
                     "unconstrained motion plan"
                  << nocolor << std::endl;
        return -1;
    }

    bool constraints_satisfied;
    if(!checkConstraints(constraints_satisfied))
    {
        std::cout << red
                  << "ERROR [MotionPlanner::generateSteppedMotionPlan]: Failure while checking the "
                     "constraints"
                  << nocolor << std::endl;
        return -1;
    }

    if(constraints_satisfied)
    {
        std::cout << green
                  << "[MotionPlanner::generateSteppedMotionPlan]: The imposed constraints are "
                     "satisfied, the stepped motion planner found a solution!"
                  << nocolor << std::endl;

        return 1;
    }

    std::cout << blue
              << "[MotionPlanner::generateSteppedMotionPlan]: Generating a further constrained "
                 "motion plan to ensure the imposed constraints are satisfied"
              << nocolor << std::endl;

    if(generateConstrainedMotionPlan(
           x_ini, x0, planned_state, u_ini, u0, planned_control, (uint)(max_iterations / 2)) != 1)
    {
        std::cout << red
                  << "ERROR [MotionPlanner::generateSteppedMotionPlan]: Failed to generate the "
                     "constrained motion plan"
                  << nocolor << std::endl;
        return -1;
    }

    std::cout << green
              << "[MotionPlanner::generateSteppedMotionPlan]: The imposed constraints are "
                 "satisfied, the stepped motion planner found a solution!"
              << nocolor << std::endl;

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
