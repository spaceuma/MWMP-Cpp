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
#include "StateSpaceModels.hpp"
#include <ctime>
#include <Eigen/Dense>
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

using namespace StateSpaceModels;

TEST(StateSpaceModels, getters_test)
{
    MobileManipulator * exoter_model = new MobileManipulator("exoter");

    EXPECT_EQ(41, exoter_model->getNumberStates());

    EXPECT_EQ(7, exoter_model->getNumberInputs());

    std::vector<uint> goal_distance_indexes = {0, 1, 2};
    EXPECT_EQ(goal_distance_indexes, exoter_model->getIndexesGoalDistance());

    EXPECT_EQ(0.015, exoter_model->getThresholdGoalDistance());

    std::vector<uint> goal_orientation_indexes = {15, 7, 19};
    EXPECT_EQ(goal_orientation_indexes, exoter_model->getIndexesGoalOrientation());

    EXPECT_EQ(0.15, exoter_model->getThresholdGoalOrientation());

    std::vector<uint> robot_pose_indexes = {9, 10};
    EXPECT_EQ(robot_pose_indexes, exoter_model->getIndexesRobotPose());

    EXPECT_EQ(0.30, exoter_model->getRiskDistance());

    EXPECT_EQ(1, exoter_model->getSafetyDistance());
}

TEST(StateSpaceModels, linearized_matrixes_test)
{
    MobileManipulator * exoter_model = new MobileManipulator("exoter");

    // Matrix A
    Eigen::VectorXd x;
    FileManager::readVectorFile("inputs/x.txt", x);

    double time_step = 1.006289308;

    Eigen::MatrixXd A1;
    FileManager::readMatrixFile("results/A.txt", A1);

    double ini_time = clock();
    Eigen::MatrixXd A2 = Eigen::MatrixXd::Zero(x.size(), x.size());
    exoter_model->getLinearizedMatrixA(x, time_step, A2);
    std::cout<<cyan <<"[StateSpaceModels::linearized_matrixes_test] Elapsed time matrix A: "<< (double)(clock() - ini_time) / CLOCKS_PER_SEC << " s" << nocolor<<std::endl;

    for(uint i = 0; i < x.size(); i++)
        for(uint j = 0; j < x.size(); j++)
        {
            A1(i,j) = (int)(A1(i,j)*10000+0.5);
            A1(i,j) /= 10000;
            A2(i,j) = (int)(A2(i,j)*10000+0.5);
            A2(i,j) /= 10000;
        }

    ASSERT_TRUE(A1.isApprox(A2));

    // Matrix B
    Eigen::VectorXd u;
    FileManager::readVectorFile("inputs/u.txt", u);

    Eigen::MatrixXd B1;
    FileManager::readMatrixFile("results/B.txt", B1);

    ini_time = clock();
    Eigen::MatrixXd B2 = Eigen::MatrixXd::Zero(x.size(), u.size());
    exoter_model->getLinearizedMatrixB(x, u, time_step, B2);
    std::cout<<cyan <<"[StateSpaceModels::linearized_matrixes_test] Elapsed time matrix B: "<< (double)(clock() - ini_time) / CLOCKS_PER_SEC << " s" << nocolor<<std::endl;

    for(uint i = 0; i < x.size(); i++)
        for(uint j = 0; j < u.size(); j++)
        {
            B1(i,j) = (int)(B1(i,j)*10000+0.5);
            B1(i,j) /= 10000;
            B2(i,j) = (int)(B2(i,j)*10000+0.5);
            B2(i,j) /= 10000;
        }

    ASSERT_TRUE(B1.isApprox(B2));
}

TEST(StateSpaceModels, constraints_matrixes_test)
{
    MobileManipulator * exoter_model = new MobileManipulator("exoter");

    // Matrix C
    Eigen::MatrixXd C1;
    FileManager::readMatrixFile("results/C.txt", C1);

    double ini_time = clock();
    Eigen::MatrixXd C2 = Eigen::MatrixXd::Zero(exoter_model->getNumberStateInputConstraints(), exoter_model->getNumberStates());
    exoter_model->getConstraintsMatrixC(C2);
    std::cout<<cyan <<"[StateSpaceModels::constraints_matrixes_test] Elapsed time matrix C: "<< (double)(clock() - ini_time) / CLOCKS_PER_SEC << " s" << nocolor<<std::endl;

    for(uint i = 0; i < exoter_model->getNumberStateInputConstraints(); i++)
        for(uint j = 0; j < exoter_model->getNumberStates(); j++)
        {
            C1(i,j) = (int)(C1(i,j)*10000+0.5);
            C1(i,j) /= 10000;
            C2(i,j) = (int)(C2(i,j)*10000+0.5);
            C2(i,j) /= 10000;
        }

    ASSERT_TRUE(C1.isApprox(C2));

    // Matrix D
    Eigen::MatrixXd D1;
    FileManager::readMatrixFile("results/D.txt", D1);

    ini_time = clock();
    Eigen::MatrixXd D2 = Eigen::MatrixXd::Zero(exoter_model->getNumberStateInputConstraints(), exoter_model->getNumberInputs());
    exoter_model->getConstraintsMatrixD(D2);
    std::cout<<cyan <<"[StateSpaceModels::constraints_matrixes_test] Elapsed time matrix D: "<< (double)(clock() - ini_time) / CLOCKS_PER_SEC << " s" << nocolor<<std::endl;

    for(uint i = 0; i < exoter_model->getNumberStateInputConstraints(); i++)
        for(uint j = 0; j < exoter_model->getNumberInputs(); j++)
        {
            D1(i,j) = (int)(D1(i,j)*10000+0.5);
            D1(i,j) /= 10000;
            D2(i,j) = (int)(D2(i,j)*10000+0.5);
            D2(i,j) /= 10000;
        }

    ASSERT_TRUE(D1.isApprox(D2));

    // Vector r
    Eigen::VectorXd r1;
    FileManager::readVectorFile("results/r.txt", r1);

    ini_time = clock();
    Eigen::VectorXd r2 = Eigen::VectorXd::Zero(exoter_model->getNumberStateInputConstraints());
    exoter_model->getConstraintsMatrixR(r2);
    std::cout<<cyan <<"[StateSpaceModels::constraints_matrixes_test] Elapsed time matrix r: "<< (double)(clock() - ini_time) / CLOCKS_PER_SEC << " s" << nocolor<<std::endl;

    ASSERT_TRUE(r1.isApprox(r2));

    // Matrix G
    Eigen::MatrixXd G1;
    FileManager::readMatrixFile("results/G.txt", G1);

    ini_time = clock();
    Eigen::MatrixXd G2 = Eigen::MatrixXd::Zero(exoter_model->getNumberPureStateConstraints(), exoter_model->getNumberStates());
    exoter_model->getConstraintsMatrixG(G2);
    std::cout<<cyan <<"[StateSpaceModels::constraints_matrixes_test] Elapsed time matrix G: "<< (double)(clock() - ini_time) / CLOCKS_PER_SEC << " s" << nocolor<<std::endl;

    ASSERT_TRUE(G1.isApprox(G2));

    // Vector h
    Eigen::VectorXd h1;
    FileManager::readVectorFile("results/h.txt", h1);

    ini_time = clock();
    Eigen::VectorXd h2 = Eigen::VectorXd::Zero(exoter_model->getNumberPureStateConstraints());
    exoter_model->getConstraintsMatrixH(h2);
    std::cout<<cyan <<"[StateSpaceModels::constraints_matrixes_test] Elapsed time matrix h: "<< (double)(clock() - ini_time) / CLOCKS_PER_SEC << " s" << nocolor<<std::endl;

    for(uint i = 0; i < exoter_model->getNumberPureStateConstraints(); i++)
    {
        h1(i) = (int)(h1(i)*10000+0.5);
        h1(i) /= 10000;
        h2(i) = (int)(h2(i)*10000+0.5);
        h2(i) /= 10000;
    }

    ASSERT_TRUE(h1.isApprox(h2));
}
