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

#ifndef __MATRIX_OPERATIONS__
#define __MATRIX_OPERATIONS__

#include <Eigen/Dense>
#include <exception>
#include <vector>

#define NOCOLOR "\033[0m"
#define BLACK "\033[1;30m"
#define RED "\033[1;31m"
#define GREEN "\033[1;32m"
#define YELLOW "\033[1;33m"
#define BLUE "\033[1;34m"
#define MAGENTA "\033[1;35m"
#define CYAN "\033[1;36m"
#define WHITE "\033[1;37m"

namespace MatrixOperations
{
//***********************************//
// Matrixes and vectors dot products //
//***********************************//
std::vector<std::vector<double>> dot(const std::vector<std::vector<double>> & A,
                                     const std::vector<std::vector<double>> & B);

bool dot(const std::vector<std::vector<double>> & A,
         const std::vector<std::vector<double>> & B,
         std::vector<std::vector<double>> & C);

std::vector<double> dot(const std::vector<double> & a, const std::vector<std::vector<double>> & B);

bool dot(const std::vector<double> & a,
         const std::vector<std::vector<double>> & B,
         std::vector<double> & c);

std::vector<double> dot(const std::vector<std::vector<double>> & A, const std::vector<double> & b);

bool dot(const std::vector<std::vector<double>> & A,
         const std::vector<double> & b,
         std::vector<double> & c);

double dot(const std::vector<double> & a, const std::vector<double> & b);

std::vector<double> dot(double n, const std::vector<double> & a);

bool dot(double n, const std::vector<double> & a, std::vector<double> & c);

std::vector<std::vector<double>> dot(double n, const std::vector<std::vector<double>> & A);

bool dot(double n,
         const std::vector<std::vector<double>> & A,
         std::vector<std::vector<double>> & C);

//**********************************//
// Traslation transformation matrix //
//**********************************//
std::vector<std::vector<double>> getTraslation(const std::vector<double> & position);

Eigen::Matrix<double, 4, 4> getTraslationTransform(const std::vector<double> & position);

bool getTraslation(const std::vector<double> & position, std::vector<std::vector<double>> & T);

//**********************************//
// Rotation transformation matrixes //
//**********************************//
std::vector<std::vector<double>> getXRot(double angle);

Eigen::Matrix<double, 4, 4> getXRotTransform(double angle);

bool getXRot(double angle, std::vector<std::vector<double>> & T);

std::vector<std::vector<double>> getYRot(double angle);

Eigen::Matrix<double, 4, 4> getYRotTransform(double angle);

bool getYRot(double angle, std::vector<std::vector<double>> & T);

std::vector<std::vector<double>> getZRot(double angle);

Eigen::Matrix<double, 4, 4> getZRotTransform(double angle);

bool getZRot(double angle, std::vector<std::vector<double>> & T);

//*********************//
// Get identity matrix //
//*********************//
std::vector<std::vector<double>> getIdentity(int size);

bool getIdentity(std::vector<std::vector<double>> & I);

//****************************************************//
// Matrix essential operations (determinant, inverse) //
//****************************************************//
double getDeterminant(const std::vector<std::vector<double>> & A);

std::vector<std::vector<double>> getCofactor(const std::vector<std::vector<double>> & A,
                                             int row,
                                             int col);

bool getCofactor(const std::vector<std::vector<double>> & A,
                 int row,
                 int col,
                 std::vector<std::vector<double>> & subA);

std::vector<std::vector<double>> getAdjoint(const std::vector<std::vector<double>> & A);

bool getAdjoint(const std::vector<std::vector<double>> & A, std::vector<std::vector<double>> & adj);

std::vector<std::vector<double>> getInverse(const std::vector<std::vector<double>> & A);

bool getInverse(const std::vector<std::vector<double>> & A,
                std::vector<std::vector<double>> & invA);

//*****************************//
// Cross product of 3D vectors //
//*****************************//
std::vector<double> getCrossProduct(const std::vector<double> & a, const std::vector<double> & b);

bool getCrossProduct(const std::vector<double> & a,
                     const std::vector<double> & b,
                     std::vector<double> & c);

//************************//
// General sum of vectors //
//************************//
std::vector<double> getSum(const std::vector<double> & a, const std::vector<double> & b);

bool getSum(const std::vector<double> & a, const std::vector<double> & b, std::vector<double> & c);

//*************************//
// General sum of matrixes //
//*************************//
std::vector<std::vector<double>> getSum(const std::vector<std::vector<double>> & A,
                                        const std::vector<std::vector<double>> & B);

bool getSum(const std::vector<std::vector<double>> & A,
            const std::vector<std::vector<double>> & B,
            std::vector<std::vector<double>> & C);

//*******************************//
// General difference of vectors //
//*******************************//
std::vector<double> getDifference(const std::vector<double> & a, const std::vector<double> & b);

bool getDifference(const std::vector<double> & a,
                   const std::vector<double> & b,
                   std::vector<double> & c);

//********************************//
// General difference of matrixes //
//********************************//
std::vector<std::vector<double>> getDifference(const std::vector<std::vector<double>> & A,
                                               const std::vector<std::vector<double>> & B);

bool getDifference(const std::vector<std::vector<double>> & A,
                   const std::vector<std::vector<double>> & B,
                   std::vector<std::vector<double>> & C);

//**************************//
// Vector magnitude or norm //
//**************************//
double getNorm(const std::vector<double> & a);

//******************//
// Matrix transpose //
//******************//
std::vector<std::vector<double>> getTranspose(const std::vector<std::vector<double>> & A);

bool getTranspose(const std::vector<std::vector<double>> & A,
                  std::vector<std::vector<double>> & At);

//****************//
// Vector 2 Eigen //
//****************//

Eigen::VectorXd getEigenVector(const std::vector<double> & a);

bool getEigenVector(const std::vector<double> & a, Eigen::VectorXd & b);

Eigen::MatrixXd getEigenMatrix(const std::vector<std::vector<double>> & A);

bool getEigenMatrix(const std::vector<std::vector<double>> & A, Eigen::MatrixXd & B);

//****************//
// Eigen 2 Vector //
//****************//

std::vector<double> getVectorFromEigen(const Eigen::VectorXd & a);

bool getVectorFromEigen(const Eigen::VectorXd & a, std::vector<double> & b);

std::vector<std::vector<double>> getMatrixFromEigen(const Eigen::MatrixXd & A);

bool getMatrixFromEigen(const Eigen::MatrixXd & A, std::vector<std::vector<double>> & B);

}    // namespace MatrixOperations
#endif
