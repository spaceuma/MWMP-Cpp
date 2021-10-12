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

#include <exception>
#include <vector>

#define pi 3.14159265359

#define reset "\033[0m"
#define black "\033[1;30m"
#define red "\033[1;31m"
#define green "\033[1;32m"
#define yellow "\033[1;33m"
#define blue "\033[1;34m"
#define magenta "\033[1;35m"
#define cyan "\033[1;36m"
#define white "\033[1;37m"

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

std::vector<double> dot(const std::vector<std::vector<double>> & A, const std::vector<double> & b);

bool dot(const std::vector<std::vector<double>> & A,
         const std::vector<double> & b,
         std::vector<double> & c);

double dot(const std::vector<double> & a, const std::vector<double> & b);

std::vector<double> dot(double n, const std::vector<double> & a);

bool dot(double n, const std::vector<double> & a, std::vector<double> & c);

//**********************************//
// Traslation transformation matrix //
//**********************************//
std::vector<std::vector<double>> getTraslation(const std::vector<double> & position);

bool getTraslation(const std::vector<double> & position, std::vector<std::vector<double>> & T);

//**********************************//
// Rotation transformation matrixes //
//**********************************//
std::vector<std::vector<double>> getXrot(double angle);

bool getXrot(double angle, std::vector<std::vector<double>> & T);

std::vector<std::vector<double>> getYrot(double angle);

bool getYrot(double angle, std::vector<std::vector<double>> & T);

std::vector<std::vector<double>> getZrot(double angle);

bool getZrot(double angle, std::vector<std::vector<double>> & T);

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
                std::vector<std::vector<double>> & inverse);

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
std::vector<std::vector<double>> getDifference(const std::vector<std::vector<double>> & a,
                                               const std::vector<std::vector<double>> & b);

bool getDifference(const std::vector<std::vector<double>> & a,
                   const std::vector<std::vector<double>> & b,
                   std::vector<std::vector<double>> & c);

//**************************//
// Vector magnitude or norm //
//**************************//
double getNorm(const std::vector<double> & a);

}    // namespace MatrixOperations
#endif
