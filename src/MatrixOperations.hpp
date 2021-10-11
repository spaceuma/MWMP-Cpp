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
std::vector<std::vector<double>> dot(std::vector<std::vector<double>> A,
                                     std::vector<std::vector<double>> B);

std::vector<double> dot(std::vector<std::vector<double>> A, std::vector<double> b);

std::vector<double> dot(double n, std::vector<double> a);

double dot(std::vector<double> a, std::vector<double> b);

std::vector<std::vector<double>> getTraslation(std::vector<double> position);

std::vector<std::vector<double>> getXrot(double angle);

std::vector<std::vector<double>> getYrot(double angle);

std::vector<std::vector<double>> getZrot(double angle);

double getDeterminant(const std::vector<std::vector<double>> * A);

std::vector<std::vector<double>> getCofactor(const std::vector<std::vector<double>> * A,
                                             int row,
                                             int col);

std::vector<std::vector<double>> getAdjoint(const std::vector<std::vector<double>> * A);

std::vector<std::vector<double>> getInverse(const std::vector<std::vector<double>> * A);

std::vector<double> getCrossProduct(std::vector<double> a, std::vector<double> b);

std::vector<double> getSum(std::vector<double> a, std::vector<double> b);

std::vector<double> getDifference(std::vector<double> a, std::vector<double> b);

double getNorm(std::vector<double> a);
}    // namespace MatrixOperations
#endif
// namespace MatrixOperations
