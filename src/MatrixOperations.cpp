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

#include "MatrixOperations.hpp"
#include <iostream>
#include <math.h>

#define pi 3.14159265359

using namespace MatrixOperations;

std::vector<std::vector<double>> MatrixOperations::dot(const std::vector<std::vector<double>> & A,
                                                       const std::vector<std::vector<double>> & B)
{
    // This function performs matrixes product
    int m = A.size();
    int n = B[0].size();
    std::vector<std::vector<double>> C(m, std::vector<double>(n));

    for(int i = 0; i < m; i++)
        for(int j = 0; j < n; j++)
        {
            C[i][j] = 0;
            for(int k = 0; k < B.size(); k++)
                C[i][j] += A[i][k] * B[k][j];
        }

    return C;
}

bool MatrixOperations::dot(const std::vector<std::vector<double>> & A,
                           const std::vector<std::vector<double>> & B,
                           std::vector<std::vector<double>> & C)
{
    // This function performs matrixes product
    int m = A.size();
    int n = B[0].size();

    if(B.size() != m || B[0].size() != n)
    {
        std::cout << red << "ERROR [MatrixOperations::dot]: Input matrixes sizes don't match"
                  << reset << std::endl;
        return false;
    }

    if(C.size() != m || C[0].size() != n)
    {
        std::cout << red << "ERROR [MatrixOperations::dot]: Wrong output matrix size" << reset
                  << std::endl;
        return false;
    }

    for(int i = 0; i < m; i++)
        for(int j = 0; j < n; j++)
        {
            C[i][j] = 0;
            for(int k = 0; k < B.size(); k++)
                C[i][j] += A[i][k] * B[k][j];
        }

    return true;
}

std::vector<double> MatrixOperations::dot(const std::vector<std::vector<double>> & A,
                                          const std::vector<double> & b)
{
    // This function performs matrix and vector product
    int m = A.size();
    int n = b.size();
    std::vector<double> c(m, 0);

    for(int i = 0; i < m; i++)
        for(int j = 0; j < n; j++)
        {
            c[i] += A[i][j] * b[j];
        }

    return c;
}

bool MatrixOperations::dot(const std::vector<std::vector<double>> & A,
                           const std::vector<double> & b,
                           std::vector<double> & c)
{
    // This function performs matrix and vector product
    int m = A.size();
    int n = b.size();

    if(c.size() != m || c.size() != n)
    {
        std::cout << red << "ERROR [MatrixOperations::dot]: Wrong output vector size" << reset
                  << std::endl;
        return false;
    }

    for(int i = 0; i < m; i++)
        for(int j = 0; j < n; j++)
        {
            c[i] += A[i][j] * b[j];
        }

    return true;
}

double MatrixOperations::dot(const std::vector<double> & a, const std::vector<double> & b)
{
    // This function performs vector and vector product
    int m = a.size();
    int n = b.size();
    double c = 0;

    if(m != n)
    {
        throw std::domain_error(
            red + std::string("ERROR [MatrixOperations::dot]: Vectors do not have the same size") +
            reset);
    }

    for(int i = 0; i < n; i++)
        c += a[i] * b[i];

    return c;
}

std::vector<double> MatrixOperations::dot(double n, const std::vector<double> & a)
{
    // This function performs scalar product
    int m = a.size();
    std::vector<double> c(m);

    for(int i = 0; i < m; i++)
        c[i] = n * a[i];

    return c;
}

bool MatrixOperations::dot(double n, const std::vector<double> & a, std::vector<double> & c)
{
    // This function performs scalar product
    int m = a.size();

    if(c.size() != m)
    {
        std::cout << red << "ERROR [MatrixOperations::dot]: Wrong output vector size" << reset
                  << std::endl;
        return false;
    }

    for(int i = 0; i < m; i++)
        c[i] = n * a[i];

    return true;
}

std::vector<std::vector<double>> MatrixOperations::getTraslation(
    const std::vector<double> & position)
{
    std::vector<std::vector<double>> T{
        {1, 0, 0, position[0]}, {0, 1, 0, position[1]}, {0, 0, 1, position[2]}, {0, 0, 0, 1}};
    return T;
}

bool MatrixOperations::getTraslation(const std::vector<double> & position,
                                     std::vector<std::vector<double>> & T)
{
    if(T.size() != 4 || T[0].size() != 4)
    {
        std::cout << red << "ERROR [MatrixOperations::getTraslation]: Wrong output matrix size"
                  << reset << std::endl;
        return false;
    }

    T[0][0] = 1;
    T[0][1] = 0;
    T[0][2] = 0;
    T[0][3] = position[0];
    T[1][0] = 0;
    T[1][1] = 1;
    T[1][2] = 0;
    T[1][3] = position[1];
    T[2][0] = 0;
    T[2][1] = 0;
    T[2][2] = 1;
    T[2][3] = position[2];
    T[3][0] = 0;
    T[3][1] = 0;
    T[3][2] = 0;
    T[3][3] = 1;

    return true;
}

std::vector<std::vector<double>> MatrixOperations::getXrot(double angle)
{
    double s = sin(angle);
    double c = cos(angle);

    if(abs(s) < 0.000000001) s = 0;
    if(abs(c) < 0.000000001) c = 0;

    std::vector<std::vector<double>> T{{1, 0, 0, 0}, {0, c, -s, 0}, {0, s, c, 0}, {0, 0, 0, 1}};
    return T;
}

bool MatrixOperations::getXrot(double angle, std::vector<std::vector<double>> & T)
{
    if(T.size() != 4 || T[0].size() != 4)
    {
        std::cout << red << "ERROR [MatrixOperations::getXrot]: Wrong output matrix size" << reset
                  << std::endl;
        return false;
    }

    double s = sin(angle);
    double c = cos(angle);

    if(abs(s) < 0.000000001) s = 0;
    if(abs(c) < 0.000000001) c = 0;

    T[0][0] = 1;
    T[0][1] = 0;
    T[0][2] = 0;
    T[0][3] = 0;
    T[1][0] = 0;
    T[1][1] = c;
    T[1][2] = -s;
    T[1][3] = 0;
    T[2][0] = 0;
    T[2][1] = s;
    T[2][2] = c;
    T[2][3] = 0;
    T[3][0] = 0;
    T[3][1] = 0;
    T[3][2] = 0;
    T[3][3] = 1;

    return true;
}

std::vector<std::vector<double>> MatrixOperations::getYrot(double angle)
{
    double s = sin(angle);
    double c = cos(angle);

    if(abs(s) < 0.000000001) s = 0;
    if(abs(c) < 0.000000001) c = 0;

    std::vector<std::vector<double>> T{{c, 0, s, 0}, {0, 1, 0, 0}, {-s, 0, c, 0}, {0, 0, 0, 1}};
    return T;
}

bool MatrixOperations::getYrot(double angle, std::vector<std::vector<double>> & T)
{
    if(T.size() != 4 || T[0].size() != 4)
    {
        std::cout << red << "ERROR [MatrixOperations::getYrot]: Wrong output matrix size" << reset
                  << std::endl;
        return false;
    }

    double s = sin(angle);
    double c = cos(angle);

    if(abs(s) < 0.000000001) s = 0;
    if(abs(c) < 0.000000001) c = 0;

    T[0][0] = c;
    T[0][1] = 0;
    T[0][2] = s;
    T[0][3] = 0;
    T[1][0] = 0;
    T[1][1] = 1;
    T[1][2] = 0;
    T[1][3] = 0;
    T[2][0] = -s;
    T[2][1] = 0;
    T[2][2] = c;
    T[2][3] = 0;
    T[3][0] = 0;
    T[3][1] = 0;
    T[3][2] = 0;
    T[3][3] = 1;

    return true;
}

std::vector<std::vector<double>> MatrixOperations::getZrot(double angle)
{
    double s = sin(angle);
    double c = cos(angle);

    if(abs(s) < 0.000000001) s = 0;
    if(abs(c) < 0.000000001) c = 0;

    std::vector<std::vector<double>> T{{c, -s, 0, 0}, {s, c, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    return T;
}

bool MatrixOperations::getZrot(double angle, std::vector<std::vector<double>> & T)
{
    if(T.size() != 4 || T[0].size() != 4)
    {
        std::cout << red << "ERROR [MatrixOperations::getZrot]: Wrong output matrix size" << reset
                  << std::endl;
        return false;
    }

    double s = sin(angle);
    double c = cos(angle);

    if(abs(s) < 0.000000001) s = 0;
    if(abs(c) < 0.000000001) c = 0;

    T[0][0] = c;
    T[0][1] = -s;
    T[0][2] = 0;
    T[0][3] = 0;
    T[1][0] = s;
    T[1][1] = c;
    T[1][2] = 0;
    T[1][3] = 0;
    T[2][0] = 0;
    T[2][1] = 0;
    T[2][2] = 1;
    T[2][3] = 0;
    T[3][0] = 0;
    T[3][1] = 0;
    T[3][2] = 0;
    T[3][3] = 1;

    return true;
}

double MatrixOperations::getDeterminant(const std::vector<std::vector<double>> & A)
{
    int m = A.size();

    if(m == 2) { return A[0][0] * A[1][1] - A[1][0] * A[0][1]; }
    else
    {
        double d;
        int c, i, j, subi, subj;
        std::vector<std::vector<double>> subA(m - 1, std::vector<double>(m - 1, 0));

        for(c = 0; c < m; c++)
        {
            if(!getCofactor(A, 0, c, subA))
            {
                throw std::domain_error(red +
                                        std::string("ERROR [MatrixOperations::getDeterminant]: "
                                                    "Failure while extracting the cofactor") +
                                        reset);
            }
            d = d + (pow(-1, c) * A[0][c] * getDeterminant(subA));
        }
        if(abs(d) < 0.0000001) d = 0;
        return d;
    }
}

std::vector<std::vector<double>> MatrixOperations::getCofactor(
    const std::vector<std::vector<double>> & A,
    int row,
    int col)
{
    int m = A.size();
    int subi = 0, subj = 0;
    std::vector<std::vector<double>> subA(m - 1, std::vector<double>(m - 1));
    for(int i = 0; i < m; i++)
    {
        for(int j = 0; j < m; j++)
        {
            if(i != row && j != col)
            {
                subA[subi][subj] = A[i][j];
                subj++;
                if(subj == m - 1)
                {
                    subj = 0;
                    subi++;
                }
            }
        }
    }

    return subA;
}

bool MatrixOperations::getCofactor(const std::vector<std::vector<double>> & A,
                                   int row,
                                   int col,
                                   std::vector<std::vector<double>> & subA)
{
    int m = A.size();

    if(subA.size() != m - 1 || subA[0].size() != m - 1)
    {
        std::cout << red << "ERROR [MatrixOperations::getCofactor]: Wrong output matrix size"
                  << reset << std::endl;
        return false;
    }

    int subi = 0, subj = 0;
    for(int i = 0; i < m; i++)
    {
        for(int j = 0; j < m; j++)
        {
            if(i != row && j != col)
            {
                subA[subi][subj] = A[i][j];
                subj++;
                if(subj == m - 1)
                {
                    subj = 0;
                    subi++;
                }
            }
        }
    }

    return true;
}

std::vector<std::vector<double>> MatrixOperations::getAdjoint(
    const std::vector<std::vector<double>> & A)
{
    int m = A.size();

    if(m == 1) return std::vector<std::vector<double>>(1, std::vector<double>(1, 1));

    int sign = 1;
    std::vector<std::vector<double>> adj(m, std::vector<double>(m));

    for(int i = 0; i < m; i++)
        for(int j = 0; j < m; j++)
        {
            sign = ((i + j) % 2 == 0) ? 1 : -1;
            std::vector<std::vector<double>> cof(m - 1, std::vector<double>(m - 1, 0));
            if(!getCofactor(A, i, j, cof))
            {
                throw std::domain_error(red +
                                        std::string("ERROR [MatrixOperations::getAdjoint]: Failure "
                                                    "while extracting the cofactor") +
                                        reset);
            }
            adj[j][i] = sign * getDeterminant(cof);
        }

    return adj;
}

bool MatrixOperations::getAdjoint(const std::vector<std::vector<double>> & A,
                                  std::vector<std::vector<double>> & adj)
{
    int m = A.size();

    if(adj.size() != m || adj.size() != m)
    {
        std::cout << red << "ERROR [MatrixOperations::getAdjoint]: Wrong output matrix size"
                  << reset << std::endl;
        return false;
    }

    if(m == 1)
    {
        adj[0][0] = 1;
        return true;
    }

    int sign = 1;

    for(int i = 0; i < m; i++)
        for(int j = 0; j < m; j++)
        {
            sign = ((i + j) % 2 == 0) ? 1 : -1;
            std::vector<std::vector<double>> cof(m - 1, std::vector<double>(m - 1, 0));
            if(!getCofactor(A, i, j, cof))
            {
                throw std::domain_error(red +
                                        std::string("ERROR [MatrixOperations::getAdjoint]: Failure "
                                                    "while extracting the cofactor") +
                                        reset);
            }
            adj[j][i] = sign * getDeterminant(cof);
        }

    return true;
}

std::vector<std::vector<double>> MatrixOperations::getInverse(
    const std::vector<std::vector<double>> & A)
{
    double det = getDeterminant(A);
    if(det == 0)
    {
        throw std::domain_error(
            red +
            std::string(
                "ERROR [MatrixOperations::getInverse]: Singular matrix, can't find its inverse") +
            reset);
    }

    int m = A.size();

    std::vector<std::vector<double>> adj(m, std::vector<double>(m, 0));
    if(!getAdjoint(A, adj))
    {
        throw std::domain_error(
            red +
            std::string(
                "ERROR [MatrixOperations::getInverse]: Failure while extracting the adjoint") +
            reset);
    }

    std::vector<std::vector<double>> inverse(m, std::vector<double>(m));

    for(int i = 0; i < m; i++)
        for(int j = 0; j < m; j++)
            inverse[i][j] = adj[i][j] / det;

    return inverse;
}

bool MatrixOperations::getInverse(const std::vector<std::vector<double>> & A,
                                  std::vector<std::vector<double>> & inverse)
{
    double det = getDeterminant(A);
    if(det == 0)
    {
        std::cout << red
                  << "ERROR [MatrixOperations::getInverse]: Singular matrix, can't find its inverse"
                  << reset << std::endl;
        return false;
    }

    int m = A.size();

    if(inverse.size() != m || inverse[0].size() != m)
    {
        std::cout << red << "ERROR [MatrixOperations::getInverse]: Wrong output matrix size"
                  << reset << std::endl;
        return false;
    }

    std::vector<std::vector<double>> adj(m, std::vector<double>(m, 0));
    if(!getAdjoint(A, adj))
    {
        std::cout << red
                  << "ERROR [MatrixOperations::getInverse]: Failure while extracting the adjoint"
                  << reset << std::endl;
        return false;
    }

    for(int i = 0; i < m; i++)
        for(int j = 0; j < m; j++)
            inverse[i][j] = adj[i][j] / det;

    return true;
}

std::vector<double> MatrixOperations::getCrossProduct(const std::vector<double> & a,
                                                      const std::vector<double> & b)
{
    std::vector<double> c(3);

    c[0] = a[1] * b[2] - b[1] * a[2];
    c[1] = b[0] * a[2] - a[0] * b[2];
    c[2] = a[0] * b[1] - b[0] * a[1];

    return c;
}

bool MatrixOperations::getCrossProduct(const std::vector<double> & a,
                                       const std::vector<double> & b,
                                       std::vector<double> & c)
{
    if(c.size() != 3)
    {
        std::cout << red << "ERROR [MatrixOperations::getCrossProduct]: Wrong output vector size"
                  << reset << std::endl;
        return false;
    }

    c[0] = a[1] * b[2] - b[1] * a[2];
    c[1] = b[0] * a[2] - a[0] * b[2];
    c[2] = a[0] * b[1] - b[0] * a[1];

    return true;
}

std::vector<double> MatrixOperations::getSum(const std::vector<double> & a,
                                             const std::vector<double> & b)
{
    std::vector<double> c;
    if(a.size() != b.size())
    {
        throw std::domain_error(
            red + std::string("ERROR [MatrixOperations::getSum]: Vectors sizes don't match") +
            reset);
        return std::vector<double>(1, 0);
    }

    for(int i = 0; i < a.size(); i++)
    {
        c.push_back(a[i] + b[i]);
    }

    return c;
}

bool MatrixOperations::getSum(const std::vector<double> & a,
                              const std::vector<double> & b,
                              std::vector<double> & c)
{
    int m = a.size();

    if(c.size() != m)
    {
        std::cout << red << "ERROR [MatrixOperations::getSum]: Wrong output vector size" << reset
                  << std::endl;
        return false;
    }

    if(b.size() != m)
    {
        std::cout << red << "ERROR [MatrixOperations::getSum]: Vectors sizes don't match" << reset
                  << std::endl;
        return false;
    }

    for(int i = 0; i < a.size(); i++)
    {
        c.push_back(a[i] + b[i]);
    }

    return true;
}

std::vector<double> MatrixOperations::getDifference(const std::vector<double> & a,
                                                    const std::vector<double> & b)
{
    std::vector<double> c;
    if(a.size() != b.size())
    {
        throw std::domain_error(
            red +
            std::string("ERROR [MatrixOperations::getDifference]: Vectors sizes don't match") +
            reset);
        return std::vector<double>(1, 0);
    }

    for(int i = 0; i < a.size(); i++)
    {
        c.push_back(a[i] - b[i]);
    }

    return c;
}

bool MatrixOperations::getDifference(const std::vector<double> & a,
                                     const std::vector<double> & b,
                                     std::vector<double> & c)
{
    int m = a.size();

    if(c.size() != m)
    {
        std::cout << red << "ERROR [MatrixOperations::getDifference]: Wrong output vector size"
                  << reset << std::endl;
        return false;
    }

    if(b.size() != m)
    {
        std::cout << red << "ERROR [MatrixOperations::getDifference]: Vectors sizes don't  match"
                  << reset << std::endl;
        return false;
    }

    for(int i = 0; i < a.size(); i++)
    {
        c.push_back(a[i] - b[i]);
    }

    return true;
}

std::vector<std::vector<double>> MatrixOperations::getDifference(
    const std::vector<std::vector<double>> & a,
    const std::vector<std::vector<double>> & b)
{
    int m = a.size();
    int n = a[0].size();
    if(m != b.size() || n != b[0].size())
    {
        throw std::domain_error(
            red +
            std::string("ERROR [MatrixOperations::getDifference]: Matrixes sizes don't match") +
            reset);
        return std::vector<std::vector<double>>(1, std::vector<double>(1, 0));
    }

    std::vector<std::vector<double>> c(m, std::vector<double>(n, 0));

    for(int i = 0; i < m; i++)
    {
        for(int j = 0; j < n; j++)
        {
            c[i][j] = a[i][j] - b[i][j];
        }
    }

    return c;
}

bool MatrixOperations::getDifference(const std::vector<std::vector<double>> & a,
                                     const std::vector<std::vector<double>> & b,
                                     std::vector<std::vector<double>> & c)
{
    int m = a.size();
    int n = a[0].size();

    if(c.size() != m || c[0].size() != n)
    {
        std::cout << red << "ERROR [MatrixOperations::getDifference]: Wrong output matrix size"
                  << reset << std::endl;
        return false;
    }

    if(m != b.size() || n != b[0].size())
    {
        std::cout << red << "ERROR [MatrixOperations::getDifference]: Matrixes sizes don't match"
                  << reset << std::endl;
        return false;
    }

    for(int i = 0; i < m; i++)
    {
        for(int j = 0; j < n; j++)
        {
            c[i][j] = a[i][j] - b[i][j];
        }
    }

    return true;
}

double MatrixOperations::getNorm(const std::vector<double> & a)
{
    double sum = 0;

    for(int i = 0; i < a.size(); i++)
        sum += pow(a[i], 2);

    return sqrt(sum);
}
