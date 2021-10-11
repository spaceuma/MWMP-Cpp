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

std::vector<std::vector<double>> MatrixOperations::dot(
    std::vector<std::vector<double>> A,
    std::vector<std::vector<double>> B)
{
    // This function performs matrixes product
    int n = A.size();
    int m = B[0].size();
    std::vector<std::vector<double>> C(n, std::vector<double>(m));

    for (int i = 0; i < n; i++)
        for (int j = 0; j < m; j++)
        {
            C[i][j] = 0;
            for (int k = 0; k < B.size(); k++)
                C[i][j] += A[i][k] * B[k][j];
        }

    return C;
}

std::vector<double> MatrixOperations::dot(std::vector<std::vector<double>> A,
                                            std::vector<double> b)
{
    // This function performs matrix and vector product
    int n = A.size();
    int m = b.size();
    std::vector<double> c(n, 0);

    for (int i = 0; i < n; i++)
        for (int j = 0; j < m; j++)
        {
            c[i] += A[i][j] * b[j];
        }

    return c;
}

double MatrixOperations::dot(std::vector<double> a,
                             std::vector<double> b)
{
    // This function performs vector and vector product
    int n = a.size();
    int m = b.size();
    double c = 0;

    if (n != m)
    {
        throw std::domain_error(red+std::string("ERROR [MatrixOperations::dot]: Vectors do not have the same size")+reset);
    }

    for (int i = 0; i < n; i++)
        c += a[i] * b[i];

    return c;
}

std::vector<double> MatrixOperations::dot(double n, std::vector<double> a)
{
    // This function performs scalar product
    int m = a.size();
    std::vector<double> c(m);

    for (int i = 0; i < m; i++)
        c[i] = n * a[i];

    return c;
}

std::vector<std::vector<double>> MatrixOperations::getTraslation(
    std::vector<double> position)
{
    std::vector<std::vector<double>> T{{1, 0, 0, position[0]},
                                       {0, 1, 0, position[1]},
                                       {0, 0, 1, position[2]},
                                       {0, 0, 0, 1}};
    return T;
}

std::vector<std::vector<double>> MatrixOperations::getXrot(double angle)
{
    double s = sin(angle);
    double c = cos(angle);

    if (abs(s) < 0.000000001) s = 0;
    if (abs(c) < 0.000000001) c = 0;

    std::vector<std::vector<double>> T{
        {1, 0, 0, 0}, {0, c, -s, 0}, {0, s, c, 0}, {0, 0, 0, 1}};
    return T;
}

std::vector<std::vector<double>> MatrixOperations::getYrot(double angle)
{
    double s = sin(angle);
    double c = cos(angle);

    if (abs(s) < 0.000000001) s = 0;
    if (abs(c) < 0.000000001) c = 0;

    std::vector<std::vector<double>> T{
        {c, 0, s, 0}, {0, 1, 0, 0}, {-s, 0, c, 0}, {0, 0, 0, 1}};
    return T;
}

std::vector<std::vector<double>> MatrixOperations::getZrot(double angle)
{
    double s = sin(angle);
    double c = cos(angle);

    if (abs(s) < 0.000000001) s = 0;
    if (abs(c) < 0.000000001) c = 0;

    std::vector<std::vector<double>> T{
        {c, -s, 0, 0}, {s, c, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    return T;
}

double MatrixOperations::getDeterminant(
    const std::vector<std::vector<double>> *A)
{
    int n = A->size();

    if (n == 2)
    {
        return (*A)[0][0] * (*A)[1][1] - (*A)[1][0] * (*A)[0][1];
    }
    else
    {
        double d;
        int c, i, j, subi, subj;
        std::vector<std::vector<double>> subA;

        for (c = 0; c < n; c++)
        {
            subA = getCofactor(A, 0, c);
            d = d + (pow(-1, c) * (*A)[0][c] * getDeterminant(&subA));
        }
        if (abs(d) < 0.0000001) d = 0;
        return d;
    }
}

std::vector<std::vector<double>> MatrixOperations::getCofactor(
    const std::vector<std::vector<double>> *A,
    int row,
    int col)
{
    int n = A->size();
    int subi = 0, subj = 0;
    std::vector<std::vector<double>> subA(n - 1, std::vector<double>(n - 1));
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            if (i != row && j != col)
            {
                subA[subi][subj] = (*A)[i][j];
                subj++;
                if (subj == n - 1)
                {
                    subj = 0;
                    subi++;
                }
            }
        }
    }

    return subA;
}

std::vector<std::vector<double>> MatrixOperations::getAdjoint(
    const std::vector<std::vector<double>> *A)
{
    int n = A->size();

    if (n == 1)
        return std::vector<std::vector<double>>(1, std::vector<double>(1, 1));

    int sign = 1;
    std::vector<std::vector<double>> adj(n, std::vector<double>(n));

    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
        {
            sign = ((i + j) % 2 == 0) ? 1 : -1;
            std::vector<std::vector<double>> cof = getCofactor(A, i, j);
            adj[j][i] = sign * getDeterminant(&cof);
        }

    return adj;
}

std::vector<std::vector<double>> MatrixOperations::getInverse(
    const std::vector<std::vector<double>> *A)
{
    double det = getDeterminant(A);
    if (det == 0)
    {
        throw std::domain_error(red+std::string("ERROR [MatrixOperations::getInverse]: Singular matrix, can't find its inverse")+reset);
    }

    std::vector<std::vector<double>> adj = getAdjoint(A);

    int n = A->size();
    std::vector<std::vector<double>> inverse(n, std::vector<double>(n));

    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            inverse[i][j] = adj[i][j] / det;

    return inverse;
}

std::vector<double> MatrixOperations::getCrossProduct(std::vector<double> a,
                                                      std::vector<double> b)
{
    std::vector<double> c(3);

    c[0] = a[1] * b[2] - b[1] * a[2];
    c[1] = b[0] * a[2] - a[0] * b[2];
    c[2] = a[0] * b[1] - b[0] * a[1];

    return c;
}

std::vector<double> MatrixOperations::getSum(std::vector<double> a,
                                             std::vector<double> b)
{
    std::vector<double> c;
    if (a.size() != b.size())
    {
        throw std::domain_error(red+std::string("ERROR [MatrixOperations::getSum]: Vectors sizes don't match")+reset);
        return std::vector<double>(1, 0);
    }

    for (int i = 0; i < a.size(); i++)
    {
        c.push_back(a[i] + b[i]);
    }

    return c;
}

std::vector<double> MatrixOperations::getDifference(std::vector<double> a,
                                                    std::vector<double> b)
{
    std::vector<double> c;
    if (a.size() != b.size())
    {
        throw std::domain_error(red+std::string("ERROR [MatrixOperations::getDifference]: Vectors sizes don't match")+reset);
        return std::vector<double>(1, 0);
    }

    for (int i = 0; i < a.size(); i++)
    {
        c.push_back(a[i] - b[i]);
    }

    return c;
}

double MatrixOperations::getNorm(std::vector<double> a)
{
    double sum = 0;

    for (int i = 0; i < a.size(); i++)
        sum += pow(a[i], 2);

    return sqrt(sum);
}
