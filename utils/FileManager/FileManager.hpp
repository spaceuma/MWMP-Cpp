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
// Authors: Gonzalo J. Paz Delgado, J. Ricardo Sánchez Ibáñez
// Supervisor: Carlos J. Pérez del Pulgar
// Affiliation: Department of Systems Engineering and Automation
// Space Robotics Lab (www.uma.es/space-robotics)

#ifndef __MM_FILE_MANAGER__
#define __MM_FILE_MANAGER__

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace FileManager
{
void readVectorFile(std::string vector_file, std::vector<double> & vector);

void readVectorFile(std::string vector_file, Eigen::VectorXd & vector);

void readMatrixFile(std::string matrix_file, std::vector<std::vector<double>> & matrix);

void readMatrixFile(std::string matrix_file,
                    std::vector<std::vector<double>> & matrix,
                    char delimiter);

void readMatrixFile(std::string matrix_file, Eigen::MatrixXd & matrix);

void readMatrixFile(std::string matrix_file, Eigen::MatrixXd & matrix, char delimiter);

void readMatrixFile(std::string matrix_file, std::vector<std::vector<uint>> & matrix);

void writeVectorFile(std::string vector_file, std::vector<double> vector);

void writeVectorFile(std::string vector_file, std::vector<double> * vector);

void writeVectorFile(std::string vector_file, const Eigen::VectorXd & vector);

void writeMatrixFile(std::string matrix_file, std::vector<std::vector<double>> & matrix);

void writeMatrixFile(std::string matrix_file, std::vector<Eigen::VectorXd> & matrix);

void writeMatrixFile(std::string matrix_file, const Eigen::MatrixXd & matrix);

void writeMatrixFile(std::string matrix_file, std::vector<std::vector<int>> & matrix);

void writeMatrixFile(std::string matrix_file, std::vector<std::vector<int8_t>> & matrix);

void writeMatrixFile(std::string matrix_file, std::vector<std::vector<uint>> & matrix);

void writeVolumeFile(std::string volume_file,
                     std::vector<std::vector<std::vector<double>>> * volume);

void writeVolumeFile(std::string volume_file, const std::vector<Eigen::MatrixXd> & volume);
}    // namespace FileManager
#endif
