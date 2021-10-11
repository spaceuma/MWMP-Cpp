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

#include "FileManager.hpp"

using namespace FileManager;

void readVectorFile(std::string vector_file, std::vector<double> & vector)
{
    std::ifstream file(vector_file.c_str(), std::ios::in);

    vector.clear();
    if(file.is_open())
    {
        std::string cell;
        while(std::getline(file, cell, ' '))
        {
            double val;
            std::stringstream numeric_value(cell);
            numeric_value >> val;
            vector.push_back(val);
        }
        file.close();
    }
    else
    {
        throw std::domain_error(
            "\033[1;31mERROR [FileManager::readVectorFile]: "
            "Can't open the file, check the path provided \033[0m\n");
    }
}


void readMatrixFile(std::string matrix_file, std::vector<std::vector<double>> & matrix)
{
    std::string line;
    std::ifstream file(matrix_file.c_str(), std::ios::in);

    uint n_row = 0;
    uint n_col = 0;
    matrix.clear();
    std::vector<double> row;
    if(file.is_open())
    {
        while(std::getline(file, line))
        {
            std::stringstream ss(line);
            std::string cell;
            while(std::getline(ss, cell, ' '))
            {
                double val;
                std::stringstream numeric_value(cell);
                numeric_value >> val;
                row.push_back(val);
                n_col++;
            }
            matrix.push_back(row);
            row.clear();
            n_row++;
        }
        file.close();

        n_col /= n_row;
    }
    else
    {
        throw std::domain_error(
            "\033[1;31mERROR [FileManager::readMatrixFile]: "
            "Can't open the file, check the path provided \033[0m\n");
    }
}

void readMatrixFile(std::string matrix_file,
                    std::vector<std::vector<double>> & matrix,
                    char delimiter)
{
    std::string line;
    std::ifstream file(matrix_file.c_str(), std::ios::in);

    double n_row = 0;
    double n_col = 0;
    matrix.clear();
    std::vector<double> row;
    if(file.is_open())
    {
        while(std::getline(file, line))
        {
            std::stringstream ss(line);
            std::string cell;
            while(std::getline(ss, cell, delimiter))
            {
                double val;
                std::stringstream numeric_value(cell);
                numeric_value >> val;
                row.push_back(val);
                n_col++;
            }
            matrix.push_back(row);
            row.clear();
            n_row++;
        }
        file.close();

        n_col /= n_row;
    }
    else
    {
        throw std::domain_error(
            "\033[1;31mERROR [FileManager::readMatrixFile]: "
            "Can't open the file, check the path provided \033[0m\n");
    }
}

void readMatrixFile(std::string matrix_file, std::vector<std::vector<uint>> & matrix)
{
    std::string line;
    std::ifstream file(matrix_file.c_str(), std::ios::in);

    uint n_row = 0;
    uint n_col = 0;
    matrix.clear();
    std::vector<uint> row;
    if(file.is_open())
    {
        while(std::getline(file, line))
        {
            std::stringstream ss(line);
            std::string cell;
            while(std::getline(ss, cell, ' '))
            {
                uint val;
                std::stringstream numeric_value(cell);
                numeric_value >> val;
                row.push_back(val);
                n_col++;
            }
            matrix.push_back(row);
            row.clear();
            n_row++;
        }
        file.close();

        n_col /= n_row;
    }
    else
    {
        throw std::domain_error(
            "\033[1;31mERROR [FileManager::readMatrixFile]: "
            "Can't open the file, check the path provided \033[0m\n");
    }
}


void writeVectorFile(std::string vector_file, std::vector<double> vector)
{
    std::ofstream target_file;
    target_file.open(vector_file);
    for(int i = 0; i < vector.size(); i++)
    {
        target_file << vector[i] << " ";
    }
    target_file.close();
}

void writeVectorFile(std::string vector_file, std::vector<double> * vector)
{
    std::ofstream target_file;
    target_file.open(vector_file);
    for(int i = 0; i < (*vector).size(); i++)
    {
        target_file << (*vector)[i] << " ";
    }
    target_file.close();
}

void writeMatrixFile(std::string matrix_file, std::vector<std::vector<double>> & matrix)
{
    std::ofstream target_file;

    target_file.open(matrix_file);
    for(int j = 0; j < matrix.size(); j++)
    {
        for(int i = 0; i < matrix[0].size(); i++)
        {
            target_file << matrix[j][i] << " ";
        }
        target_file << "\n";
    }
    target_file.close();
}

void writeMatrixFile(std::string matrix_file, std::vector<std::vector<int>> & matrix)
{
    std::ofstream target_file;

    target_file.open(matrix_file);
    for(int j = 0; j < matrix.size(); j++)
    {
        for(int i = 0; i < matrix[0].size(); i++)
        {
            target_file << matrix[j][i] << " ";
        }
        target_file << "\n";
    }
    target_file.close();
}

void writeMatrixFile(std::string matrix_file, std::vector<std::vector<int8_t>> & matrix)
{
    std::ofstream target_file;

    target_file.open(matrix_file);
    for(int j = 0; j < matrix.size(); j++)
    {
        for(int i = 0; i < matrix[0].size(); i++)
        {
            target_file << matrix[j][i] << " ";
        }
        target_file << "\n";
    }
    target_file.close();
}

void writeVolumeFile(std::string volume_file, std::vector<std::vector<std::vector<double>>> * volume)
{
    std::ofstream target_file;
    target_file.open(volume_file);

    target_file << (*volume).size() << " " << (*volume)[0].size() << " "
             << (*volume)[0][0].size() << "\n";
    for(int j = 0; j < volume->size(); j++)
    {
        for(int i = 0; i < (*volume)[0].size(); i++)
        {
            for(int k = 0; k < (*volume)[0][0].size(); k++)
            {
                target_file << (*volume)[j][i][k] << " ";
            }
        }
        target_file << "\n";
    }

    target_file.close();
}