#include "FileManager.hpp"
#include "MatrixOperations.hpp"
#include <ctime>
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

using namespace MatrixOperations;

TEST(MatrixOperationsTest, scalar_dot)
{
    std::vector<std::vector<double>> A(10, std::vector<double>(10,0));
    std::vector<std::vector<double>> B(10, std::vector<double>(10,0));
    std::vector<std::vector<double>> C(10, std::vector<double>(10,0));

    for(uint i = 0; i < 10; i++)
        for(uint j = 0; j < 10; j++)
        {
            A[i][j] = i+j;
            B[i][j] = 2*(i+j);
        }

    clock_t ini_time = clock();
    EXPECT_EQ(dot(2,A), B);
    std::cout <<cyan<<"[MatrixOperationsTest::scalar_dot] Standard elapsed time: "<< (double)(clock() - ini_time) / CLOCKS_PER_SEC << " s" <<nocolor<<std::endl;

    ini_time = clock();
    dot(2,A,C);
    EXPECT_EQ(C, B);
    std::cout <<cyan<<"[MatrixOperationsTest::scalar_dot] Passed-by-reference elapsed time: "<< (double)(clock() - ini_time) / CLOCKS_PER_SEC << " s" <<nocolor<< std::endl;
}

TEST(MatrixOperationsTest, matrixes_dot)
{
    std::vector<std::vector<double>> A(10, std::vector<double>(10,0));
    std::vector<std::vector<double>> B(10, std::vector<double>(10,0));
    std::vector<std::vector<double>> C(10, std::vector<double>(10,0));

    for(uint i = 0; i < 10; i++)
        for(uint j = 0; j < 10; j++)
        {
            A[i][j] = i+j;
        }

    FileManager::readMatrixFile("results/matrixes_dot.txt", B);

    clock_t ini_time = clock();
    EXPECT_EQ(dot(A,A), B);
    std::cout <<cyan<<"[MatrixOperationsTest::matrixes_dot] Standard elapsed time: "<< (double)(clock() - ini_time) / CLOCKS_PER_SEC << " s" <<nocolor<< std::endl;

    ini_time = clock();
    dot(A,A,C);
    EXPECT_EQ(C, B);
    std::cout <<cyan<<"[MatrixOperationsTest::matrixes_dot] Passed-by-reference elapsed time: "<< (double)(clock() - ini_time) / CLOCKS_PER_SEC << " s" <<nocolor<< std::endl;
}

TEST(MatrixOperationsTest, matrix_vector_dot)
{
    std::vector<std::vector<double>> A(10, std::vector<double>(10,0));
    std::vector<double> b = {5, 64, 2, 3, 4, 6, 789, 1, 654, 3};
    std::vector<double> c(10, 0);
    std::vector<double> d(10, 0);

    for(uint i = 0; i < 10; i++)
        for(uint j = 0; j < 10; j++)
        {
            A[i][j] = i+j;
        }

    FileManager::readVectorFile("results/matrix_vector_dot.txt", c);

    clock_t ini_time = clock();
    EXPECT_EQ(dot(A,b), c);
    std::cout<<cyan <<"[MatrixOperationsTest::matrix_vector_dot] Standard elapsed time: "<< (double)(clock() - ini_time) / CLOCKS_PER_SEC << " s" << nocolor<<std::endl;

    ini_time = clock();
    dot(A,b,d);
    EXPECT_EQ(d, c);
    std::cout<<cyan <<"[MatrixOperationsTest::matrix_vector_dot] Passed-by-reference elapsed time: "<< (double)(clock() - ini_time) / CLOCKS_PER_SEC << " s" << nocolor<<std::endl;
}

TEST(MatrixOperationsTest, matrix_determinant)
{
    std::vector<std::vector<double>> A;
    double c, d;

    FileManager::readMatrixFile("inputs/matrix_determinant.txt", A);
    d = 1;

    clock_t ini_time = clock();
    EXPECT_EQ(getDeterminant(A), d);
    std::cout<<cyan <<"[MatrixOperationsTest::matrix_determinant] Elapsed time: "<< (double)(clock() - ini_time) / CLOCKS_PER_SEC << " s" << nocolor<<std::endl;
}

TEST(MatrixOperationsTest, matrix_inverse)
{
    std::vector<std::vector<double>> A(10, std::vector<double>(10,0));
    std::vector<std::vector<double>> B(10, std::vector<double>(10,0));
    std::vector<std::vector<double>> C(10, std::vector<double>(10,0));

    FileManager::readMatrixFile("inputs/matrix_inverse.txt", A);
    FileManager::readMatrixFile("results/matrix_inverse.txt", C);

    clock_t ini_time = clock();
    B = getInverse(A);
    std::cout<<cyan <<"[MatrixOperationsTest::matrix_inverse] Standard elapsed time: "<< (double)(clock() - ini_time) / CLOCKS_PER_SEC << " s" << nocolor<<std::endl;
    for(uint i = 0; i < 10; i++)
        for(uint j = 0; j < 10; j++)
        {
            B[i][j] = (int)(B[i][j]*10000+.50001);
            B[i][j] /= 10000;
            C[i][j] = (int)(C[i][j]*10000+.50001);
            C[i][j] /= 10000;
        }
    EXPECT_EQ(B, C);

    FileManager::readMatrixFile("results/matrix_inverse.txt", C);

    ini_time = clock();
    getInverse(A,B);
    std::cout<<cyan <<"[MatrixOperationsTest::matrix_inverse] Passed-by-reference elapsed time: "<< (double)(clock() - ini_time) / CLOCKS_PER_SEC << " s" << nocolor<<std::endl;
    for(uint i = 0; i < 10; i++)
        for(uint j = 0; j < 10; j++)
        {
            B[i][j] = (int)(B[i][j]*10000+.50001);
            B[i][j] /= 10000;
            C[i][j] = (int)(C[i][j]*10000+.50001);
            C[i][j] /= 10000;
        }
    EXPECT_EQ(B, C);
}
