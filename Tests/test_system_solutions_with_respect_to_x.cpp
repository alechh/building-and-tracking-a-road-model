//
// Created by alechh on 05.04.2022.
//
#include "gtest/gtest.h"
#include "Utils.h"

/**
 * Solution of the equation (x - x0)^2 + (-A/B * x - C/B - y0)^2 = R^2
 */

TEST(SystemSolutionsWithRespectToX, FirstTest)
{
    const int A = 2;
    const int B = 1;
    const int C = 4;
    const int R = 10;
    const cv::Point pt(5, 3);

    std::tuple<double, double>  solutions = Utils::solutionOfTheSystemWithRespectToX(A, B, C, R, pt);

    const double epsilon = 1.e-10;

    EXPECT_EQ(std::get<0>(solutions), - 9.0 / 5 - sqrt(211) / 5);
    EXPECT_LE(std::abs(std::get<1>(solutions) + 9.0 / 5 - sqrt(211) / 5), epsilon);
}


TEST(SystemSolutionsWithRespectToX, SecondTest)
{
    const int A = 5;
    const int B = 5;
    const int C = 5;
    const int R = 3;
    const cv::Point pt(1, 1);

    std::tuple<double, double>  solutions = Utils::solutionOfTheSystemWithRespectToX(A, B, C, R, pt);

    EXPECT_EQ(std::get<0>(solutions), -2);
    EXPECT_EQ(std::get<1>(solutions), 1);
}

TEST(SystemSolutionsWithRespectToX, ThirdTest)
{
    const int A = 0;
    const int B = 2;
    const int C = 0;
    const int R = 0;
    const cv::Point pt(0, 0);

    std::tuple<double, double>  solutions = Utils::solutionOfTheSystemWithRespectToX(A, B, C, R, pt);

    EXPECT_EQ(std::get<0>(solutions), 0);
    EXPECT_EQ(std::get<1>(solutions), 0);
}

TEST(SystemSolutionsWithRespectToX, FourthTest)
{
    const int A = 0;
    const int B = 2;
    const int C = 0;
    const int R = 0;
    const cv::Point pt(10, 0);

    std::tuple<double, double>  solutions = Utils::solutionOfTheSystemWithRespectToX(A, B, C, R, pt);

    EXPECT_EQ(std::get<0>(solutions), 10);
    EXPECT_EQ(std::get<1>(solutions), 10);
}

