//
// Created by alechh on 05.04.2022.
//
#include "gtest/gtest.h"
#include "Utils.h"

TEST(TangentLineCalculation, FirstTest)
{
    const cv::Point touchPoint(0, -5);
    const cv::Point firstDerivative(1, 0);

    std::vector<double> tangentLine = Utils::calculateCoefficientsOfTheTangent(touchPoint, firstDerivative);

    EXPECT_EQ(tangentLine[0], 1);
    EXPECT_EQ(tangentLine[1], -1);
    EXPECT_EQ(tangentLine[2], 5);
}

TEST(TangentLineCalculation, SecondTest)
{
    const cv::Point touchPoint(0, 5);
    const cv::Point firstDerivative(-1, 0);

    std::vector<double> tangentLine = Utils::calculateCoefficientsOfTheTangent(touchPoint, firstDerivative);

    EXPECT_EQ(tangentLine[0], -1);
    EXPECT_EQ(tangentLine[1], -1);
    EXPECT_EQ(tangentLine[2], -5);
}

TEST(TangentLineCalculation, ThirdTest)
{
    const cv::Point touchPoint(0, -5);
    const cv::Point firstDerivative(0, 0);

    std::vector<double> tangentLine = Utils::calculateCoefficientsOfTheTangent(touchPoint, firstDerivative);

    EXPECT_EQ(tangentLine[0], 0);
    EXPECT_EQ(tangentLine[1], -1);
    EXPECT_EQ(tangentLine[2], 5);
}

TEST(TangentLineCalculation, FourthTest)
{
    const cv::Point touchPoint(3, -4);
    const cv::Point2f firstDerivative(-3.0 / 4, 0);

    std::vector<double> tangentLine = Utils::calculateCoefficientsOfTheTangent(touchPoint, firstDerivative);

    EXPECT_EQ(tangentLine[0], -0.75);
    EXPECT_EQ(tangentLine[1], -1);
    EXPECT_EQ(tangentLine[2], 6.25);
}