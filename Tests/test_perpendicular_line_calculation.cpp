//
// Created by alechh on 05.04.2022.
//
#include "gtest/gtest.h"
#include "Utils.h"

TEST(PerpendicularLineCalculation, FIRST_TEST)
{
    std::vector<double> coefficientsOfTheLine(3);
    const cv::Point touchPoint(-1, 0);

    coefficientsOfTheLine[0] = 1;
    coefficientsOfTheLine[1] = 1;
    coefficientsOfTheLine[2] = 1;

    std::vector<double> coefficientsOfThePerpendicularLine = Utils::calculateCoefficientsOfThePerpendicularLine(
            coefficientsOfTheLine, touchPoint);

    EXPECT_EQ(coefficientsOfThePerpendicularLine[0], 1);
    EXPECT_EQ(coefficientsOfThePerpendicularLine[1], -1);
    EXPECT_EQ(coefficientsOfThePerpendicularLine[2], 1);
}

TEST(PerpendicularLineCalculation, SECOND_TEST)
{
    std::vector<double> coefficientsOfTheLine(3);
    const cv::Point touchPoint(1, 0);

    coefficientsOfTheLine[0] = -1;
    coefficientsOfTheLine[1] = 1;
    coefficientsOfTheLine[2] = 1;

    std::vector<double> coefficientsOfThePerpendicularLine = Utils::calculateCoefficientsOfThePerpendicularLine(
            coefficientsOfTheLine, touchPoint);

    EXPECT_EQ(coefficientsOfThePerpendicularLine[0], 1);
    EXPECT_EQ(coefficientsOfThePerpendicularLine[1], 1);
    EXPECT_EQ(coefficientsOfThePerpendicularLine[2], -1);
}

TEST(PerpendicularLineCalculation, THIRD_TEST)
{
    std::vector<double> coefficientsOfTheLine(3);
    const cv::Point touchPoint(4, 0);

    coefficientsOfTheLine[0] = 2.5;
    coefficientsOfTheLine[1] = 1;
    coefficientsOfTheLine[2] = -10;

    std::vector<double> coefficientsOfThePerpendicularLine = Utils::calculateCoefficientsOfThePerpendicularLine(
            coefficientsOfTheLine, touchPoint);

    EXPECT_EQ(coefficientsOfThePerpendicularLine[0], 1);
    EXPECT_EQ(coefficientsOfThePerpendicularLine[1], -2.5);
    EXPECT_EQ(coefficientsOfThePerpendicularLine[2], -4);
}

TEST(PerpendicularLineCalculation, FOURTH_TEST)
{
    std::vector<double> coefficientsOfTheLine(3);
    const cv::Point touchPoint(0, 0);

    coefficientsOfTheLine[0] = 100;
    coefficientsOfTheLine[1] = 2;
    coefficientsOfTheLine[2] = 0;

    std::vector<double> coefficientsOfThePerpendicularLine = Utils::calculateCoefficientsOfThePerpendicularLine(
            coefficientsOfTheLine, touchPoint);

    EXPECT_EQ(coefficientsOfThePerpendicularLine[0], 2);
    EXPECT_EQ(coefficientsOfThePerpendicularLine[1], -100);
    EXPECT_EQ(coefficientsOfThePerpendicularLine[2], 0);
}

TEST(PerpendicularLineCalculation, EMPTY_LINE_TEST)
{
    std::vector<double> coefficientsOfTheLine(3);
    const cv::Point touchPoint(0, 0);

    coefficientsOfTheLine[0] = 0;
    coefficientsOfTheLine[1] = 0;
    coefficientsOfTheLine[2] = 0;

    std::vector<double> coefficientsOfThePerpendicularLine = Utils::calculateCoefficientsOfThePerpendicularLine(
            coefficientsOfTheLine, touchPoint);

    EXPECT_EQ(coefficientsOfThePerpendicularLine[0], 0);
    EXPECT_EQ(coefficientsOfThePerpendicularLine[1], 0);
    EXPECT_EQ(coefficientsOfThePerpendicularLine[2], 0);
}

TEST(PerpendicularLineCalculation, VERTICAL_LINE_TEST)
{
    std::vector<double> coefficientsOfTheLine(3);
    const cv::Point touchPoint(0, 0);

    coefficientsOfTheLine[0] = 1;
    coefficientsOfTheLine[1] = 0;
    coefficientsOfTheLine[2] = 0;

    std::vector<double> coefficientsOfThePerpendicularLine = Utils::calculateCoefficientsOfThePerpendicularLine(
            coefficientsOfTheLine, touchPoint);

    EXPECT_EQ(coefficientsOfThePerpendicularLine[0], 0);
    EXPECT_EQ(coefficientsOfThePerpendicularLine[1], -1);
    EXPECT_EQ(coefficientsOfThePerpendicularLine[2], 0);
}

TEST(PerpendicularLineCalculation, HORIZONTAL_LINE_TEST)
{
    std::vector<double> coefficientsOfTheLine(3);
    const cv::Point touchPoint(0, 0);

    coefficientsOfTheLine[0] = 0;
    coefficientsOfTheLine[1] = 2;
    coefficientsOfTheLine[2] = 0;

    std::vector<double> coefficientsOfThePerpendicularLine = Utils::calculateCoefficientsOfThePerpendicularLine(
            coefficientsOfTheLine, touchPoint);

    EXPECT_EQ(coefficientsOfThePerpendicularLine[0], 2);
    EXPECT_EQ(coefficientsOfThePerpendicularLine[1], 0);
    EXPECT_EQ(coefficientsOfThePerpendicularLine[2], 0);
}
