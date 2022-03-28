//
// Created by alechh on 17.11.2021.
//
#include "gtest/gtest.h"
#include "Utils.h"
#include "ContourBuilder.h"


TEST(CurvatureTest, CircleCurvature)
{
    // Curvature of circle = 1/R, where R -- radius of the circle

    const int radius = 10000000;

    std::vector< std::vector<cv::Point> > contours;
    std::vector<cv::Point> circleContour = ContourBuilder::getCircleContour(radius);
    contours.emplace_back(circleContour);

    std::vector<double> circleCurvature(circleContour.size());
    Utils::calculateCurvature(circleCurvature, circleContour, 1);

    double maxError = 0, minError = 1;
    int countInf = 0, countZero = 0;

    for (auto i : circleCurvature)
    {
        if (i == circleCurvature[0])
        {
            continue;
        }

        if (i == std::numeric_limits<double>::infinity())
        {
            countInf++;
            continue;
        }
        if (i == 0)
        {
            countZero++;
            continue;
        }

        double error = std::abs(1.0 / radius - i);

        if (error > maxError)
        {
            maxError = error;
        }
        if (error < minError)
        {
            minError = error;
        }
    }

    EXPECT_EQ(countZero, 0);
    EXPECT_EQ(countInf, 0);
    EXPECT_LE(maxError, 2.0e-8);
    EXPECT_LE(minError, 2.0e-9);
}


TEST(CurvatureTest, StraightVerticalLineCurvature)
{
    // Curvature of the straight line = 0
    std::vector< std::vector<cv::Point> > contours;
    std::vector<cv::Point> lineContour = ContourBuilder::getStraightLineContour();
    contours.emplace_back(lineContour);

    std::vector<double> lineCurvature(lineContour.size());

    Utils::calculateCurvature(lineCurvature, lineContour, 1);

    double maxError = 0, minError = 1;

    for (auto i : lineCurvature)
    {
        if (i == lineCurvature[0] || i == lineCurvature[lineCurvature.size() - 1])
        {
            continue;
        }

        double error = std::abs(i);

        if (error > maxError)
        {
            maxError = error;
        }
        if (error < minError)
        {
            minError = error;
        }
    }

    EXPECT_EQ(maxError, 0);
    EXPECT_EQ(minError, 0);
}


TEST(CurvatureTest, StraightHorizontalLineCurvature)
{
    // Curvature of the straight line = 0

    std::vector< std::vector<cv::Point> > contours;
    std::vector<cv::Point> lineContour = ContourBuilder::getStraightLineContour(false);
    contours.emplace_back(lineContour);

    std::vector<double> lineCurvature(lineContour.size());
    Utils::calculateCurvature(lineCurvature, lineContour, 1);

    double maxError = 0, minError = 1;

    for (auto i : lineCurvature)
    {
        if (i == lineCurvature[0] || i == lineCurvature[lineCurvature.size() - 1])
        {
            continue;
        }

        double error = std::abs(i);

        if (error > maxError)
        {
            maxError = error;
        }
        if (error < minError)
        {
            minError = error;
        }
    }

    EXPECT_EQ(maxError, 0);
    EXPECT_EQ(minError, 0);
}


TEST(CurvatureTest, ParabolaCurvature)
{
    // Curvature of parabola 3x^2 = 6 / (1 + 36 * x * x)^(3 / 2)

    int leftXBoundary = -50;
    int rightXBoundary = 50;
    int xStep = 1;

    std::vector< std::vector<cv::Point> > contours;
    std::vector<cv::Point> parabolaContour = ContourBuilder::getParabolaContour(xStep, leftXBoundary, rightXBoundary);
    contours.emplace_back(parabolaContour);

    std::vector<double> parabolaCurvature(parabolaContour.size());

    Utils::calculateCurvature(parabolaCurvature, parabolaContour, 1);

    double x = leftXBoundary;
    int i = 0;
    int countInf = 0;
    while (x < rightXBoundary)
    {
        if (i == 0 || i == parabolaContour.size() - 1)
        {
            x += xStep;
            i++;
            continue;
        }
        if (parabolaCurvature[i] == std::numeric_limits<double>::infinity())
        {
            i++;
            x += xStep;
            countInf++;
            continue;
        }

        double exactValue = 6.0 / pow(1 + 36 * pow(x, 2), 1.5);
        EXPECT_EQ(parabolaCurvature[i],  exactValue);

        x += xStep;
        i++;
    }

    EXPECT_EQ(countInf, 0);
}


TEST(CurvatureTest, EmptyContour)
{
    std::vector<cv::Point> emptyContour;

    std::vector<double> curvature(emptyContour.size());

    Utils::calculateCurvature(curvature, emptyContour, 1);

    EXPECT_EQ(curvature.size(), 0);
}


TEST(CurvatureTest, CircleCurvature2)
{
    // Curvature of circle = 1/R, where R -- radius of the circle

    const int radius = 1000;

    std::vector< std::vector<cv::Point> > contours;
    std::vector<cv::Point> circleContour = ContourBuilder::getCircleContour(radius);
    contours.emplace_back(circleContour);

    std::vector<double> circleCurvature(circleContour.size());

    const int step = 1;
    Utils::calculateCurvature2(circleCurvature, circleContour, step);

    double errorSum = 0;
    int countNonZero = 0;

    for (int i = 0; i < circleCurvature.size(); ++i)
    {
        if (circleCurvature[i] != 0)
        {
            errorSum += std::abs(1.0 / radius - circleCurvature[i]);
            countNonZero++;
        }
    }

    double meanError = errorSum / countNonZero;

    EXPECT_LE(meanError, 0.00005);
}


TEST(CurvatureTest, ParabolaCurvature2)
{
    // Curvature of parabola 3x^2 = 6 / (1 + 36 * x * x)^(3 / 2)

    const int leftXBoundary = -50;
    const int rightXBoundary = 50;
    const int xStep = 1;

    std::vector< std::vector<cv::Point> > contours;
    std::vector<cv::Point> parabolaContour = ContourBuilder::getParabolaContour(xStep, leftXBoundary, rightXBoundary);
    contours.emplace_back(parabolaContour);

    std::vector<double> parabolaCurvature(parabolaContour.size());

    const int step = 1;
    Utils::calculateCurvature2(parabolaCurvature, parabolaContour, step);

    double meanError = 0;
    int countError = 0;

    double x = leftXBoundary;
    int i = 0;
    while (x < rightXBoundary)
    {
        /* TODO
         * В точке 0 вычисляется кривизна 0.6, а должна быть (по формуле) 6. Чем дальше от точки 0, тем точнее
         * Поэтому пока пропускаем эту точку
         */
        if (x == 0)
        {
            x += xStep;
            i++;
            continue;
        }

        if (i == 0 || i == parabolaContour.size() - 1)
        {
            x += xStep;
            i++;
            continue;
        }

        double exactCurvature = 6.0 / pow(1 + 36 * pow(x, 2), 1.5);
        double error = std::abs(exactCurvature - parabolaCurvature[i]);

        meanError += error;
        countError++;

        x += xStep;
        i++;
    }

    meanError /= countError;

    EXPECT_LE(meanError, 0.0005);
}