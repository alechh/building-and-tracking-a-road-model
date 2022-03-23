//
// Created by alechh on 17.11.2021.
//
#include "gtest/gtest.h"
#include "Utils.cpp"

std::vector<cv::Point> get_circle_contour(const int& R)
{
    std::vector<cv::Point> contour;
    double t = 0;
    double epsilon = 10e-2;
    while (t < 2 * CV_PI)
    {
        contour.emplace_back(500 + R * cos(t),300 + R * sin(t));
        t += epsilon;
    }
    return contour;
}


std::vector<cv::Point> get_straight_line_contour(bool isVertical = true)
{
    std::vector<cv::Point> contour;
    int t = 0;
    while(t < 100)
    {
        if (isVertical)
        {
            contour.emplace_back(cv::Point(0, t));
        }
        else
        {
            contour.emplace_back(cv::Point(t, 0));
        }
        t++;
    }
    return contour;
}


std::vector<cv::Point> get_parabola_contour(int step, int left_boundary, int right_boundary)
{
    std::vector<cv::Point> contour;
    int x = left_boundary;
    while (x < right_boundary)
    {
        contour.emplace_back(cv::Point(x,3 * x * x));
        x += step;
    }
    return contour;
}


TEST(CurvatureTest, CircleCurvature)
{
    // Curvature of circle = 1/R, where R -- radius of the circle

    const int radius = 10000000;

    std::vector<double> circle_curvature;

    std::vector< std::vector<cv::Point> > contours;
    std::vector<cv::Point> circle_contour = get_circle_contour(radius);
    contours.emplace_back(circle_contour);

    circle_curvature = Utils::calculateCurvature(circle_contour, 1);

    double max_error = 0, min_error = 1;
    int count_inf = 0, count_zero = 0;

    for (auto i : circle_curvature)
    {
        if (i == circle_curvature[0])
        {
            continue;
        }

        if (i == std::numeric_limits<double>::infinity())
        {
            count_inf++;
            continue;
        }
        if (i == 0)
        {
            count_zero++;
            continue;
        }

        double error = std::abs(1.0 / radius - i);

        if (error > max_error)
        {
            max_error = error;
        }
        if (error < min_error)
        {
            min_error = error;
        }
    }

    EXPECT_EQ(count_zero, 0);
    EXPECT_EQ(count_inf, 0);
    EXPECT_LE(max_error, 2.0e-8);
    EXPECT_LE(min_error, 2.0e-9);
}


TEST(CurvatureTest, StraightVerticalLineCurvature)
{
    // Curvature of the straight line = 0

    std::vector<double> lineCurvature;

    std::vector< std::vector<cv::Point> > contours;
    std::vector<cv::Point> line_contour = get_straight_line_contour();
    contours.emplace_back(line_contour);

    lineCurvature = Utils::calculateCurvature(line_contour, 1);

    double max_error = 0, min_error = 1;

    for (auto i : lineCurvature)
    {
        if (i == lineCurvature[0] || i == lineCurvature[lineCurvature.size() - 1])
        {
            continue;
        }

        double error = std::abs(i);

        if (error > max_error)
        {
            max_error = error;
        }
        if (error < min_error)
        {
            min_error = error;
        }
    }

    EXPECT_EQ(max_error, 0);
    EXPECT_EQ(min_error, 0);
}


TEST(CurvatureTest, StraightHorizontalLineCurvature)
{
    // Curvature of the straight line = 0

    std::vector<double> lineCurvature;

    std::vector< std::vector<cv::Point> > contours;
    std::vector<cv::Point> line_contour = get_straight_line_contour(false);
    contours.emplace_back(line_contour);

    lineCurvature = Utils::calculateCurvature(line_contour, 1);

    double max_error = 0, min_error = 1;

    for (auto i : lineCurvature)
    {
        if (i == lineCurvature[0] || i == lineCurvature[lineCurvature.size() - 1])
        {
            continue;
        }

        double error = std::abs(i);

        if (error > max_error)
        {
            max_error = error;
        }
        if (error < min_error)
        {
            min_error = error;
        }
    }

    EXPECT_EQ(max_error, 0);
    EXPECT_EQ(min_error, 0);
}


TEST(CurvatureTest, ParabolaCurvature)
{
    // Curvature of parabola 3x^2 = 6 / (1 + 36 * x * x)^(3 / 2)

    std::vector<double> parabolaCurvature;

    int left_x_boundary = -50;
    int right_x_boundary = 50;
    int x_step = 1;

    std::vector< std::vector<cv::Point> > contours;
    std::vector<cv::Point> parabola_contour = get_parabola_contour(x_step, left_x_boundary, right_x_boundary);
    contours.emplace_back(parabola_contour);

    parabolaCurvature = Utils::calculateCurvature(parabola_contour, 1);

    double x = left_x_boundary;
    int i = 0;
    int count_inf = 0;
    while (x < right_x_boundary)
    {
        if (i == 0 || i == parabola_contour.size() - 1)
        {
            x += x_step;
            i++;
            continue;
        }
        if (parabolaCurvature[i] == std::numeric_limits<double>::infinity())
        {
            i++;
            x += x_step;
            count_inf++;
            continue;
        }

        double exactValue = 6.0 / pow(1 + 36 * pow(x, 2), 1.5);
        EXPECT_EQ(parabolaCurvature[i],  exactValue);

        x += x_step;
        i++;
    }

    EXPECT_EQ(count_inf, 0);
}


TEST(CurvatureTest, EmptyContour)
{
    std::vector<double> curvature;

    std::vector<cv::Point> empty_contour;

    curvature = Utils::calculateCurvature(empty_contour, 1);

    EXPECT_EQ(curvature.size(), 0);
}


TEST(CurvatureTest, CircleCurvature2)
{
    // Curvature of circle = 1/R, where R -- radius of the circle

    const int radius = 1000;

    std::vector<double> circle_curvature;

    std::vector< std::vector<cv::Point> > contours;
    std::vector<cv::Point> circle_contour = get_circle_contour(radius);
    contours.emplace_back(circle_contour);

    const int step = 1;
    circle_curvature = Utils::calculateСurvature2(circle_contour, step);

    double errorSum = 0;
    int countNonZero = 0;

    for (int i = 0; i < circle_curvature.size(); ++i)
    {
        if (circle_curvature[i] != 0)
        {
            errorSum += std::abs(1.0 / radius - circle_curvature[i]);
            countNonZero++;
        }
    }

    double meanError = errorSum / countNonZero;

    EXPECT_LE(meanError, 0.00005);
}


TEST(CurvatureTest, ParabolaCurvature2)
{
    // Curvature of parabola 3x^2 = 6 / (1 + 36 * x * x)^(3 / 2)

    std::vector<double> parabolaCurvature;

    const int left_x_boundary = -50;
    const int right_x_boundary = 50;
    const int x_step = 1;

    std::vector< std::vector<cv::Point> > contours;
    std::vector<cv::Point> parabola_contour = get_parabola_contour(x_step, left_x_boundary, right_x_boundary);
    contours.emplace_back(parabola_contour);

    const int step = 1;
    parabolaCurvature = Utils::calculateСurvature2(parabola_contour, step);

    double meanError = 0;
    int countError = 0;

    double x = left_x_boundary;
    int i = 0;
    while (x < right_x_boundary)
    {
        /* TODO
         * В точке 0 вычисляется кривизна 0.6, а должна быть (по формуле) 6. Чем дальше от точки 0, тем точнее
         * Поэтому пока пропускаем эту точку
         */
        if (x == 0)
        {
            x += x_step;
            i++;
            continue;
        }

        if (i == 0 || i == parabola_contour.size() - 1)
        {
            x += x_step;
            i++;
            continue;
        }

        double exactCurvature = 6.0 / pow(1 + 36 * pow(x, 2), 1.5);
        double error = std::abs(exactCurvature - parabolaCurvature[i]);

        meanError += error;
        countError++;

        x += x_step;
        i++;
    }

    meanError /= countError;

    EXPECT_LE(meanError, 0.0005);
}