//
// Created by alechh on 17.11.2021.
//
#include "gtest/gtest.h"
#include "Utils.cpp"

std::vector<cv::Point> get_circle_contour(const int& R)
{
    std::vector<cv::Point> contour;
    double t = 0;
    double epsilon = 10e-4;
    while (t < 2 * CV_PI)
    {
        contour.emplace_back(500 + R * cos(t),300 + R * sin(t));
        t += epsilon;
        //std::cout << "t = " << t << std::endl;
    }
    return contour;
}


TEST(BasicTest, CurvatureCalculation)
{
    // Curvature of circle = 1/R, where R -- radius of the circle

    Utils ut;

    const int radius = 10000000;

    std::vector<double> circle_curvature;

    std::vector< std::vector<cv::Point> > contours;
    std::vector<cv::Point> circle_contour = get_circle_contour(radius);
    contours.emplace_back(circle_contour);
    //std::cout << "Circle contour size = " << circle_contour.size() << std::endl;

    circle_curvature = ut.calculate_curvature(circle_contour, 1);

    double max_error = 0, min_error = 1;
    int count_inf = 0, count_zero = 0;

    for (auto i : circle_curvature)
    {
        if (i == circle_curvature[0])
        {
            continue;
        }

        //std::cout << "i = " << i << std::endl;

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
        //std::cout << "Error = " << error << std::endl;
    }
    //std::cout <<  "Max Error = " << max_error << std::endl;
    //std::cout << "Min Error = " << min_error << std::endl;
    //std::cout << std::endl << "Number of inf = " << count_inf << std::endl;
    //std::cout << "Number of zero = " << count_zero << std::endl;
    //std::cout << "Number of non inf/zero = " << circle_contour.size() - count_zero - count_inf << std::endl;

    EXPECT_EQ(count_zero, 0);
    EXPECT_EQ(count_inf, 0);
    ASSERT_LE(max_error, 2.19706e-08);
    ASSERT_LE(min_error, 3.6e-14);
}

