//
// Created by alechh on 26.03.2022.
//

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "contourBuilder.h"
#include "Utils.h"


/**
 * returns a contour that looks like the picture that is in the folder /contours/Simple_right_contour.jpg
 * Radius of the circle = 202.002
 * Curvature of the arc segment = 0.0049
 * Center of the circle = (681, 242)
 * @return
 */
std::vector<cv::Point> contourBuilder::getSimpleRightContour()
{
    std::vector<cv::Point> contour;

    for (int yCoordinate = 500; yCoordinate > 240; --yCoordinate)
    {
        contour.emplace_back(cv::Point(479, yCoordinate));
    }

    cv::Point centerOfTheCircle(681, contour[contour.size() - 1].y + 1);

    double radius = Utils::distanceBetweenPoints(centerOfTheCircle, contour[contour.size() - 1]);

    double epsilon = 1.0 / 15;
    double angle = 180;
    while (angle <= 270)
    {
        contour.emplace_back(cv::Point(centerOfTheCircle.x + radius * cos(angle / 180 * CV_PI), centerOfTheCircle.y + radius * sin(angle / 180 * CV_PI)));
        angle += epsilon;
    }

    for (int xCoordinate = 681; xCoordinate < 1000; ++xCoordinate)
    {
        contour.emplace_back(cv::Point(xCoordinate, 40));
    }

    return contour;
}


/**
 * Returns a contour that looks like the picture that is in the folder /contours/Simple_upper_right_contour.jpg
 * Radius of the circle = 202.249
 * Curvature of the arc segment = 0.0049
 * Center of the circle = (681, 200)
 * @return
 */
std::vector<cv::Point> contourBuilder::getSimpleUpperRightContour()
{
    std::vector<cv::Point> contour;

    for (int yCoordinate = 1; yCoordinate < 200; ++yCoordinate)
    {
        contour.emplace_back(cv::Point(479, yCoordinate));
    }

    cv::Point centerOfTheCircle(681, contour[contour.size() - 1].y + 1);

    double radius = Utils::distanceBetweenPoints(centerOfTheCircle, contour[contour.size() - 1]);

    double epsilon = 1.0 / 15;
    double angle = 90;
    while (angle <= 180)
    {
        contour.emplace_back(cv::Point(centerOfTheCircle.x + radius * cos(angle / 180 * CV_PI), (centerOfTheCircle.y + radius * sin(angle / 180 * CV_PI))));
        angle += epsilon;
    }

    for (int xCoordinate = 681; xCoordinate < 1000; ++xCoordinate)
    {
        contour.emplace_back(cv::Point(xCoordinate, 401));
    }

    return contour;
}

void contourBuilder::saveContourOnImage(const std::vector<cv::Point> &contour)
{
    const int ROWS = 500;
    const int COLS = 1000;
    const int TYPE = 16;

    cv::Mat pictuteOfTheContour(ROWS, COLS, TYPE, cv::Scalar(0, 0, 0));
    for (const auto &point : contour)
    {
        cv::circle(pictuteOfTheContour, point, 1, cv::Scalar(255, 0, 0));
    }
    cv::imwrite("../images/contour.jpg", pictuteOfTheContour);
}
