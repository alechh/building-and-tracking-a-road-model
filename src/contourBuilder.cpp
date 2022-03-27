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
 * Curvature of the arc segment = 0.005 (1 / RADIUS_OF_THE_CIRCLE)
 * Center of the circle = (650, 252)
 * @return
 */
std::vector<cv::Point> contourBuilder::getSimpleRightContour(const int RADIUS_OF_THE_CIRCLE, const int VERTICAL_LINE_SIZE, const int HORIZONTAL_LINE_SIZE)
{
    const int ROWS = 500;

    std::vector<cv::Point> contour;

    for (int yCoordinate = ROWS; yCoordinate > ROWS - VERTICAL_LINE_SIZE; --yCoordinate)
    {
        contour.emplace_back(cv::Point(HORIZONTAL_LINE_SIZE + RADIUS_OF_THE_CIRCLE, yCoordinate));
    }

    cv::Point centerOfTheCircle(contour[contour.size() - 1].x + RADIUS_OF_THE_CIRCLE, contour[contour.size() - 1].y + 1);

    const int epsilonDenominator = RADIUS_OF_THE_CIRCLE / 13; // почему 13 не знает никто
    double epsilon = 1.0 / epsilonDenominator;
    double angle = 180;
    while (angle <= 270)
    {
        contour.emplace_back(cv::Point(centerOfTheCircle.x + RADIUS_OF_THE_CIRCLE * cos(angle / 180 * CV_PI), centerOfTheCircle.y + RADIUS_OF_THE_CIRCLE * sin(angle / 180 * CV_PI)));
        angle += epsilon;
    }

    int lastYCoordinateOfCircle = centerOfTheCircle.y + RADIUS_OF_THE_CIRCLE * sin(angle / 180 * CV_PI);

    for (int xCoordinate = centerOfTheCircle.x; xCoordinate < HORIZONTAL_LINE_SIZE + centerOfTheCircle.x; ++xCoordinate)
    {
        contour.emplace_back(cv::Point(xCoordinate, lastYCoordinateOfCircle));
    }

    return contour;
}


/**
 * Returns a contour that looks like the picture that is in the folder /contours/Simple_upper_right_contour.jpg
 * Curvature of the arc segment = 0.005 (1 / RADIUS_OF_THE_CIRCLE)
 * Center of the circle = (650, 250)
 * @return
 */
std::vector<cv::Point> contourBuilder::getSimpleUpperRightContour(const int RADIUS_OF_THE_CIRCLE, const int VERTICAL_LINE_SIZE, const int HORIZONTAL_LINE_SIZE)
{
    std::vector<cv::Point> contour;

    for (int yCoordinate = 1; yCoordinate < VERTICAL_LINE_SIZE; ++yCoordinate)
    {
        contour.emplace_back(cv::Point(HORIZONTAL_LINE_SIZE + RADIUS_OF_THE_CIRCLE, yCoordinate));
    }

    cv::Point centerOfTheCircle(contour[contour.size() - 1].x + RADIUS_OF_THE_CIRCLE, contour[contour.size() - 1].y + 1);

    const int epsilonDenominator = RADIUS_OF_THE_CIRCLE / 13; // почему 13 не знает никто
    double epsilon = 1.0 / epsilonDenominator;
    double angle = 180;
    while (angle >= 90)
    {
        contour.emplace_back(cv::Point(centerOfTheCircle.x + RADIUS_OF_THE_CIRCLE * cos(angle / 180 * CV_PI), centerOfTheCircle.y + RADIUS_OF_THE_CIRCLE * sin(angle / 180 * CV_PI)));
        angle -= epsilon;
    }

    // -1 чтобы контуры хорошо склеивались
    double lastYOfTheCircle = contour[contour.size() - 1].y - 1;

    for (int xCoordinate = centerOfTheCircle.x; xCoordinate < centerOfTheCircle.x + HORIZONTAL_LINE_SIZE; ++xCoordinate)
    {
        contour.emplace_back(cv::Point(xCoordinate, lastYOfTheCircle));
    }

    return contour;
}

void contourBuilder::saveContoursOnImage(const std::vector<std::vector<cv::Point>> &contours)
{
    const int ROWS = 500;
    const int COLS = 1000;
    const int TYPE = 16;

    cv::Mat pictuteOfTheContour(ROWS, COLS, TYPE, cv::Scalar(0, 0, 0));
    for (const auto &contour : contours)
    {
        for (const auto &point : contour)
        {
            cv::circle(pictuteOfTheContour, point, 1, cv::Scalar(255, 0, 0));
        }
    }

    cv::imwrite("../images/contour.jpg", pictuteOfTheContour);
}


/**
 * Returns a contour that looks like the picture that is in the folder /contours/Simple_straight_contour.jpg
 * @return
 */
std::vector<cv::Point> contourBuilder::getSimpleStraightContour()
{
    const int VERTICAL_LINE_SIZE = 450;
    const int X_COORDINATE_OF_THE_LINE = 450;

    std::vector<cv::Point> contour;

    for (int yCoordinate = 1; yCoordinate < VERTICAL_LINE_SIZE; ++yCoordinate)
    {
        contour.emplace_back(cv::Point(X_COORDINATE_OF_THE_LINE, yCoordinate));
    }

    return contour;
}


/**
 * Returns a contour that looks like the picture that is in the folder /contours/Right_contour.jpg
 * @return
 */
std::vector<std::vector<cv::Point>> contourBuilder::getRightContours(const int RADIUS_OF_THE_CIRCLES, const int VERTICAL_LINE_SIZES, const int HORIZONTAL_LINE_SIZES)
{
    std::vector<std::vector<cv::Point>> contours;

    contours.emplace_back(getSimpleUpperRightContour(RADIUS_OF_THE_CIRCLES, VERTICAL_LINE_SIZES, HORIZONTAL_LINE_SIZES));

    std::cout << contours[0].size() << std::endl;

    contours.emplace_back(getSimpleRightContour(RADIUS_OF_THE_CIRCLES, VERTICAL_LINE_SIZES, HORIZONTAL_LINE_SIZES));

    std::cout << contours[1].size() << std::endl;

    return contours;
}
