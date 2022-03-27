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

    double epsilon = 1.0 / 15;
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

    double epsilon = 1.0 / 15;
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
std::vector<std::vector<cv::Point>> contourBuilder::getRightContours()
{
    std::vector<std::vector<cv::Point>> contours;

    const int centerXOfTheCircles = 600;

    std::vector<cv::Point> firstContour;

    for (int yCoordinate = 500; yCoordinate > 450; --yCoordinate)// 500 - 240
    {
        firstContour.emplace_back(cv::Point(479, yCoordinate));
    }

    cv::Point centerOfTheCircle(centerXOfTheCircles, firstContour[firstContour.size() - 1].y + 1);

    double radius = Utils::distanceBetweenPoints(centerOfTheCircle, firstContour[firstContour.size() - 1]);

    double epsilon = 1.0 / 15;
    double angle = 180;
    while (angle <= 270)
    {
        firstContour.emplace_back(cv::Point(centerOfTheCircle.x + radius * cos(angle / 180 * CV_PI), centerOfTheCircle.y + radius * sin(angle / 180 * CV_PI)));
        angle += epsilon;
    }

    int lastYOfTheArc = centerOfTheCircle.y + radius * sin(angle / 180 * CV_PI);

    for (int xCoordinate = centerXOfTheCircles; xCoordinate < 1000; ++xCoordinate)
    {
        firstContour.emplace_back(cv::Point(xCoordinate, lastYOfTheArc));
    }

    contours.emplace_back(firstContour);


    std::vector<cv::Point> secondContour;

    for (int yCoordinate = 1; yCoordinate < 100; ++yCoordinate) // 1 -- 200
    {
        secondContour.emplace_back(cv::Point(479, yCoordinate));
    }

    cv::Point centerOfTheCircle2(centerXOfTheCircles, secondContour[secondContour.size() - 1].y + 1);

    double radius2 = Utils::distanceBetweenPoints(centerOfTheCircle2, secondContour[secondContour.size() - 1]);

    double angle2 = 90;
    int firstYOfTheArc2 = centerOfTheCircle2.y + radius2 * sin(angle2 / 180 * CV_PI);

    while (angle2 <= 180)
    {
        secondContour.emplace_back(cv::Point(centerOfTheCircle2.x + radius2 * cos(angle2 / 180 * CV_PI), centerOfTheCircle2.y + radius2 * sin(angle2 / 180 * CV_PI)));
        angle2 += epsilon;
    }

    for (int xCoordinate = centerXOfTheCircles; xCoordinate < 1000; ++xCoordinate)
    {
        secondContour.emplace_back(cv::Point(xCoordinate, firstYOfTheArc2));
    }

    contours.emplace_back(secondContour);

    return contours;
}
