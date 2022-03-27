//
// Created by alechh on 26.03.2022.
//

#include "ContourBuilder.h"
#include "Utils.h"

using namespace cv;

/**
 * returns a contour that looks like the picture that is in the folder /contours/Simple_right_contour.jpg
 * Curvature of the arc segment = 0.005 (1 / RADIUS_OF_THE_CIRCLE)
 * Center of the circle = (650, 252)
 * @return
 */
std::vector<cv::Point> ContourBuilder::getSimpleRightContour(const int RADIUS_OF_THE_CIRCLE, const int VERTICAL_LINE_SIZE, const int HORIZONTAL_LINE_SIZE)
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
std::vector<cv::Point> ContourBuilder::getSimpleUpperRightContour(const int RADIUS_OF_THE_CIRCLE, const int VERTICAL_LINE_SIZE, const int HORIZONTAL_LINE_SIZE)
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


/**
 * Returns a contour that looks like the picture that is in the folder /contours/Simple_straight_contour.jpg
 * @return
 */
std::vector<cv::Point> ContourBuilder::getSimpleStraightContour()
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
std::vector<std::vector<cv::Point>> ContourBuilder::getRightContours(const int RADIUS_OF_THE_CIRCLES, const int VERTICAL_LINE_SIZES, const int HORIZONTAL_LINE_SIZES)
{
    std::vector<std::vector<cv::Point>> contours;

    contours.emplace_back(getSimpleUpperRightContour(RADIUS_OF_THE_CIRCLES, VERTICAL_LINE_SIZES, HORIZONTAL_LINE_SIZES));

    contours.emplace_back(getSimpleRightContour(RADIUS_OF_THE_CIRCLES, VERTICAL_LINE_SIZES, HORIZONTAL_LINE_SIZES));

    return contours;
}


/**
 * /**
 * Returns a contour that looks like the picture that is in the folder /contours/Simple_Right_Contour_2.jpg
 * @return
 */
std::vector<cv::Point> ContourBuilder::getSimpleRightContour2(const int RADIUS_OF_THE_CIRCLE, const int VERTICAL_LINE_SIZE)
{
    const int ROWS = 500;
    const int X_COORDINATE_OF_THE_LINE = 400;

    std::vector<cv::Point> contour;

    for (int yCoordinate = ROWS; yCoordinate > ROWS - VERTICAL_LINE_SIZE; --yCoordinate)
    {
        contour.emplace_back(cv::Point(X_COORDINATE_OF_THE_LINE, yCoordinate));
    }

    cv::Point centerOfTheCircle(contour[contour.size() - 1].x, contour[contour.size() - 1].y - RADIUS_OF_THE_CIRCLE);

    const int epsilonDenominator = RADIUS_OF_THE_CIRCLE / 13; // почему 13 не знает никто
    double epsilon = 1.0 / epsilonDenominator;
    double angle = 90;
    while (angle <= 180)
    {
        contour.emplace_back(cv::Point(centerOfTheCircle.x + RADIUS_OF_THE_CIRCLE * cos(angle / 180 * CV_PI), centerOfTheCircle.y + RADIUS_OF_THE_CIRCLE * sin(angle / 180 * CV_PI)));
        angle += epsilon;
    }

    int lastYCoordinateOfCircle = contour[contour.size() - 1].y;

    for (int yCoordinate = lastYCoordinateOfCircle; yCoordinate > lastYCoordinateOfCircle - VERTICAL_LINE_SIZE; --yCoordinate)
    {
        contour.emplace_back(cv::Point(centerOfTheCircle.x - RADIUS_OF_THE_CIRCLE, yCoordinate));
    }

    return contour;
}

std::vector<cv::Point> ContourBuilder::getCircleContour(const int R)
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

std::vector<cv::Point> ContourBuilder::getStraightLineContour(bool isVertical)
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

std::vector<cv::Point> ContourBuilder::getParabolaContour(int step, int leftBoundary, int rightBoundary)
{
    std::vector<cv::Point> contour;
    int x = leftBoundary;
    while (x < rightBoundary)
    {
        contour.emplace_back(cv::Point(x,3 * x * x));
        x += step;
    }
    return contour;
}