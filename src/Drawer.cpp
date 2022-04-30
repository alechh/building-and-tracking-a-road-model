//
// Created by alechh on 01.04.2022.
//

#include "CurvatureCalculator.h"
#include "ContourBuilder.h"
#include <opencv2/highgui.hpp>
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc.hpp"
#include <vector>
#include "Utils.h"
#include "RoadModel.h"
#include "RoadModelBuilder.h"
#include "Drawer.h"

/**
 * Drawing contour points according to the curvature in them.
 * If the curvature at the point = 0, then the point is drawn in red, if != 0, then blue
 * @param dst -- destination image
 * @param contour -- vector of the points
 * @param contourCurvature -- vector of the curvature of the contour
 */
void Drawer::drawContourPointsDependingOnItsCurvature(cv::Mat &dst, const std::vector<cv::Point> &contour,
                                                      const std::vector<double> &contourCurvature,
                                                      bool addMissingPoints)
{
    if (!addMissingPoints)
    {
        for (int i = 0; i < contour.size(); ++i)
        {
            drawPointDependingOnCurvature(dst, contour[i], contourCurvature[i]);
        }
        return;
    }

    const int DELTA = 2;
    cv::Point prevPoint = contour[0];
    double prevCurvature = contourCurvature[0];
    drawPointDependingOnCurvature(dst, prevPoint, contourCurvature[0]);

    for (int i = 1; i < contour.size(); ++i)
    {
        const cv::Point &currPoint = contour[i];
        cv::Point2f difference = currPoint - prevPoint;

        difference.x = std::abs(difference.x);
        difference.y = std::abs(difference.y);

        if (difference.x > DELTA || difference.y > DELTA)
        {
            const cv::Point directionalVector(currPoint.x - prevPoint.x, currPoint.y - prevPoint.y);

            double t = 0;

            while (std::abs(directionalVector.x * t + prevPoint.x - currPoint.x) > DELTA ||
                   std::abs(directionalVector.y * t + prevPoint.y - currPoint.y) > DELTA)
            {
                t += 0.01; // 0.05 тоже внешне норм

                cv::Point2d newPoint;
                newPoint.x = directionalVector.x * t + prevPoint.x;
                newPoint.y = directionalVector.y * t + prevPoint.y;

                drawPointDependingOnCurvature(dst, newPoint, prevCurvature);
            }
        }
        prevPoint = currPoint;
        prevCurvature = contourCurvature[i];
    }
}

/**
 * Drawing contours on an image
 * @param contours
 * @param dst
 * @param numberOfContours -- number of contours to draw
 */
void Drawer::drawContours(cv::Mat &dst, const std::vector<std::vector<cv::Point>> &contours, int numberOfContours)
{
    if (numberOfContours > contours.size())
    {
        std::cerr << "Utils::drawContours: numberOfContours must be less than contours.size()" << std::endl;
        return;
    }

    std::size_t boundary = (numberOfContours == 0) ? contours.size() : numberOfContours;

    for (std::size_t i = 0; i < boundary; ++i)
    {
        cv::drawContours(dst, contours, i, cv::Scalar(255, 255, 255));
    }
}

void Drawer::drawContoursOnImage(const std::vector<std::vector<cv::Point>> &contours)
{
    const int ROWS = 500;
    const int COLS = 1000;
    const int TYPE = 16;

    cv::Mat pictuteOfTheContour(ROWS, COLS, TYPE, cv::Scalar(255, 255, 255));
    for (const auto &contour: contours)
    {
        for (const auto &point: contour)
        {
            circle(pictuteOfTheContour, point, 1, cv::Scalar(255, 0, 0), 3);
        }
    }

    imwrite("../images/contour.jpg", pictuteOfTheContour);
}

void Drawer::drawContoursPointByPoint(cv::Mat &dts, const std::vector<std::vector<cv::Point>> &contours)
{
    for (const auto &contour: contours)
    {
        srand(time(0));
        int red = rand() % 255;
        int green = rand() % 255;
        int blue = rand() % 255;

        for (const auto &point: contour)
        {
            cv::circle(dts, point, 1, cv::Scalar(blue, green, red));
        }
    }
}

void Drawer::drawContourPointByPoint(cv::Mat &dst, const std::vector<cv::Point> &contour)
{
    for (const auto &point: contour)
    {
        cv::circle(dst, point, 1, cv::Scalar(255, 255, 255));
    }
}

void
Drawer::drawContoursPointsDependingOnItsCurvatures(cv::Mat &dst, const std::vector<std::vector<cv::Point>> &contours,
                                                   const std::vector<std::vector<double>> &contoursCurvatures,
                                                   bool addMissingPoints)
{
    for (int i = 0; i < contours.size(); ++i)
    {
        drawContourPointsDependingOnItsCurvature(dst, contours[i], contoursCurvatures[i], addMissingPoints);
    }
}

void Drawer::drawPointDependingOnCurvature(cv::Mat &dst, const cv::Point &point, double curvature)
{
    const double CURVATURE_THRESHOLD = 0.005;
    if (curvature <= CURVATURE_THRESHOLD)
    {
        circle(dst, point, 1, cv::Scalar(0, 0, 255));
    }
    else
    {
        circle(dst, point, 1, cv::Scalar(255, 0, 0));
    }
}
