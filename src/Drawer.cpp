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
 * If the curvature at the point = 0, then the point is drawn in blue, if != 0, then red
 * @param dst -- destination image
 * @param contour -- vector of the points
 * @param contourCurvature -- vector of the curvature of the contour
 */
void Drawer::drawContourPointsDependingOnItsCurvature(cv::Mat &dst, const std::vector<cv::Point> &contour, const std::vector<double> &contourCurvature)
{
    for (int i = 0; i < contour.size(); ++i)
    {
        if (contourCurvature[i] == 0)
        {
            circle(dst, contour[i], 1, cv::Scalar(255, 0, 0));
        }
        else
        {
            circle(dst, contour[i], 1, cv::Scalar(0, 0, 255));
        }
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
    for (const auto & contour : contours)
    {
        srand(time(0));
        int red = rand() % 255;
        int green = rand() % 255;
        int blue = rand() % 255;

        for (const auto & point : contour)
        {
            cv::circle(dts, point, 1, cv::Scalar(blue, green, red));
        }
    }
}

void Drawer::drawContourPointByPoint(cv::Mat &dst, const std::vector<cv::Point> &contour)
{
    for (const auto &point : contour)
    {
        cv::circle(dst, point, 1, cv::Scalar(255, 255, 255));
    }
}
