//
// Created by alechh on 01.04.2022.
//

#ifndef BUILDING_AND_TRACKING_A_ROAD_MODEL_DRAWER_H
#define BUILDING_AND_TRACKING_A_ROAD_MODEL_DRAWER_H


#include <vector>
#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>
#include "RoadModel.h"

class Drawer
{

public:
    static void drawContourPointsDependingOnItsCurvature(cv::Mat &dst, const std::vector<cv::Point> &contour,
                                                         const std::vector<double> &contourCurvature);

    static void
    drawContours(cv::Mat &dst, const std::vector<std::vector<cv::Point>> &contours, int numberOfContours = 0);

    static void drawContoursOnImage(const std::vector<std::vector<cv::Point>> &contours);

    static void drawContoursPointByPoint(cv::Mat &dts, const std::vector<std::vector<cv::Point>> &contours);
};


#endif //BUILDING_AND_TRACKING_A_ROAD_MODEL_DRAWER_H
