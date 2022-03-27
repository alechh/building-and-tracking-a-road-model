//
// Created by alechh on 26.03.2022.
//

#ifndef BUILDING_AND_TRACKING_A_ROAD_MODEL_CONTOURBUILDER_H
#define BUILDING_AND_TRACKING_A_ROAD_MODEL_CONTOURBUILDER_H

#include <iostream>
#include <opencv2/core/types.hpp>


class contourBuilder {
public:
    static std::vector<cv::Point> getSimpleRightContour(int RADIUS_OF_THE_CIRCLE = 200, int VERTICAL_LINE_SIZE = 250, int HORIZONTAL_LINE_SIZE = 250);
    static std::vector<cv::Point> getSimpleUpperRightContour(int RADIUS_OF_THE_CIRCLE = 200, int VERTICAL_LINE_SIZE = 250, int HORIZONTAL_LINE_SIZE = 250);
    static std::vector<cv::Point> getSimpleStraightContour();
    static void saveContoursOnImage(const std::vector<std::vector<cv::Point>> &contours);
    static std::vector<std::vector<cv::Point>> getRightContours(int RADIUS_OF_THE_CIRCLES = 110, int VERTICAL_LINE_SIZES = 100, int HORIZONTAL_LINE_SIZES = 300);
};


#endif //BUILDING_AND_TRACKING_A_ROAD_MODEL_CONTOURBUILDER_H
