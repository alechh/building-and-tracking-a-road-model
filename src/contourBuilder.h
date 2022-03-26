//
// Created by alechh on 26.03.2022.
//

#ifndef BUILDING_AND_TRACKING_A_ROAD_MODEL_CONTOURBUILDER_H
#define BUILDING_AND_TRACKING_A_ROAD_MODEL_CONTOURBUILDER_H

#include <iostream>
#include <opencv2/core/types.hpp>


class contourBuilder {
public:
    static std::vector<cv::Point> getSimpleRightContour();
    static std::vector<cv::Point> getSimpleUpperRightContour();
    static std::vector<cv::Point> getSimpleStraightContour();
    static void saveContoursOnImage(const std::vector<std::vector<cv::Point>> &contours);
    static std::vector<std::vector<cv::Point>> getRightContours();
};


#endif //BUILDING_AND_TRACKING_A_ROAD_MODEL_CONTOURBUILDER_H
