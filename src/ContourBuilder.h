//
// Created by alechh on 26.03.2022.
//

#ifndef BUILDING_AND_TRACKING_A_ROAD_MODEL_CONTOURBUILDER_H
#define BUILDING_AND_TRACKING_A_ROAD_MODEL_CONTOURBUILDER_H

#include <iostream>
#include <opencv2/core/types.hpp>


class ContourBuilder
{
public:
    static std::vector<cv::Point>
    getSimpleRightContour(int RADIUS_OF_THE_CIRCLE = 200, int VERTICAL_LINE_SIZE = 250, int HORIZONTAL_LINE_SIZE = 250,
                          int X_COORDINATE_OF_THE_STRAIGHT_LINE = 600, int t = 0);

    static std::vector<cv::Point>
    getSimpleUpperRightContour(int RADIUS_OF_THE_CIRCLE = 200, int VERTICAL_LINE_SIZE = 250,
                               int HORIZONTAL_LINE_SIZE = 250,
                               int X_COORDINATE_OF_THE_STRAIGHT_LINE = 600, int t = 0);

    static std::vector<cv::Point>
    getSimpleStraightContour(int X_COORDINATE_OF_THE_LINE = 450, int VERTICAL_LINE_SIZE = 500, int t = 0);

    static std::vector<cv::Point> getSimpleRightContour2(int RADIUS_OF_THE_CIRCLE = 100, int VERTICAL_LINE_SIZE = 200);

    static std::vector<std::vector<cv::Point>>
    getRightContours(int RADIUS_OF_THE_CIRCLES = 110, int VERTICAL_LINE_SIZES = 100, int HORIZONTAL_LINE_SIZES = 300,
                     int X_COORDINATE_OF_THE_STRAIGHT_LINE = 400, int t = 0);

    static std::vector<std::vector<cv::Point>>
    getRightAndLeftContours(int RADIUS_OF_THE_CIRCLES = 80, int VERTICAL_LINE_SIZES = 100,
                            int HORIZONTAL_LINE_SIZES = 150,
                            int X_COORDINATE_OF_THE_STRAIGHT_LINE = 300, int t = 0);

    static std::vector<cv::Point> getCircleContour(int R);

    static std::vector<cv::Point> getStraightLineContour(bool isVertical = true);

    static std::vector<cv::Point> getParabolaContour(int step, int leftBoundary, int rightBoundary);

    static std::vector<std::vector<cv::Point>>
    getContoursWithT(int RADIUS_OF_THE_CIRCLES = 110, int VERTICAL_LINE_SIZES = 100, int HORIZONTAL_LINE_SIZES = 300,
                     int X_COORDINATE_OF_THE_STRAIGHT_LINE = 400, int t = 0);
};


#endif //BUILDING_AND_TRACKING_A_ROAD_MODEL_CONTOURBUILDER_H
