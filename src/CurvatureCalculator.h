//
// Created by alechh on 29.03.2022.
//

#ifndef BUILDING_AND_TRACKING_A_ROAD_MODEL_CURVATURECALCULATOR_H
#define BUILDING_AND_TRACKING_A_ROAD_MODEL_CURVATURECALCULATOR_H


#include <vector>
#include <opencv2/core/types.hpp>

class CurvatureCalculator
{
public:

    static void
    calculateCurvature2ForAllContours(std::vector<std::vector<double>> &contoursCurvatures,
                                      const std::vector<std::vector<cv::Point>> &contours, const int step);

    static void
    calculateCurvatureForAllContours(std::vector<std::vector<double>> &contoursCurvatures,
                                      const std::vector<std::vector<cv::Point>> &contours, const int step);

    static void
    calculateCurvature(std::vector<double> &contourCurvature, const std::vector<cv::Point> &vecContourPoints,
                       int step);

    static void
    calculateCurvature2(std::vector<double> &contourCurvature, const std::vector<cv::Point> &contour,
                        const int step);
};


#endif //BUILDING_AND_TRACKING_A_ROAD_MODEL_CURVATURECALCULATOR_H
