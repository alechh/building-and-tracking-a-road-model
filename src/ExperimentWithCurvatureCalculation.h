//
// Created by alechh on 21.02.2022.
//

#ifndef BUILDING_AND_TRACKING_A_ROAD_MODEL_EXPERIMENTWITHCURVATURECALCULATION_H
#define BUILDING_AND_TRACKING_A_ROAD_MODEL_EXPERIMENTWITHCURVATURECALCULATION_H


#include <opencv2/core/mat.hpp>

class ExperimentWithCurvatureCalculation {
public:
    static void drawArcsOnContour(cv::Mat &src, const std::vector<cv::Point> &contour, const std::vector<double> &contourCurvature);

private:
    static void drawArc(cv::Mat &src, double radius, cv::Point center, const cv::Scalar &color);
};


#endif //BUILDING_AND_TRACKING_A_ROAD_MODEL_EXPERIMENTWITHCURVATURECALCULATION_H
