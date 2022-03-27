//
// Created by alechh on 21.02.2022.
//

#ifndef BUILDING_AND_TRACKING_A_ROAD_MODEL_EXPERIMENTWITHCURVATURECALCULATION_H
#define BUILDING_AND_TRACKING_A_ROAD_MODEL_EXPERIMENTWITHCURVATURECALCULATION_H


#include <opencv2/core/mat.hpp>
#include "RoadModel.h"

class ExperimentWithCurvatureCalculation {
public:
    static void drawArcsOnContour(cv::Mat &src, const std::vector<cv::Point> &contour, const std::vector<double> &contourCurvature);
    static RoadModel buildRoadModelBasedOnTheSingleContour(const std::vector<cv::Point> &contour, const std::vector<double> &contourCurvature);
    static void showCurvatureOnImage(cv::Mat &src, const std::vector<cv::Point> &contour, const std::vector<double> &contourCurvature);

private:
    static void drawArc(cv::Mat &src, double radius, cv::Point center, const cv::Scalar &color);
    static cv::Point getCenterOfTheArc(const std::vector<cv::Point> &segment, double R);
    static void addArcToTheModel(const std::vector<cv::Point> &arcSegment, RoadModel &roadModel, double curvature);
};


#endif //BUILDING_AND_TRACKING_A_ROAD_MODEL_EXPERIMENTWITHCURVATURECALCULATION_H
