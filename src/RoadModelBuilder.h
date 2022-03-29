//
// Created by alechh on 21.02.2022.
//

#ifndef BUILDING_AND_TRACKING_A_ROAD_MODEL_ROADMODELBUILDER_H
#define BUILDING_AND_TRACKING_A_ROAD_MODEL_ROADMODELBUILDER_H


#include <opencv2/core/mat.hpp>
#include <vector>
#include <opencv2/core/types.hpp>
#include "RoadModel.h"

class RoadModelBuilder {
public:
    static void buildRoadModelBasedOnTheSingleContour(RoadModel &roadModel,
                                                      const std::vector<cv::Point> &contour,
                                                      const std::vector<double> &contourCurvature,
                                                      bool isRightContour);

    static void drawContourPointsDependingOnItsCurvature(cv::Mat &dst, const std::vector<cv::Point> &contour,
                                                         const std::vector<double> &contourCurvature);


    static void
    getPPlusAndPMinus(const std::vector<cv::Point> &segment, cv::Point &pPlus, cv::Point &pMinus, int &pPlusIndex,
                      int &pMinusIndex, cv::Point &centerPoint, double &h);

private:
    static cv::Point calculateCenterOfTheArc(const std::vector<cv::Point> &segment, double R);

    static void addArcToTheModel(const std::vector<cv::Point> &arcSegment, RoadModel &roadModel,
                                 double curvature, bool isRightContour);

    static double calculateAngleShiftUpper(const cv::Point &firstPointOfTheSegment, const cv::Point &circleCenter);

    static double calculateAngleShiftLower(const cv::Point &lastPointOfTheSegment, const cv::Point &circleCenter,
                                           double radiusOfTheCircle);

    static double
    getAngleOfTheArc(const std::vector<cv::Point> &segment, const cv::Point &center, double radiusOfTheCircle);

    static void
    calculationStartAndEndAnglesOfTheArc(double &startAngle, double &endAngle, const std::vector<cv::Point> &segment,
                                         const cv::Point &center, double radiusOfTheCircle);

};


#endif //BUILDING_AND_TRACKING_A_ROAD_MODEL_ROADMODELBUILDER_H
