//
// Created by alechh on 21.02.2022.
//

#ifndef BUILDING_AND_TRACKING_A_ROAD_MODEL_ROADMODELBUILDER_H
#define BUILDING_AND_TRACKING_A_ROAD_MODEL_ROADMODELBUILDER_H


#include <opencv2/core/mat.hpp>
#include <vector>
#include <opencv2/core/types.hpp>
#include "RoadModel.h"
#include "RoadModelTracker.h"

class RoadModelBuilder
{
public:
    static void buildRoadModelBasedOnTheSingleContour(RoadModelTracker &modelTracker,
                                                      const std::vector<cv::Point> &contour,
                                                      const std::vector<double> &contourCurvature,
                                                      bool isRightContour);


    static void
    calculatePPlusAndPMinus(const std::vector<cv::Point> &segment, cv::Point &pPlus, cv::Point &pMinus, int &pPlusIndex,
                            int &pMinusIndex, cv::Point &centerPoint, double &h);

    static void buildRoadModel(RoadModelTracker &modelTracker,
                               const std::vector<std::vector<cv::Point>> &contours,
                               const std::vector<std::vector<double>> &contoursCurvatures, int COLS);

private:
    static cv::Point calculateCenterOfTheArc(const std::vector<cv::Point> &segment, double R);

    static void
    addArcToTheModel(RoadModelTracker &modelTracker, std::vector<cv::Point> &arcSegment, double curvature,
                     bool isRightContour);

    static void
    addLineSegmentToModel(RoadModelTracker &modelTracker, std::vector<cv::Point> &lineSegment,
                          bool isRightContour);

    static double calculateAngleShiftUpper(const cv::Point &firstPointOfTheSegment, const cv::Point &circleCenter);

    static double calculateAngleShiftLower(const cv::Point &lastPointOfTheSegment, const cv::Point &circleCenter,
                                           double radiusOfTheCircle);

    static double
    calculateAngleOfTheArc(const std::vector<cv::Point> &segment, const cv::Point &center, double radiusOfTheCircle);

    static void
    calculationStartAndEndAnglesOfTheArc(double &startAngle, double &endAngle, const std::vector<cv::Point> &segment,
                                         const cv::Point &center, double radiusOfTheCircle);

    static void
    addLineSegmentPointsToArcSegment(std::vector<cv::Point> &lineSegment, std::vector<cv::Point> &arcSegment,
                                     double &currSumOfArcSegmentCurvatures);

    static void
    addArcSegmentPointsToLineSegment(std::vector<cv::Point> &arcSegment, std::vector<cv::Point> &lineSegment);

};


#endif //BUILDING_AND_TRACKING_A_ROAD_MODEL_ROADMODELBUILDER_H
