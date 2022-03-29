//
// Created by alechh on 21.02.2022.
//

#ifndef BUILDING_AND_TRACKING_A_ROAD_MODEL_ROADMODELBUILDER_H
#define BUILDING_AND_TRACKING_A_ROAD_MODEL_ROADMODELBUILDER_H


#include <opencv2/core/mat.hpp>
#include "RoadModel.h"

class RoadModelBuilder {
public:
    static void buildRoadModelBasedOnTheSingleContour(RoadModel &roadModel,
                                                      const std::vector<cv::Point> &contour,
                                                      const std::vector<double> &contourCurvature,
                                                      bool isRightContour);
    static void drawContourPointsDependingOnItsCurvature(cv::Mat &dst, const std::vector<cv::Point> &contour, const std::vector<double> &contourCurvature);

private:
    static cv::Point calculateCenterOfTheArc(const std::vector<cv::Point> &segment, double R);
    static void addArcToTheModel(const std::vector<cv::Point> &arcSegment, RoadModel &roadModel,
                                 double curvature, bool isRightContour);
};


#endif //BUILDING_AND_TRACKING_A_ROAD_MODEL_ROADMODELBUILDER_H
