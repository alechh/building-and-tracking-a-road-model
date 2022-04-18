//
// Created by alechh on 06.04.2022.
//

#ifndef TEST_FUNCTIONS_FOR_VKR_ROADMODELTRACKER_H
#define TEST_FUNCTIONS_FOR_VKR_ROADMODELTRACKER_H

#include "RoadModel.h"


class RoadModelTracker
{
private:
    bool isModelConstructed;

    const double arcCenterDelta = 20;
    const double arcRadiusDelta = 10;
    const double lineBeginDelta = 20;
    const double lineEndDelta = 20;

    bool needReplace(const std::shared_ptr<CircularArc> &newArcSegment, const CircularArc &currModelArcSegment) const;

    bool needReplace(const std::shared_ptr<LineSegment> &newLineSegment, const LineSegment &currModelLineSegment) const;


public:
    std::shared_ptr<RoadModel> roadModel;

    explicit RoadModelTracker(std::shared_ptr<RoadModel> roadModel);

    void trackRightSide(const CircularArc &newArcSegment);

    void trackRightSide(const LineSegment &newLineSegment);

    void trackLeftSide(const CircularArc &newArcSegment);

    void trackLeftSide(const LineSegment &newLineSegment);

    void modelHasBeenConstructed();
};


#endif //TEST_FUNCTIONS_FOR_VKR_ROADMODELTRACKER_H
