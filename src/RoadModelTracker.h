//
// Created by alechh on 06.04.2022.
//

#ifndef TEST_FUNCTIONS_FOR_VKR_ROADMODELTRACKER_H
#define TEST_FUNCTIONS_FOR_VKR_ROADMODELTRACKER_H

#include "RoadModel.h"


class RoadModelTracker
{
private:
    RoadModel roadModel;
    const double arcCenterDelta = 0.5;
    const double arcRadiusDelta = 0.5;
    const double lineBeginDelta = 0.5;
    const double lineEndDelta = 0.5;

    bool needReplace(const CircularArc &newArcSegment, const LineSegment &currModelLineSegment) const;
    bool needReplace(const CircularArc &newArcSegment, const CircularArc &currModelArcSegment) const;
    bool needReplace(const LineSegment &newLineSegment, const LineSegment &currModelLineSegment) const;
    bool needReplace(const LineSegment &newLineSegment, const CircularArc &currModelArcSegment) const;

public:
    explicit RoadModelTracker(RoadModel roadModel);
    void track(const CircularArc &arcSegment);
};


#endif //TEST_FUNCTIONS_FOR_VKR_ROADMODELTRACKER_H
