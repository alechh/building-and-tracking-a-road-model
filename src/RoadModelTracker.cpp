//
// Created by alechh on 06.04.2022.
//

#include "RoadModelTracker.h"

#include <utility>

RoadModelTracker::RoadModelTracker(RoadModel roadModel) : roadModel(std::move(roadModel))
{}

void RoadModelTracker::trackRightSide(const CircularArc &newArcSegment)
{
    std::shared_ptr<ModelElement> currModelElement(this->roadModel.getRightHead());

    bool replaced = false;

    while (currModelElement)
    {
        if (needReplace(newArcSegment, *currModelElement))
        {
            this->roadModel.replaceModelRightElement(newArcSegment, *currModelElement);
            replaced = true;
            break;
        }

        currModelElement = currModelElement->next;
    }

    if (!replaced)
    {
        this->roadModel.addElementToRight(newArcSegment);
    }
}

bool RoadModelTracker::needReplace(const CircularArc &newArcSegment, const LineSegment &currModelLineSegment) const
{
    return false;
}

bool RoadModelTracker::needReplace(const CircularArc &newArcSegment, const CircularArc &currModelArcSegment) const
{
    if (std::abs(newArcSegment.getRadius() - currModelArcSegment.getRadius()) < this->arcRadiusDelta)
    {
        return true;
    }

    if (std::abs(newArcSegment.getCenter().x - currModelArcSegment.getCenter().x) < this->arcCenterDelta ||
        std::abs(newArcSegment.getCenter().y - currModelArcSegment.getCenter().y) < this->arcCenterDelta)
    {
        return true;
    }

    return false;
}

bool RoadModelTracker::needReplace(const LineSegment &newLineSegment, const CircularArc &currModelArcSegment) const
{
    return false;
}

bool RoadModelTracker::needReplace(const LineSegment &newLineSegment, const LineSegment &currModelLineSegment) const
{
    if (std::abs(newLineSegment.getBeginPoint().x - currModelLineSegment.getBeginPoint().x) < this->lineBeginDelta ||
        std::abs(newLineSegment.getBeginPoint().y - currModelLineSegment.getBeginPoint().y) < this->lineBeginDelta)
    {
        return true;
    }

    if (std::abs(newLineSegment.getEndPoint().x - currModelLineSegment.getEndPoint().x) < this->lineEndDelta ||
        std::abs(newLineSegment.getEndPoint().y - currModelLineSegment.getEndPoint().y) < this->lineEndDelta)
    {
        return true;
    }

    return false;
}

bool RoadModelTracker::needReplace(const CircularArc &newArcSegment, const ModelElement &currModelElement) const
{
}

bool RoadModelTracker::needReplace(const LineSegment &newLineSegment, const ModelElement &currModelElement) const
{
}

void RoadModelTracker::trackRightSide(const LineSegment &newLineSegment)
{
    std::shared_ptr<ModelElement> currModelElement(this->roadModel.getRightHead());

    bool replaced = false;

    while (currModelElement)
    {
        if (needReplace(newLineSegment, *currModelElement))
        {
            this->roadModel.replaceModelRightElement(newLineSegment, *currModelElement);
            replaced = true;
            break;
        }

        currModelElement = currModelElement->next;
    }

    if (!replaced)
    {
        this->roadModel.addElementToRight(newLineSegment);
    }
}

void RoadModelTracker::trackLeftSide(const CircularArc &newArcSegment)
{
    std::shared_ptr<ModelElement> currModelElement(this->roadModel.getLeftHead());

    bool replaced = false;

    while (currModelElement)
    {
        if (needReplace(newArcSegment, *currModelElement))
        {
            this->roadModel.replaceModelLeftElement(newArcSegment, *currModelElement);
            replaced = true;
            break;
        }

        currModelElement = currModelElement->next;
    }

    if (!replaced)
    {
        this->roadModel.addElementToLeft(newArcSegment);
    }
}

void RoadModelTracker::trackLeftSide(const LineSegment &newLineSegment)
{
    std::shared_ptr<ModelElement> currModelElement(this->roadModel.getLeftHead());

    bool replaced = false;

    while (currModelElement)
    {
        if (needReplace(newLineSegment, *currModelElement))
        {
            this->roadModel.replaceModelLeftElement(newLineSegment, *currModelElement);
            replaced = true;
            break;
        }

        currModelElement = currModelElement->next;
    }

    if (!replaced)
    {
        this->roadModel.addElementToLeft(newLineSegment);
    }
}

