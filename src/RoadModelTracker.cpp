//
// Created by alechh on 06.04.2022.
//

#include "RoadModelTracker.h"

#include <utility>

RoadModelTracker::RoadModelTracker(RoadModel &roadModel) : roadModel(roadModel)
{}

void RoadModelTracker::trackRightSide(const CircularArc &newArcSegment)
{
    std::shared_ptr<ModelElement> currModelElement(this->roadModel.getRightHead());

    bool replaced = false;

    while (currModelElement)
    {
        auto* tempLineSegment = dynamic_cast<LineSegment*>(currModelElement.get());

        if (!tempLineSegment) // если тип currModelElement -- CircularArc, то tempLineSegment == nullptr
        {
            auto* tempCircularArc = dynamic_cast<CircularArc*>(currModelElement.get());

            //std::shared_ptr<LineSegment> tempLineSegmentPointer = std::make_shared<LineSegment>(tempLineSegment);
            if (needReplace(newArcSegment, *tempCircularArc))
            {
                this->roadModel.replaceModelRightElement(newArcSegment, *currModelElement);
                replaced = true;
                break;
            }
        }

        currModelElement = currModelElement->next;
    }

    // TODO Не надо просто добавлять
    if (!replaced)
    {
        this->roadModel.addElementToRight(newArcSegment);
    }
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

void RoadModelTracker::trackRightSide(const LineSegment &newLineSegment)
{
    std::shared_ptr<ModelElement> currModelElement(this->roadModel.getRightHead());

    bool replaced = false;

    while (currModelElement)
    {
        auto* tempCircularArc = dynamic_cast<CircularArc*>(currModelElement.get());

        if (!tempCircularArc) // если тип currModelElement -- LineSegment, то tempLineSegment == nullptr
        {
            auto* tempLineSegment = dynamic_cast<LineSegment*>(currModelElement.get());

            if (needReplace(newLineSegment, *tempLineSegment))
            {
                this->roadModel.replaceModelRightElement(newLineSegment, *currModelElement);
                replaced = true;
                break;
            }
        }

        currModelElement = currModelElement->next;
    }

    // TODO Не надо просто добавлять
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
        auto* tempLineSegment = dynamic_cast<LineSegment*>(currModelElement.get());

        if (!tempLineSegment)
        {
            auto* tempCircularArc = dynamic_cast<CircularArc*>(currModelElement.get());

            if (needReplace(newArcSegment, *tempCircularArc))
            {
                this->roadModel.replaceModelLeftElement(newArcSegment, *currModelElement);
                replaced = true;
                break;
            }
        }

        currModelElement = currModelElement->next;
    }

    // TODO Не надо просто добавлять
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
        auto* tempCircularArc = dynamic_cast<CircularArc*>(currModelElement.get());

        if (!tempCircularArc)
        {
            auto* tempLineSegment = dynamic_cast<LineSegment*>(currModelElement.get());

            if (needReplace(newLineSegment, *tempLineSegment))
            {
                this->roadModel.replaceModelLeftElement(newLineSegment, *currModelElement);
                replaced = true;
                break;
            }
        }

        currModelElement = currModelElement->next;
    }

    // TODO Не надо просто добавлять
    if (!replaced)
    {
        this->roadModel.addElementToLeft(newLineSegment);
    }
}

