//
// Created by alechh on 06.04.2022.
//

#include "RoadModelTracker.h"

#include <utility>

RoadModelTracker::RoadModelTracker(std::shared_ptr<RoadModel> roadModel) : roadModel(std::move(roadModel)),
                                                                           isModelConstructed(false)
{}

void RoadModelTracker::trackRightSide(const CircularArc &newArcSegment)
{
    std::shared_ptr<CircularArc> newArcSegmentPointer = std::make_shared<CircularArc>(newArcSegment);

    std::shared_ptr<ModelElement> currModelElement(this->roadModel->getRightHead());

    bool replaced = false;

    if (this->isModelConstructed)
    {
        while (currModelElement)
        {
            auto *tempLineSegment = dynamic_cast<LineSegment *>(currModelElement.get());

            if (!tempLineSegment) // если тип currModelElement -- CircularArc, то tempLineSegment == nullptr
            {
                auto *tempCircularArc = dynamic_cast<CircularArc *>(currModelElement.get());

                if (!tempCircularArc)
                {
                    return;
                }

                //std::shared_ptr<LineSegment> tempLineSegmentPointer = std::make_shared<LineSegment>(tempLineSegment);
                if (needReplace(newArcSegmentPointer, *tempCircularArc))
                {
                    this->roadModel->replaceModelRightElement(newArcSegmentPointer, currModelElement);
                    replaced = true;
                    break;
                }
            }

            currModelElement = currModelElement->next;
        }
    }

    // TODO Не надо просто добавлять
    if (!replaced)
    {
        this->roadModel->addElementToRight(newArcSegment);
    }
}

bool RoadModelTracker::needReplace(const std::shared_ptr<CircularArc> &newArcSegment,
                                   const CircularArc &currModelArcSegment) const
{
    cv::Point centerDelta = newArcSegment->getCenter() - currModelArcSegment.getCenter();

    centerDelta.x = std::abs(centerDelta.x);
    centerDelta.y = std::abs(centerDelta.y);

    if (centerDelta.x < this->arcCenterDelta && centerDelta.y < this->arcCenterDelta &&
        std::abs(newArcSegment->getRadius() - currModelArcSegment.getRadius()) < this->arcRadiusDelta)
    {
        return true;
    }

    return false;
}

bool RoadModelTracker::needReplace(const std::shared_ptr<LineSegment> &newLineSegment,
                                   const LineSegment &currModelLineSegment) const
{
    cv::Point beginDelta = newLineSegment->getBeginPoint() - currModelLineSegment.getBeginPoint();
    cv::Point endDelta = newLineSegment->getEndPoint() - currModelLineSegment.getEndPoint();

    beginDelta.x = std::abs(beginDelta.x);
    beginDelta.y = std::abs(beginDelta.y);
    endDelta.x = std::abs(endDelta.x);
    endDelta.y = std::abs(endDelta.y);

    if (beginDelta.x < this->lineBeginDelta && beginDelta.y < this->lineBeginDelta &&
        endDelta.x < this->lineEndDelta && endDelta.y < this->lineEndDelta)
    {
        return true;
    }

    return false;
}

void RoadModelTracker::trackRightSide(const LineSegment &newLineSegment)
{
    std::shared_ptr<LineSegment> newLineSegmentPointer = std::make_shared<LineSegment>(newLineSegment);

    std::shared_ptr<ModelElement> currModelElement(this->roadModel->getRightHead());

    bool replaced = false;

    if (this->isModelConstructed)
    {
        while (currModelElement)
        {
            auto *tempCircularArc = dynamic_cast<CircularArc *>(currModelElement.get());

            if (!tempCircularArc) // если тип currModelElement -- LineSegment, то tempLineSegment == nullptr
            {
                auto *tempLineSegment = dynamic_cast<LineSegment *>(currModelElement.get());

                if (needReplace(newLineSegmentPointer, *tempLineSegment))
                {
                    this->roadModel->replaceModelRightElement(newLineSegmentPointer, currModelElement);
                    replaced = true;
                    break;
                }
            }

            currModelElement = currModelElement->next;
        }
    }

    // TODO Не надо просто добавлять
    if (!replaced)
    {
        this->roadModel->addElementToRight(newLineSegment);
    }
}

void RoadModelTracker::trackLeftSide(const CircularArc &newArcSegment)
{
    std::shared_ptr<CircularArc> newArcSegmentPointer = std::make_shared<CircularArc>(newArcSegment);

    std::shared_ptr<ModelElement> currModelElement(this->roadModel->getLeftHead());

    bool replaced = false;

    if (this->isModelConstructed)
    {
        while (currModelElement)
        {
            auto *tempLineSegment = dynamic_cast<LineSegment *>(currModelElement.get());

            if (!tempLineSegment)
            {
                auto *tempCircularArc = dynamic_cast<CircularArc *>(currModelElement.get());

                if (needReplace(newArcSegmentPointer, *tempCircularArc))
                {
                    this->roadModel->replaceModelLeftElement(newArcSegmentPointer, currModelElement);
                    replaced = true;
                    break;
                }
            }

            currModelElement = currModelElement->next;
        }
    }

    // TODO Не надо просто добавлять
    if (!replaced)
    {
        this->roadModel->addElementToLeft(newArcSegment);
    }
}

void RoadModelTracker::trackLeftSide(const LineSegment &newLineSegment)
{
    std::shared_ptr<LineSegment> newLineSegmentPointer = std::make_shared<LineSegment>(newLineSegment);

    std::shared_ptr<ModelElement> currModelElement(this->roadModel->getLeftHead());

    bool replaced = false;

    if (this->isModelConstructed)
    {
        while (currModelElement)
        {
            auto *tempCircularArc = dynamic_cast<CircularArc *>(currModelElement.get());

            if (!tempCircularArc)
            {
                auto *tempLineSegment = dynamic_cast<LineSegment *>(currModelElement.get());

                if (needReplace(newLineSegmentPointer, *tempLineSegment))
                {
                    this->roadModel->replaceModelLeftElement(newLineSegmentPointer, currModelElement);
                    replaced = true;
                    break;
                }
            }

            currModelElement = currModelElement->next;
        }
    }

    // TODO Не надо просто добавлять
    if (!replaced)
    {
        this->roadModel->addElementToLeft(newLineSegment);
    }
}

void RoadModelTracker::modelHasBeenConstructed()
{
    this->isModelConstructed = true;
}

