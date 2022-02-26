//
// Created by alechh on 26.02.2022.
//

#include "RoadModel.h"

ModelElement::ModelElement() : next(nullptr) {}

ModelElement::~ModelElement()
{
    delete this->next;
}

void ModelElement::setNextElement(ModelElement *nextElement)
{
    this->next = nextElement;
}

ModelElement *ModelElement::getNext() const
{
    return this->next;
}

LineSegment::LineSegment(cv::Point begin, cv::Point end) : ModelElement(), begin(begin), end(end) {}

LineSegment::LineSegment(cv::Point begin, cv::Point end, ModelElement *nextElement) : ModelElement(), begin(begin), end(end)
{
    setNextElement(nextElement);
}

CircularArc::CircularArc(cv::Point center, double radius): center(center), radius(radius) {}

CircularArc::CircularArc(cv::Point center, double radius, ModelElement *nextElement): center(center), radius(radius)
{
    setNextElement(nextElement);
}

RoadModel::RoadModel(): leftHead(nullptr), rightHead(nullptr) {}

RoadModel::RoadModel(ModelElement *leftHead, ModelElement *rightHead): leftHead(leftHead), rightHead(rightHead) {}

void RoadModel::addElementToRight(cv::Point begin, cv::Point end)
{
    if (this->rightHead)
    {
        this->rightHead->setNextElement(new LineSegment(std::move(begin), std::move(end)));
    }
    else
    {
        this->rightHead = new LineSegment(std::move(begin), std::move(end));
    }
}

void RoadModel::addElementToRight(cv::Point center, double radius)
{
    if (this->rightHead)
    {
        this->rightHead->setNextElement(new CircularArc(std::move(center), radius));
    }
    else
    {
        this->rightHead = new CircularArc(std::move(center), radius);
    }
}

void RoadModel::addElementToLeft(cv::Point begin, cv::Point end)
{
    if (this->leftHead)
    {
        this->leftHead->setNextElement(new LineSegment(std::move(begin), std::move(end)));
    }
    else
    {
        this->leftHead = new LineSegment(std::move(begin), std::move(end));
    }
}

void RoadModel::addElementToLeft(cv::Point center, double radius)
{
    if (this->leftHead)
    {
        this->leftHead->setNextElement(new CircularArc(std::move(center), radius));
    }
    else
    {
        this->leftHead = new CircularArc(std::move(center), radius);
    }
}


