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
