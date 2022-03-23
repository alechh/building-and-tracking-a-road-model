//
// Created by alechh on 26.02.2022.
//

#include "RoadModel.h"
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <utility>

ModelElement::ModelElement() : next(nullptr) {}

void ModelElement::setNextElement(const ModelElement &nextElement)
{
    this->next = std::make_shared<ModelElement>(nextElement);
}

//ModelElement::ModelElement(const ModelElement &elem): next(nullptr)
//{
//    /**
//     * Подразумевается, что копировать объекты не придется, поэтому поле next всегда заполняется nullptr.
//     * Этот копирующий конструктор нужен, чтобы работала функция ModelElement::setNextElement
//     * (а именно, чтобы создалавлся shared_ptr).
//     */
//}

void ModelElement::drawModelElement(cv::Mat &src) const {}

void ModelElement::drawModelElementPoints(cv::Mat &src) const {}

void ModelElement::setNextElement(const LineSegment &nextElement)
{
    this->next = std::make_shared<LineSegment>(nextElement);
}

void ModelElement::setNextElement(const CircularArc &nextElement)
{
    this->next = std::make_shared<CircularArc>(nextElement);
}

void ModelElement::printInformation() const {}

LineSegment::LineSegment(cv::Point begin, cv::Point end) : ModelElement(), begin(std::move(begin)), end(std::move(end)) {}

LineSegment::LineSegment(cv::Point begin, cv::Point end, const ModelElement &nextElement) : ModelElement(), begin(std::move(begin)), end(std::move(end))
{
    setNextElement(nextElement);
}

void LineSegment::drawModelElement(cv::Mat &src) const
{
    cv::line(src, this->begin, this->end, cv::Scalar(0, 0, 255), 1);
}

void LineSegment::drawModelElementPoints(cv::Mat &src) const
{
    this->drawModelElement(src);
}

void LineSegment::printInformation() const
{
    std::cout << "lineSegment: \n\tbegin: " << this->begin << "\n\tend:" << this->end << std::endl;
}

CircularArc::CircularArc(cv::Point center, double radius, double startAngle, double endAngle, std::vector<cv::Point> points)
        : center(std::move(center)), radius(radius), startAngle(startAngle), endAngle(endAngle), pointsOfTheArc(std::move(points)) {}

CircularArc::CircularArc(cv::Point center, double radius, double startAngle, double endAngle, const ModelElement &nextElement)
        : center(std::move(center)), radius(radius), startAngle(startAngle), endAngle(endAngle)
{
    setNextElement(nextElement);
}

void CircularArc::drawModelElement(cv::Mat &src) const
{
    //cv::circle(src, this->center, 1, cv::Scalar(0, 0, 255));
    cv::ellipse(src, this->center, cv::Size(this->radius, this->radius), 0, this->startAngle, this->endAngle, cv::Scalar(255, 0, 0), 2);
}

void CircularArc::drawModelElementPoints(cv::Mat &src) const
{
    for (const auto &point : this->pointsOfTheArc)
    {
        if (point == this->pointsOfTheArc[0] || point == this->pointsOfTheArc[this->pointsOfTheArc.size() - 1])
        {
            cv::circle(src, point, 4, cv::Scalar(0, 255, 0));
        }
        else
        {
            cv::circle(src, point, 1, cv::Scalar(255, 0, 0));
        }
    }
}

void CircularArc::printInformation() const
{
    std::cout << "circularArc:\n\tcenter" << this->center << "\n\tradius:" << this->radius << std::endl;
}

RoadModel::RoadModel(): leftHead(nullptr), rightHead(nullptr), modelLeftElementCounter(0), modelRightElementCounter(0) {}

RoadModel::RoadModel(ModelElement *leftHead, ModelElement *rightHead): leftHead(leftHead), rightHead(rightHead)
{
    if (this->leftHead)
    {
        modelLeftElementCounter = 1;
    }

    if (this->rightHead)
    {
        modelRightElementCounter = 1;
    }
}

void RoadModel::addElementToRight(cv::Point begin, cv::Point end)
{
    if (this->rightHead)
    {
        std::shared_ptr<ModelElement> curr(rightHead);
        while (curr->next)
        {
            curr = curr->next;
        }
        curr->setNextElement(LineSegment(std::move(begin), std::move(end)));
        this->modelRightElementCounter++;
    }
    else
    {
        this->rightHead = std::make_shared<LineSegment>(std::move(begin), std::move(end));
        this->modelRightElementCounter++;
        //this->rightHead = new LineSegment(std::move(begin), std::move(end));
    }
}

void RoadModel::addElementToRight(cv::Point center, double radius, double startAngle, double endAngle,
                                  std::vector<cv::Point> points)
{
    if (this->rightHead)
    {
        std::shared_ptr<ModelElement> curr(rightHead);
        while (curr->next)
        {
            curr = curr->next;
        }
        curr->setNextElement(CircularArc(std::move(center), radius, startAngle, endAngle, std::move(points)));
        this->modelRightElementCounter++;
    }
    else
    {
        this->rightHead = std::make_shared<CircularArc>(std::move(center), radius, startAngle, endAngle, std::move(points));
        this->modelRightElementCounter++;
        //this->rightHead = new CircularArc(std::move(center), radius);
    }
}

void RoadModel::addElementToLeft(cv::Point begin, cv::Point end)
{
    if (this->leftHead)
    {
        std::shared_ptr<ModelElement> curr(leftHead);
        while (curr->next)
        {
            curr = curr->next;
        }
        curr->setNextElement(LineSegment(std::move(begin), std::move(end)));
        this->modelLeftElementCounter++;
    }
    else
    {
        this->leftHead = std::make_shared<LineSegment>(std::move(begin), std::move(end));
        this->modelLeftElementCounter++;
        //this->leftHead = new LineSegment(std::move(begin), std::move(end));
    }
}

void RoadModel::addElementToLeft(cv::Point center, double radius, double startAngle, double endAngle,
                                 std::vector<cv::Point> points)
{
    if (this->leftHead)
    {
        std::shared_ptr<ModelElement> curr(leftHead);
        while (curr->next)
        {
            curr = curr->next;
        }
        curr->setNextElement(CircularArc(std::move(center), radius, 0, 0, std::move(points)));
        this->modelLeftElementCounter++;
    }
    else
    {
        this->leftHead = std::make_shared<CircularArc>(std::move(center), radius, startAngle, endAngle, std::move(points));
        this->modelLeftElementCounter++;
        //this->leftHead = new CircularArc(std::move(center), radius);
    }
}

int RoadModel::getModelLeftElementCounter() const
{
    return this->modelLeftElementCounter;
}

int RoadModel::getModelRightElementCounter() const
{
    return this->modelRightElementCounter;
}

int RoadModel::countModelRightElements() const
{
    int modelRightElementsCounter = 0;
    std::shared_ptr<ModelElement> curr(this->rightHead);
    while(curr)
    {
        modelRightElementsCounter++;
        curr = curr->next;
    }

    return modelRightElementsCounter;
}

void RoadModel::drawModel(cv::Mat &dst) const
{
    if (this->leftHead)
    {
        drawLeftSide(dst);
    }

    if (this->rightHead)
    {
        drawRightSide(dst);
    }
}

void RoadModel::drawRightSide(cv::Mat &dst) const
{
    std::shared_ptr<ModelElement> currModelElement(this->rightHead);

    while(currModelElement)
    {
        currModelElement->drawModelElement(dst);
        currModelElement = currModelElement->next;
    }
}

void RoadModel::drawLeftSide(cv::Mat &dst) const
{
    std::shared_ptr<ModelElement> currModelElement(this->leftHead);

    while(currModelElement)
    {
        currModelElement->drawModelElement(dst);
        currModelElement = currModelElement->next;
    }
}

void RoadModel::printInformationOfTheRightSide() const
{
    if (this->rightHead)
    {
        std::shared_ptr<ModelElement> currModelElement(this->rightHead);

        while(currModelElement)
        {
            currModelElement->printInformation();
            currModelElement = currModelElement->next;
        }
    }
}

void RoadModel::drawModelPoints(cv::Mat &dst) const
{
    if (this->rightHead)
    {
        drawRightSidePoints(dst);
    }
}

void RoadModel::drawRightSidePoints(cv::Mat &dst) const
{
    std::shared_ptr<ModelElement> currModelElement(this->rightHead);

    while(currModelElement)
    {
        currModelElement->drawModelElementPoints(dst);
        currModelElement = currModelElement->next;
    }
}



