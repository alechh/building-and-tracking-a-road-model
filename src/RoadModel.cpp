//
// Created by alechh on 26.02.2022.
//

#include "RoadModel.h"
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <utility>
#include "Utils.h"

ModelElement::ModelElement() : next(nullptr)
{}

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

void ModelElement::drawModelElement(cv::Mat &src) const
{}

void ModelElement::drawModelElementPoints(cv::Mat &src) const
{}

void ModelElement::setNextElement(const LineSegment &nextElement)
{
    this->next = std::make_shared<LineSegment>(nextElement);
}

void ModelElement::setNextElement(const CircularArc &nextElement)
{
    this->next = std::make_shared<CircularArc>(nextElement);
}

void ModelElement::printInformation() const
{}

LineSegment::LineSegment(cv::Point begin, cv::Point end) : ModelElement(), begin(std::move(begin)), end(std::move(end))
{}

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
    std::cout << "\tlineSegment: \n\t\tbegin: " << this->begin << "\n\t\tend:" << this->end << std::endl;
}

cv::Point LineSegment::getBeginPoint() const
{
    return this->begin;
}

cv::Point LineSegment::getEndPoint() const
{
    return this->end;
}

CircularArc::CircularArc(cv::Point center, double radius, double startAngle, double endAngle,
                         std::vector<cv::Point> points)
        : center(std::move(center)), radius(radius), startAngle(startAngle), endAngle(endAngle),
          pointsOfTheArc(std::move(points))
{
    if (std::isnan(this->radius))
    {
        this->radius = 0;
    }
    if (std::isnan(this->startAngle))
    {
        this->startAngle = 0;
    }
    if (std::isnan(this->endAngle))
    {
        this->endAngle = 0;
    }
}

void CircularArc::drawModelElement(cv::Mat &src) const
{
    //cv::circle(src, this->center, 1, cv::Scalar(0, 0, 255));
    cv::ellipse(src, this->center, cv::Size(this->radius, this->radius), 0, this->startAngle, this->endAngle,
                cv::Scalar(255, 0, 0), 1);
}

void CircularArc::drawModelElementPoints(cv::Mat &src) const
{
    for (const auto &point: this->pointsOfTheArc)
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
    std::cout << "\tcircularArc:\n\t\tcenter" << this->center << "\n\t\tradius:" << this->radius << std::endl;
}

double CircularArc::getRadius() const
{
    return this->radius;
}

cv::Point CircularArc::getCenter() const
{
    return this->center;
}

double CircularArc::getStartAngle() const
{
    return this->startAngle;
}

double CircularArc::getEndAngle() const
{
    return this->endAngle;
}

RoadModel::RoadModel() : leftHead(std::shared_ptr<ModelElement>(nullptr)),
                         rightHead(std::shared_ptr<ModelElement>(nullptr)), modelLeftElementCounter(0),
                         modelRightElementCounter(0)
{}

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
        this->rightHead = std::make_shared<CircularArc>(std::move(center), radius, startAngle, endAngle,
                                                        std::move(points));
        this->modelRightElementCounter++;
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
        curr->setNextElement(CircularArc(std::move(center), radius, startAngle, endAngle, std::move(points)));
        this->modelLeftElementCounter++;
    }
    else
    {
        this->leftHead = std::make_shared<CircularArc>(std::move(center), radius, startAngle, endAngle,
                                                       std::move(points));
        this->modelLeftElementCounter++;
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

    while (currModelElement)
    {
        currModelElement->drawModelElement(dst);
        currModelElement = currModelElement->next;
    }
}

void RoadModel::drawLeftSide(cv::Mat &dst) const
{
    std::shared_ptr<ModelElement> currModelElement(this->leftHead);

    while (currModelElement)
    {
        currModelElement->drawModelElement(dst);
        currModelElement = currModelElement->next;
    }
}

void RoadModel::printInformationOfTheRightSide() const
{
    if (this->rightHead)
    {
        std::cout << "Right Side:" << std::endl;
        std::shared_ptr<ModelElement> currModelElement(this->rightHead);

        while (currModelElement)
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

    if (this->leftHead)
    {
        drawLeftSidePoints(dst);
    }
}

void RoadModel::drawRightSidePoints(cv::Mat &dst) const
{
    std::shared_ptr<ModelElement> currModelElement(this->rightHead);

    while (currModelElement)
    {
        currModelElement->drawModelElementPoints(dst);
        currModelElement = currModelElement->next;
    }
}

void RoadModel::printInformationOfTheLeftSide() const
{
    if (this->leftHead)
    {
        std::shared_ptr<ModelElement> currModelElement(this->leftHead);
        std::cout << "Left Side:" << std::endl;

        while (currModelElement)
        {
            currModelElement->printInformation();
            currModelElement = currModelElement->next;
        }
    }
}

void RoadModel::printInformationOfTheModel() const
{
    this->printInformationOfTheLeftSide();
    this->printInformationOfTheRightSide();
}

std::shared_ptr<ModelElement> RoadModel::getRightHead() const
{
    return this->rightHead;
}

std::shared_ptr<ModelElement> RoadModel::getLeftHead() const
{
    return this->leftHead;
}

void RoadModel::replaceModelRightElement(const std::shared_ptr<ModelElement> &newModelElement,
                                         const std::shared_ptr<ModelElement> &prevModelElement)
{
    std::shared_ptr<ModelElement> tempElement(this->rightHead);

    if (tempElement != prevModelElement)
    {
        while (tempElement->next != prevModelElement && tempElement->next)
        {
            tempElement = tempElement->next;
        }

        tempElement->next = newModelElement;
        newModelElement->next = prevModelElement->next;
    }
    else
    {
        auto tempCircularArcPointer = dynamic_cast<CircularArc *>(newModelElement.get());
        if (!tempCircularArcPointer)
        {
            LineSegment *tempLineSegmentPointer = nullptr;
            tempLineSegmentPointer = dynamic_cast<LineSegment *>(newModelElement.get());

            std::shared_ptr<LineSegment> newModelLineSegment = std::make_shared<LineSegment>(*tempLineSegmentPointer);

            this->rightHead = newModelLineSegment;
            newModelLineSegment->next = tempElement->next;
        }
        else
        {
            std::shared_ptr<CircularArc> newModelCircularArc = std::make_shared<CircularArc>(*tempCircularArcPointer);

            this->rightHead = newModelCircularArc;
            newModelCircularArc->next = tempElement->next;
        }
    }
}

void RoadModel::replaceModelLeftElement(const std::shared_ptr<ModelElement> &newModelElement,
                                        const std::shared_ptr<ModelElement> &prevModelElement)
{
    std::shared_ptr<ModelElement> tempElement(this->leftHead);

    if (tempElement != prevModelElement)
    {
        while (tempElement->next != prevModelElement && tempElement->next)
        {
            tempElement = tempElement->next;
        }

        tempElement->next = newModelElement;
        newModelElement->next = prevModelElement->next;
    }
    else
    {
        auto tempCircularArcPointer = dynamic_cast<CircularArc *>(newModelElement.get());
        if (!tempCircularArcPointer)
        {
            LineSegment *tempLineSegmentPointer = nullptr;
            tempLineSegmentPointer = dynamic_cast<LineSegment *>(newModelElement.get());

            std::shared_ptr<LineSegment> newModelLineSegment = std::make_shared<LineSegment>(*tempLineSegmentPointer);

            this->leftHead = newModelLineSegment;
            newModelLineSegment->next = tempElement->next;
        }
        else
        {
            std::shared_ptr<CircularArc> newModelCircularArc = std::make_shared<CircularArc>(*tempCircularArcPointer);

            this->leftHead = newModelCircularArc;
            newModelCircularArc->next = tempElement->next;
        }
    }
}

void RoadModel::addElementToRight(const CircularArc &newCircularArc)
{
    if (this->rightHead)
    {
        std::shared_ptr<ModelElement> curr(rightHead);
        while (curr->next)
        {
            curr = curr->next;
        }

//        if (addConnectingSegment(curr, newCircularArc))
//        {
//            //std::cout << "connecting segment was added" << std::endl;
//            curr = curr->next;
//        }

        curr->next = std::make_shared<CircularArc>(newCircularArc);
        this->modelRightElementCounter++;
    }
    else
    {
        this->rightHead = std::make_shared<CircularArc>(newCircularArc);
        this->modelRightElementCounter++;
    }
}

void RoadModel::addElementToLeft(const CircularArc &newCircularArc)
{
    if (this->leftHead)
    {
        std::shared_ptr<ModelElement> curr(leftHead);
        while (curr->next)
        {
            curr = curr->next;
        }

//        if (addConnectingSegment(curr, newCircularArc))
//        {
//            //std::cout << "connecting segment was added" << std::endl;
//            curr = curr->next;
//        }

        curr->next = std::make_shared<CircularArc>(newCircularArc);
        this->modelLeftElementCounter++;
    }
    else
    {
        this->leftHead = std::make_shared<CircularArc>(newCircularArc);
        this->modelLeftElementCounter++;
    }
}

void RoadModel::addElementToRight(const LineSegment &newLineSegment)
{
    if (this->rightHead)
    {
        std::shared_ptr<ModelElement> curr(rightHead);
        while (curr->next)
        {
            curr = curr->next;
        }

//        if (addConnectingSegment(curr, newLineSegment))
//        {
//            //std::cout << "connecting segment was added" << std::endl;
//            curr = curr->next;
//        }

        curr->next = std::make_shared<LineSegment>(newLineSegment);
        this->modelRightElementCounter++;
    }
    else
    {
        this->rightHead = std::make_shared<LineSegment>(newLineSegment);
        this->modelRightElementCounter++;
    }
}

void RoadModel::addElementToLeft(const LineSegment &newLineSegment)
{
    if (this->leftHead)
    {
        std::shared_ptr<ModelElement> curr(leftHead);
        while (curr->next)
        {
            curr = curr->next;
        }

//        if (addConnectingSegment(curr, newLineSegment))
//        {
//            //std::cout << "connecting segment was added" << std::endl;
//            curr = curr->next;
//        }

        curr->next = std::make_shared<LineSegment>(newLineSegment);
        this->modelLeftElementCounter++;
    }
    else
    {
        this->leftHead = std::make_shared<LineSegment>(newLineSegment);
        this->modelLeftElementCounter++;
    }
}

bool RoadModel::addConnectingSegment(std::shared_ptr<LineSegment> &lastLineSegment, const LineSegment &newLineSegment)
{
    int distance = Utils::distanceBetweenPoints(lastLineSegment->getEndPoint(), newLineSegment.getBeginPoint());

    if (distance != 0 && distance < this->MAX_DISTANCE_BETWEEN_ADJACENT_MODEL_ELEMENTS)
    {
        lastLineSegment->next = std::make_shared<LineSegment>(lastLineSegment->getEndPoint(),
                                                              newLineSegment.getBeginPoint());
        return true;
    }

    return false;
}

bool RoadModel::addConnectingSegment(std::shared_ptr<CircularArc> &lastCircularArc, const LineSegment &newLineSegment)
{
    cv::Point beginPoint;
    beginPoint.x = lastCircularArc->getCenter().x +
                   lastCircularArc->getRadius() * cos(lastCircularArc->getStartAngle() / 180 * CV_PI);
    beginPoint.y = lastCircularArc->getCenter().y +
                   lastCircularArc->getRadius() * sin(lastCircularArc->getStartAngle() / 180 * CV_PI);

    cv::Point endPoint;
    endPoint.x = lastCircularArc->getCenter().x +
                 lastCircularArc->getRadius() * cos(lastCircularArc->getEndAngle() / 180 * CV_PI);
    endPoint.y = lastCircularArc->getCenter().y +
                 lastCircularArc->getRadius() * sin(lastCircularArc->getEndAngle() / 180 * CV_PI);

    int distanceWithBeginPoint = Utils::distanceBetweenPoints(newLineSegment.getBeginPoint(), beginPoint);
    int distanceWithEndPoint = Utils::distanceBetweenPoints(newLineSegment.getBeginPoint(), endPoint);

    cv::Point correctPoint;
    int correctDistance;

    if (distanceWithBeginPoint < distanceWithEndPoint)
    {
        correctPoint = beginPoint;
        correctDistance = distanceWithBeginPoint;
    }
    else
    {
        correctPoint = endPoint;
        correctDistance = distanceWithEndPoint;
    }

    if (correctDistance != 0 && correctDistance < this->MAX_DISTANCE_BETWEEN_ADJACENT_MODEL_ELEMENTS)
    {
        lastCircularArc->next = std::make_shared<LineSegment>(correctPoint, newLineSegment.getBeginPoint());
        return true;
    }

    return false;
}

bool RoadModel::addConnectingSegment(std::shared_ptr<LineSegment> &lastLineSegment, const CircularArc &newCircularArc)
{
    cv::Point beginPoint;
    beginPoint.x = newCircularArc.getCenter().x +
                   newCircularArc.getRadius() * cos(newCircularArc.getStartAngle() / 180 * CV_PI);
    beginPoint.y = newCircularArc.getCenter().y +
                   newCircularArc.getRadius() * sin(newCircularArc.getStartAngle() / 180 * CV_PI);

    cv::Point endPoint;
    beginPoint.x =
            newCircularArc.getCenter().x + newCircularArc.getRadius() * cos(newCircularArc.getEndAngle() / 180 * CV_PI);
    beginPoint.y =
            newCircularArc.getCenter().y + newCircularArc.getRadius() * sin(newCircularArc.getEndAngle() / 180 * CV_PI);

    int distanceWithBeginPoint = Utils::distanceBetweenPoints(lastLineSegment->getEndPoint(), beginPoint);
    int distanceWithEndPoint = Utils::distanceBetweenPoints(lastLineSegment->getEndPoint(), endPoint);

    cv::Point correctPoint;
    int correctDistance;

    if (distanceWithBeginPoint < distanceWithEndPoint)
    {
        correctPoint = beginPoint;
        correctDistance = distanceWithBeginPoint;
    }
    else
    {
        correctPoint = endPoint;
        correctDistance = distanceWithEndPoint;
    }

    if (correctDistance != 0 && correctDistance < this->MAX_DISTANCE_BETWEEN_ADJACENT_MODEL_ELEMENTS)
    {
        lastLineSegment->next = std::make_shared<LineSegment>(lastLineSegment->getEndPoint(), correctPoint);
        return true;
    }

    return false;
}

bool RoadModel::addConnectingSegment(std::shared_ptr<CircularArc> &lastCircularArc, const CircularArc &newCircularArc)
{
    cv::Point beginPointOfLastArc;
    beginPointOfLastArc.x = lastCircularArc->getCenter().x +
                            lastCircularArc->getRadius() * cos(lastCircularArc->getStartAngle() / 180 * CV_PI);
    beginPointOfLastArc.y = lastCircularArc->getCenter().y +
                            lastCircularArc->getRadius() * sin(lastCircularArc->getStartAngle() / 180 * CV_PI);

    cv::Point endPointOfLastArc;
    endPointOfLastArc.x = lastCircularArc->getCenter().x +
                          lastCircularArc->getRadius() * cos(lastCircularArc->getEndAngle() / 180 * CV_PI);
    endPointOfLastArc.y = lastCircularArc->getCenter().y +
                          lastCircularArc->getRadius() * sin(lastCircularArc->getEndAngle() / 180 * CV_PI);

    cv::Point beginPointOfNewArc;
    beginPointOfNewArc.x = newCircularArc.getCenter().x +
                           newCircularArc.getRadius() * cos(newCircularArc.getStartAngle() / 180 * CV_PI);
    beginPointOfNewArc.y = newCircularArc.getCenter().y +
                           newCircularArc.getRadius() * sin(newCircularArc.getStartAngle() / 180 * CV_PI);

    cv::Point endPointOfNewArc;
    beginPointOfLastArc.x =
            newCircularArc.getCenter().x + newCircularArc.getRadius() * cos(newCircularArc.getEndAngle() / 180 * CV_PI);
    beginPointOfLastArc.y =
            newCircularArc.getCenter().y + newCircularArc.getRadius() * sin(newCircularArc.getEndAngle() / 180 * CV_PI);

    int distanceWithBeginPoints = Utils::distanceBetweenPoints(beginPointOfNewArc, beginPointOfLastArc);
    int distanceWithEndPoints = Utils::distanceBetweenPoints(endPointOfNewArc, endPointOfLastArc);
    int distanceBeginNewEndLast = Utils::distanceBetweenPoints(beginPointOfNewArc, endPointOfLastArc);
    int distanceEndNewBeginLast = Utils::distanceBetweenPoints(endPointOfNewArc, beginPointOfLastArc);

    if (distanceWithBeginPoints > this->MAX_DISTANCE_BETWEEN_ADJACENT_MODEL_ELEMENTS &&
        distanceWithEndPoints > this->MAX_DISTANCE_BETWEEN_ADJACENT_MODEL_ELEMENTS &&
        distanceBeginNewEndLast > this->MAX_DISTANCE_BETWEEN_ADJACENT_MODEL_ELEMENTS &&
        distanceEndNewBeginLast > this->MAX_DISTANCE_BETWEEN_ADJACENT_MODEL_ELEMENTS)
    {
        return false;
    }

    std::string errorMessage = "RoadModel::addConnectingSegment(std::shared_ptr<ModelElement> &lastModelElement, const CircularArc &newCircularArc)";
    errorMessage += "\n lastModelElement = CircularArc, newModelElement = CircularArc";
    throw errorMessage;

    cv::Point correctPoint;
    int correctDistance;

    int distance = Utils::distanceBetweenPoints(endPointOfLastArc, beginPointOfNewArc);

    if (distance != 0 && distance < this->MAX_DISTANCE_BETWEEN_ADJACENT_MODEL_ELEMENTS)
    {
        lastCircularArc->next = std::make_shared<LineSegment>(endPointOfLastArc, beginPointOfNewArc);
        return true;
    }

    return false;
}

bool RoadModel::addConnectingSegment(std::shared_ptr<ModelElement> &lastModelElement, const CircularArc &newCircularArc)
{
    std::shared_ptr<CircularArc> tempCircularArcPointer = std::dynamic_pointer_cast<CircularArc>(lastModelElement);

    if (tempCircularArcPointer) // если тип currModelElement -- LineSegment, то tempLineSegment == nullptr
    {
        return false;

        /*
         * Решил пока их не соединять, потому что не предполагается, что не может быть ситуации, когда подряд в модели идут две дуги
         * Если же на деле это так, то это на самом деле две дуги, относящиеся к разным контурам
         */
        return addConnectingSegment(tempCircularArcPointer, newCircularArc);
    }
    else
    {
        std::shared_ptr<LineSegment> tempLineSegmentPointer = std::dynamic_pointer_cast<LineSegment>(lastModelElement);
        return addConnectingSegment(tempLineSegmentPointer, newCircularArc);
    }
}

bool RoadModel::addConnectingSegment(std::shared_ptr<ModelElement> &lastModelElement, const LineSegment &newLineSegment)
{
    std::shared_ptr<CircularArc> tempCircularArcPointer = std::dynamic_pointer_cast<CircularArc>(lastModelElement);

    if (tempCircularArcPointer) // если тип currModelElement -- LineSegment, то tempLineSegment == nullptr
    {
        return addConnectingSegment(tempCircularArcPointer, newLineSegment);
    }
    else
    {
        std::shared_ptr<LineSegment> tempLineSegmentPointer = std::dynamic_pointer_cast<LineSegment>(lastModelElement);
        return addConnectingSegment(tempLineSegmentPointer, newLineSegment);
    }
}

void RoadModel::drawLeftSidePoints(cv::Mat &dst) const
{
    std::shared_ptr<ModelElement> currModelElement(this->leftHead);

    while (currModelElement)
    {
        currModelElement->drawModelElementPoints(dst);
        currModelElement = currModelElement->next;
    }
}

