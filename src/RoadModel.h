//
// Created by alechh on 26.02.2022.
//

#ifndef BUILDING_AND_TRACKING_A_ROAD_MODEL_ROADMODEL_H
#define BUILDING_AND_TRACKING_A_ROAD_MODEL_ROADMODEL_H


#include <opencv2/core/types.hpp>

class LineSegment;
class CircularArc;

class ModelElement
{
public:
    std::shared_ptr<ModelElement> next;

    ModelElement();
    virtual ~ModelElement() = default;
    void setNextElement(const ModelElement &nextElement);
    void setNextElement(const LineSegment &nextElement);
    void setNextElement(const CircularArc &nextElement);
    virtual void drawModelElement(cv::Mat &src) const;
    virtual void printInformation() const;
    virtual void drawModelElementPoints(cv::Mat &src) const;
};

class LineSegment: public ModelElement
{
private:
    cv::Point begin, end;
public:
    LineSegment(cv::Point begin, cv::Point end);
    LineSegment(cv::Point begin, cv::Point end, const ModelElement &nextElement);
    void drawModelElement(cv::Mat &src) const override;
    void printInformation() const override;
    void drawModelElementPoints(cv::Mat &src) const override;
};

class CircularArc: public  ModelElement
{
private:
    cv::Point center;
    double radius;
    double startAngle, endAngle;
    std::vector<cv::Point> pointsOfTheArc;
public:
    CircularArc(cv::Point center, double radius, double startAngle, double endAngle, std::vector<cv::Point> points);
    CircularArc(cv::Point center, double radius, double startAngle, double endAngle, const ModelElement &nextElement);
    void drawModelElement(cv::Mat &src) const override;
    void printInformation() const override;
    void drawModelElementPoints(cv::Mat &src) const override;
};


class RoadModel{
    /**
     * Пока предполагается, что строим модель для своей полосы движения
     */
private:
    int modelLeftElementCounter;
    int modelRightElementCounter;

    void drawLeftSide(cv::Mat &dst) const;
    void drawRightSide(cv::Mat &dst) const;
    void drawRightSidePoints(cv::Mat &dst) const;
    void printInformationOfTheRightSide() const;
    void printInformationOfTheLeftSide() const;

public:
    std::shared_ptr<ModelElement> leftHead, rightHead;

    RoadModel();
    void addElementToRight(cv::Point begin, cv::Point end);
    void addElementToRight(cv::Point center, double radius, double startAngle, double endAngle,
                           std::vector<cv::Point> points);
    void addElementToLeft(cv::Point begin, cv::Point end);
    void addElementToLeft(cv::Point center, double radius, double startAngle, double endAngle,
                          std::vector<cv::Point> points);
    int getModelLeftElementCounter() const;
    int getModelRightElementCounter() const;
    void drawModel(cv::Mat &dst) const;
    void printInformationOfTheModel() const;
    void drawModelPoints(cv::Mat &dst) const;
};


#endif //BUILDING_AND_TRACKING_A_ROAD_MODEL_ROADMODEL_H
