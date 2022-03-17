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
    //ModelElement *next;

    ModelElement();
    virtual ~ModelElement() = default;
    //ModelElement(const ModelElement &elem);
    void setNextElement(const ModelElement &nextElement);
    void setNextElement(const LineSegment &nextElement);
    void setNextElement(const CircularArc &nextElement);
    virtual void drawModelElement(cv::Mat &src) const;
    virtual void printInformation() const;
};

class LineSegment: public ModelElement
{
private:
    cv::Point begin, end;
public:
    LineSegment(cv::Point begin, cv::Point end);
    LineSegment(cv::Point begin, cv::Point end, ModelElement nextElement);
    void drawModelElement(cv::Mat &src) const override;
    void printInformation() const override;
};

class CircularArc: public  ModelElement
{
private:
    cv::Point center;
    double radius;
    double startAngle, endAngle;
public:
    CircularArc(cv::Point center, double radius, double startAndle, double endAngle);
    CircularArc(cv::Point center, double radius, double startAngle, double endAngle, ModelElement nextElement);
    void drawModelElement(cv::Mat &src) const override;
    void printInformation() const override;
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

public:
    std::shared_ptr<ModelElement> leftHead, rightHead;
    //ModelElement *leftHead, *rightHead;

    RoadModel();
    RoadModel(ModelElement *leftHead, ModelElement *rightHead);
    void addElementToRight(cv::Point begin, cv::Point end);
    void addElementToRight(cv::Point center, double radius, double startAngle, double endAngle);
    void addElementToLeft(cv::Point begin, cv::Point end);
    void addElementToLeft(cv::Point center, double radius, double startAngle, double endAngle);
    int getModelLeftElementCounter() const;
    int getModelRightElementCounter() const;
    int countModelRightElements() const;
    void drawModel(cv::Mat &dst) const;
    void printInformationOfTheRightSide() const;

};


#endif //BUILDING_AND_TRACKING_A_ROAD_MODEL_ROADMODEL_H
