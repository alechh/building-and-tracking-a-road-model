//
// Created by alechh on 26.02.2022.
//

#ifndef BUILDING_AND_TRACKING_A_ROAD_MODEL_ROADMODEL_H
#define BUILDING_AND_TRACKING_A_ROAD_MODEL_ROADMODEL_H


#include <opencv2/core/types.hpp>

class ModelElement
{
public:
    std::shared_ptr<ModelElement> next;
    //ModelElement *next;

    ModelElement();
    virtual ~ModelElement();
    ModelElement(const ModelElement &elem);
    void setNextElement(const ModelElement &nextElement);
    virtual void drawModelElement(cv::Mat &src) const;
};

class LineSegment: public ModelElement
{
private:
    cv::Point begin, end;
public:
    LineSegment(cv::Point begin, cv::Point end);
    LineSegment(cv::Point begin, cv::Point end, ModelElement nextElement);
    void drawModelElement(cv::Mat &src) const override;
};

class CircularArc: public  ModelElement
{
private:
    cv::Point center;
    double radius;
public:
    CircularArc(cv::Point center, double radius);
    CircularArc(cv::Point center, double radius, ModelElement nextElement);
    void drawModelElement(cv::Mat &src) const override;
};


class RoadModel{
    /**
     * Пока предполагается, что строим модель для своей полосы движения
     */
public:
    std::shared_ptr<ModelElement> leftHead, rightHead;
    //ModelElement *leftHead, *rightHead;

    RoadModel();
    RoadModel(ModelElement *leftHead, ModelElement *rightHead);
    void addElementToRight(cv::Point begin, cv::Point end);
    void addElementToRight(cv::Point center, double radius);
    void addElementToLeft(cv::Point begin, cv::Point end);
    void addElementToLeft(cv::Point center, double radius);
};


#endif //BUILDING_AND_TRACKING_A_ROAD_MODEL_ROADMODEL_H
