//
// Created by alechh on 26.02.2022.
//

#ifndef BUILDING_AND_TRACKING_A_ROAD_MODEL_ROADMODEL_H
#define BUILDING_AND_TRACKING_A_ROAD_MODEL_ROADMODEL_H


#include <opencv2/core/types.hpp>

class ModelElement
{
private:
    ModelElement *next;
public:
    ModelElement();
    ~ModelElement();
    void setNextElement(ModelElement *nextElement);
    ModelElement* getNext() const;
};

class LineSegment: public ModelElement
{
private:
    cv::Point begin, end;
public:
    LineSegment(cv::Point begin, cv::Point end);
    LineSegment(cv::Point begin, cv::Point end, ModelElement *nextElement);
};

class CircularArc: public  ModelElement
{
private:
    cv::Point center;
    double radius;
public:
    CircularArc(cv::Point center, double radius);
    CircularArc(cv::Point center, double radius, ModelElement *nextElement);
};


class RoadModel{
    /**
     * Пока предполагается, что строим модель для своей полосы движения
     */
public:
    ModelElement *leftHead, *rightHead;
    RoadModel();
    RoadModel(ModelElement *leftHead, ModelElement *rightHead);
    void addElementToRight(cv::Point begin, cv::Point end);
    void addElementToRight(cv::Point center, double radius);
    void addElementToLeft(cv::Point begin, cv::Point end);
    void addElementToLeft(cv::Point center, double radius);
};


#endif //BUILDING_AND_TRACKING_A_ROAD_MODEL_ROADMODEL_H
