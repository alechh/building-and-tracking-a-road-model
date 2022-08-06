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

    virtual void drawModelElement(cv::Mat &src, int size = 0) const;

    virtual void printInformation() const;

    virtual void drawModelElementPoints(cv::Mat &src, int size = 0) const;
};

class LineSegment : public ModelElement
{
private:
    cv::Point begin, end;
public:
    LineSegment(cv::Point begin, cv::Point end);

    void drawModelElement(cv::Mat &src, int size = 0) const override;

    void printInformation() const override;

    void drawModelElementPoints(cv::Mat &src, int size = 0) const override;

    cv::Point getBeginPoint() const;

    cv::Point getEndPoint() const;

    friend void addLineSegmentPoints(std::vector<cv::Point2f> &modelPoints, const LineSegment &lineSegment);
};

class CircularArc : public ModelElement
{
private:
    cv::Point center;
    double radius;
    double startAngle, endAngle;

    // TODO в последствии это поле нужно будет убрать, пока оно нужно для дебага
    std::vector<cv::Point> pointsOfTheArc;
public:
    CircularArc(cv::Point center, double radius, double startAngle, double endAngle, std::vector<cv::Point> points);

    void drawModelElement(cv::Mat &src, int size = 0) const override;

    void printInformation() const override;

    void drawModelElementPoints(cv::Mat &src, int size = 0) const override;

    double getRadius() const;

    cv::Point getCenter() const;

    double getStartAngle() const;

    double getEndAngle() const;
};


class RoadModel
{
    /**
     * Пока предполагается, что строим модель для своей полосы движения
     */
private:
    std::shared_ptr<ModelElement> leftHead, rightHead;

    int modelLeftElementCounter;
    int modelRightElementCounter;

    const int MAX_DISTANCE_BETWEEN_ADJACENT_MODEL_ELEMENTS = 100;

    void drawLeftSide(cv::Mat &dst, int size = 0) const;

    void drawRightSide(cv::Mat &dst, int size = 0) const;

    void drawRightSidePoints(cv::Mat &dst) const;

    void drawLeftSidePoints(cv::Mat &dst) const;

    void printInformationOfTheRightSide() const;

    void printInformationOfTheLeftSide() const;

    bool addConnectingSegment(std::shared_ptr<LineSegment> &lastLineSegment, const LineSegment &newLineSegment);

    bool addConnectingSegment(std::shared_ptr<CircularArc> &lastCircularArc, const LineSegment &newLineSegment);

    bool addConnectingSegment(std::shared_ptr<LineSegment> &lastLineSegment, const CircularArc &newCircularArc);

    bool addConnectingSegment(std::shared_ptr<CircularArc> &lastCircularArc, const CircularArc &newCircularArc);

    bool addConnectingSegment(std::shared_ptr<ModelElement> &lastModelElement, const CircularArc &newCircularArc);

    bool addConnectingSegment(std::shared_ptr<ModelElement> &lastModelElement, const LineSegment &newLineSegment);

public:
    RoadModel();

    void addElementToRight(cv::Point begin, cv::Point end);

    void addElementToRight(cv::Point center, double radius, double startAngle, double endAngle,
                           std::vector<cv::Point> points);

    void addElementToRight(const CircularArc &newCircularArc);

    void addElementToRight(const LineSegment &newLineSegment);

    void addElementToLeft(cv::Point begin, cv::Point end);

    void addElementToLeft(cv::Point center, double radius, double startAngle, double endAngle,
                          std::vector<cv::Point> points);

    void addElementToLeft(const CircularArc &newCircularArc);

    void addElementToLeft(const LineSegment &newLineSegment);

    int getModelLeftElementCounter() const;

    int getModelRightElementCounter() const;

    void drawModel(cv::Mat &dst, int size = 0) const;

    void printInformationOfTheModel() const;

    void drawModelPoints(cv::Mat &dst) const;

    std::shared_ptr<ModelElement> getRightHead() const;

    std::shared_ptr<ModelElement> getLeftHead() const;

    void replaceModelRightElement(const std::shared_ptr<ModelElement> &newModelElement,
                                  const std::shared_ptr<ModelElement> &prevModelElement);

    void replaceModelLeftElement(const std::shared_ptr<ModelElement> &newModelElement,
                                 const std::shared_ptr<ModelElement> &prevModelElement);

    std::vector<cv::Point2f> getModelPoints() const;
};


#endif //BUILDING_AND_TRACKING_A_ROAD_MODEL_ROADMODEL_H
