//
// Created by alechh on 21.02.2022.
//

#include <opencv2/opencv.hpp>
#include "ExperimentWithCurvatureCalculation.h"
#include "RoadModel.h"
#include "Utils.h"


void ExperimentWithCurvatureCalculation::drawArc(cv::Mat &src, double radius, cv::Point center, const cv::Scalar &color = cv::Scalar(0, 0, 255))
{
    /**
     * https://docs.opencv.org/3.4/d6/d6e/group__imgproc__draw.html#ga28b2267d35786f5f890ca167236cbc69
     */
    cv::ellipse(src, center, cv::Size(radius, radius), 180 / CV_PI, 0, 360 / 2, color, 2);
}

void ExperimentWithCurvatureCalculation::drawArcsOnContour(cv::Mat &src, const std::vector<cv::Point> &contour, const std::vector<double> &contourCurvature)
{
    if (contour.size() != contourCurvature.size())
    {
        std::cerr << "ExperimentWithCurvatureCalculation::drawArcsOnContour: contour.size() != curvature.size()" << std::endl;
        return;
    }

    for (int i = 0; i < contour.size(); ++i)
    {
        if (contourCurvature[i] != std::numeric_limits<double>::infinity())
        {
            if (contourCurvature[i] == 0)
            {
                drawArc(src, 3, contour[i], cv::Scalar(0, 255, 0));
            }
            else
            {
                drawArc(src, contourCurvature[i], contour[i]);
            }
        }
    }
}


cv::Point getCenterOfTheArc(const std::vector<cv::Point> &segment, double R)
{
    cv::Point center;

    if (segment.size() < 3)
    {
        std::cerr << "cv::Point getCenterOfTheArc -- segment.size() < 3 (=" << segment.size() << ")" << std::endl;
        return center;
    }

    int pMinusIndex, pPlusIndex;
    cv::Point pMinus, pPlus, centerPoint;
    Utils::getPPlusAndPMinus(segment, pPlus, pMinus, pPlusIndex, pMinusIndex, centerPoint);

    cv::Point2f firstDerivative = Utils::getFirstDerivative(pPlus, pMinus, pPlusIndex, pMinusIndex);

    // A * x + B * y + C = 0
    std::vector<double> coefficientsOfTheTangent = Utils::getCoefficientsOfTheTangent(centerPoint, firstDerivative);

    // A * x + B * y + C = 0
    std::vector<double> coefficientsOfThePerpendicularLine = Utils::getCoefficientsOfThePerpendicularLine(coefficientsOfTheTangent, centerPoint);

    double A, B, C;
    A = coefficientsOfThePerpendicularLine[0];
    B = coefficientsOfThePerpendicularLine[1];
    C = coefficientsOfThePerpendicularLine[2];

    std::tuple<cv::Point, cv::Point> candidatesForCenter = Utils::calculatingPointsOfStraightLineAtCertainDistanceFromGivenPoint(A, B, C, R, centerPoint);

    center = Utils::chooseAmongTwoCandidatesForCenter(segment, candidatesForCenter);
    //return (begin + end) / 2;
    return center;
}


/**
 * Adding an arc segment to the road model.
 * If a segment has less than three points, then it will be added to the model as a segment
 * @param arcSegment -- vector of the points of the arc segment
 * @param roadModel -- model of the road
 * @param curvature -- curvature of the segment
 */
void addArcToTheModel(const std::vector<cv::Point> &arcSegment, RoadModel &roadModel, double curvature)
{
    // TODO Мб не так делать, а приклеить её к отрезку??
    if (arcSegment.size() < 3) // если у дуги <3 точек, то отрисуем её как отрезок
    {
        if (arcSegment.size() == 1)
        {
            roadModel.addElementToRight(arcSegment[0], arcSegment[0]);
        }
        else if (arcSegment.size() == 2)
        {
            roadModel.addElementToRight(arcSegment[0], arcSegment[1]);
        }
    }
    else
    {
        double radiusOfTheCircle = 1.0 / curvature;
        cv::Point center = getCenterOfTheArc(arcSegment, radiusOfTheCircle);

        double startAngle, endAngle;

        Utils::calculationStartAndEndAnglesOfTheArc(startAngle, endAngle, arcSegment, center, radiusOfTheCircle);

        startAngle = 0;
        endAngle = 360;

        roadModel.addElementToRight(center, radiusOfTheCircle, startAngle, endAngle, arcSegment);
    }
}


RoadModel ExperimentWithCurvatureCalculation::buildRoadModelBasedOnTheSingleContour(const std::vector<cv::Point> &contour, const std::vector<double> &contourCurvature)
{
    /**
     * Построение модели дороги (а точнее нашей полосы движения) по одному контуру.
     * Пока строим ПРАВУЮ часть модели
     */
    RoadModel roadModel;

    cv::Point currElementBegin = contour[0];
    cv::Point currElementEnd = contour[0];

    int currLineSegmentNumber = 0;
    int currArcSegmentNumber = 0;
    double currSumOfArcSegmentCurvatures = 0;
    std::vector<cv::Point> arcSegment;
    std::vector<cv::Point> lineSegment;

    const int minLineSegmentSize = 100;

    const double curvatureThreshold = 0.6; // это порог кривизны. Если кривизна ниже этого порога, то считаем эту часть контура прямой

    double prevCurvature = contourCurvature[0]; // это предыдущее значение, чтобы выделять участки контура с одним и тем же значением кривизны для построения модели
    const double delta = 0.5; // это для дельта-окрестности кривизны (если prevCurvature - delta <= currCurvature < prevCurvature + delta, то currCurvature относится к текущему участку

    for (int i = 0; i < contour.size(); ++i)
    {
        if (std::abs(contourCurvature[i]) == 0) // если это часть прямой
        {
            if (currArcSegmentNumber > 0 && currLineSegmentNumber >= minLineSegmentSize) // если до прямой этого была дуга
            {
                addArcToTheModel(arcSegment, roadModel, currSumOfArcSegmentCurvatures / currArcSegmentNumber);

                arcSegment.clear();

                currArcSegmentNumber = 0;
                currSumOfArcSegmentCurvatures = 0;

                currElementBegin = contour[i];
            }

            if (currLineSegmentNumber == 0)
            {
                currElementBegin = contour[i];
            }

            currLineSegmentNumber++;
            lineSegment.emplace_back(contour[i]);
            currElementEnd = contour[i];
        }
        else // если это дуга окружности
        {
            if (currLineSegmentNumber > 0) // если до дуги шел участок прямой
            {
                if (currLineSegmentNumber <= minLineSegmentSize) // если в сегменте прямой "мало" точек, то добавим его к сегменту дуги
                {
                    for (const auto &point : lineSegment)
                    {
                        arcSegment.emplace_back(point);
                    }
                    lineSegment.clear();
                }
                else
                {
                    roadModel.addElementToRight(currElementBegin, currElementEnd);

                    currLineSegmentNumber = 0;
                    lineSegment.clear();

                    currElementBegin = contour[i];
                    currElementEnd = contour[i];
                    //prevCurvature = contourCurvature[i];
                }
            }

            if (contourCurvature[i] != std::numeric_limits<double>::infinity())
            {
                if (std::abs(contourCurvature[i] - prevCurvature) <= delta) // если продолжается текущий участок
                {
                    currElementEnd = contour[i];

                    arcSegment.emplace_back(contour[i]);

                    currArcSegmentNumber++;
                    currSumOfArcSegmentCurvatures += contourCurvature[i];
                }
                else // если встретился новый участок уровня кривизны
                {
                    addArcToTheModel(arcSegment, roadModel, currSumOfArcSegmentCurvatures / currArcSegmentNumber);

                    arcSegment.clear();

                    currArcSegmentNumber = 1;
                    currSumOfArcSegmentCurvatures = contourCurvature[i];

                    currElementBegin = contour[i];
                    currElementEnd = contour[i];
                    prevCurvature = contourCurvature[i];

                    arcSegment.emplace_back(contour[i]);
                }
            }
        }
    }

    if (currArcSegmentNumber > 0)
    {
        addArcToTheModel(arcSegment, roadModel, currSumOfArcSegmentCurvatures / currArcSegmentNumber);
    }
    else if (currLineSegmentNumber > 0)
    {
        roadModel.addElementToRight(currElementBegin, currElementEnd);
    }

    return roadModel;
}

void ExperimentWithCurvatureCalculation::showCurvatureOnImage(cv::Mat &src, const std::vector<cv::Point> &contour, const std::vector<double> &contourCurvature)
{
    for (int i = 0; i < contour.size(); ++i)
    {
        if (contourCurvature[i] == 0)
        {
            cv::circle(src, contour[i], 1, cv::Scalar(255, 0, 0));
        }
        else
        {
            cv::circle(src, contour[i], 1, cv::Scalar(0, 0, 255));
        }
    }
}



