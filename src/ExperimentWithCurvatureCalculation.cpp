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


void drawArcSegmentAndTangent(cv::Mat &dst, const std::vector<cv::Point> &segment, const std::vector<double> &coefficientsOfTheTangent)
{
    for (const auto &point : segment)
    {
        cv::circle(dst, point, 1, cv::Scalar(0, 0, 255));
    }

    // Ax + By + C = 0
    double A, B, C;
    A = coefficientsOfTheTangent[0];
    B = coefficientsOfTheTangent[1];
    C = coefficientsOfTheTangent[2];

    cv::Point first, second;
    first.x = 0;
    first.y = C / B;

    second.y = 0;
    second.x = -C / A;

    cv::line(dst, first, second, cv::Scalar(255, 0, 0));
}


/**
 * A circle center search function that describes an arc segment.
 * The idea is as follows: there is a tangent line to the arc segment, then its perpendicular line is searched.
 * Then there is a point at a distance R from the point of contact — this point will be the center of the circle.
 * @param segment -- points of the arc segment.
 * @param R -- radius of the circle that describes the segment
 * @return
 */
cv::Point ExperimentWithCurvatureCalculation::getCenterOfTheArc(const std::vector<cv::Point> &segment, double R)
{
    cv::Point center;

    if (segment.size() < 3)
    {
        std::cerr << "cv::Point getCenterOfTheArc -- segment.size() < 3 (=" << segment.size() << ")" << std::endl;
        return center;
    }

    int pMinusIndex, pPlusIndex;
    cv::Point pMinus, pPlus, centerPoint;
    double h;
    Utils::getPPlusAndPMinus(segment, pPlus, pMinus, pPlusIndex, pMinusIndex, centerPoint, h);

    cv::Point2f firstDerivative = Utils::getFirstDerivative(pPlus, pMinus, pPlusIndex, pMinusIndex, h);

    // TODO Возможно, есть более красивый и правильные признак того, что производную надо взять с минусом
    if (pPlus.y > pMinus.y)
    {
        firstDerivative.x = -firstDerivative.x;
    }

    // A * x + B * y + C = 0
    std::vector<double> coefficientsOfTheTangent = Utils::getCoefficientsOfTheTangent(centerPoint, firstDerivative);

    // A' * x + B' * y + C' = 0
    std::vector<double> coefficientsOfThePerpendicularLine = Utils::getCoefficientsOfThePerpendicularLine(coefficientsOfTheTangent, centerPoint);

    double A, B, C;
    A = coefficientsOfThePerpendicularLine[0];
    B = coefficientsOfThePerpendicularLine[1];
    C = coefficientsOfThePerpendicularLine[2];

    // B берётся с минусом из-за того, что в OpenCV координата y растет вниз
    std::tuple<cv::Point, cv::Point> candidatesForCenter = Utils::calculatingPointsOfStraightLineAtCertainDistanceFromGivenPoint(A, -B, C, R, centerPoint);

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
void
ExperimentWithCurvatureCalculation::addArcToTheModel(const std::vector<cv::Point> &arcSegment, RoadModel &roadModel,
                                                     double curvature, bool isRightContour)
{
    double radiusOfTheCircle = 1.0 / curvature;
    cv::Point center = getCenterOfTheArc(arcSegment, radiusOfTheCircle);

    double startAngle, endAngle;

    Utils::calculationStartAndEndAnglesOfTheArc(startAngle, endAngle, arcSegment, center, radiusOfTheCircle);

    if (isRightContour)
    {
        roadModel.addElementToRight(center, radiusOfTheCircle, startAngle, endAngle, arcSegment);
    }
    else
    {
        roadModel.addElementToLeft(center, radiusOfTheCircle, startAngle, endAngle, arcSegment);
    }

}


void ExperimentWithCurvatureCalculation::buildRoadModelBasedOnTheSingleContour(RoadModel &roadModel,
                                                                               const std::vector<cv::Point> &contour,
                                                                               const std::vector<double> &contourCurvature,
                                                                               bool isRightContour)
{
    /**
     * Построение модели дороги (а точнее нашей полосы движения) по одному контуру.
     * Пока строим ПРАВУЮ часть модели
     */

    cv::Point currElementBegin = contour[0];
    cv::Point currElementEnd = contour[0];

    int currLineSegmentNumber = 0;
    int currArcSegmentNumber = 0;
    double currSumOfArcSegmentCurvatures = 0;
    std::vector<cv::Point> arcSegment;
    std::vector<cv::Point> lineSegment;

    const int MIN_LINE_SEGMENT_SIZE = 50;
    const int MIN_ARC_SEGMENT_SIZE = 10;

    const double CURVATURE_THRESHOLD = 0; // это порог кривизны (почему он такой, не знает никто). Если кривизна ниже этого порога, то считаем эту часть контура прямой
    const double DELTA = 0.5; // это для дельта-окрестности кривизны (если prevCurvature - DELTA <= currCurvature < prevCurvature + DELTA, то currCurvature относится к текущему участку

    double prevCurvature = contourCurvature[0]; // это предыдущее значение, чтобы выделять участки контура с одним и тем же значением кривизны для построения модели

    for (int i = 0; i < contour.size(); ++i)
    {
        if (std::abs(contourCurvature[i]) <= CURVATURE_THRESHOLD) // если это часть прямой
        {
            if (currArcSegmentNumber > 0 && currLineSegmentNumber >= MIN_LINE_SEGMENT_SIZE) // если до прямой была дуга
            {
                if (arcSegment.size() < MIN_ARC_SEGMENT_SIZE)
                {
                    for (const auto &point : arcSegment)
                    {
                        lineSegment.emplace_back(point);
                    }
                    currLineSegmentNumber += arcSegment.size();
                }
                else
                {
                    addArcToTheModel(arcSegment, roadModel, currSumOfArcSegmentCurvatures / currArcSegmentNumber, isRightContour);
                }

                arcSegment.clear();

                currArcSegmentNumber = 0;
                currSumOfArcSegmentCurvatures = 0;
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
                if (currLineSegmentNumber <= MIN_LINE_SEGMENT_SIZE) // если в сегменте прямой "мало" точек, то добавим его к сегменту дуги
                {
                    for (const auto &point : lineSegment)
                    {
                        arcSegment.emplace_back(point);
                    }
                    lineSegment.clear();
                    currLineSegmentNumber = 0;
                }
                else if (arcSegment.size() >= MIN_ARC_SEGMENT_SIZE)
                {
                    if (isRightContour)
                    {
                        roadModel.addElementToRight(currElementBegin, currElementEnd);
                    }
                    else
                    {
                        roadModel.addElementToLeft(currElementBegin, currElementEnd);
                    }

                    currLineSegmentNumber = 0;
                    lineSegment.clear();

                    currElementBegin = contour[i];
                    currElementEnd = contour[i];
                    //prevCurvature = contourCurvature[i];
                }
            }

            if (contourCurvature[i] != std::numeric_limits<double>::infinity())
            {
                if (std::abs(contourCurvature[i] - prevCurvature) <= DELTA) // если продолжается текущий участок
                {
                    currElementEnd = contour[i];

                    arcSegment.emplace_back(contour[i]);

                    currArcSegmentNumber++;
                    currSumOfArcSegmentCurvatures += contourCurvature[i];

                    prevCurvature = contourCurvature[i];
                }
                else // если встретился новый участок уровня кривизны
                {
                    if (arcSegment.size() >= MIN_ARC_SEGMENT_SIZE)
                    {
                        addArcToTheModel(arcSegment, roadModel, currSumOfArcSegmentCurvatures / currArcSegmentNumber,
                                         isRightContour);

                        arcSegment.clear();

                        currArcSegmentNumber = 1;
                        currSumOfArcSegmentCurvatures = contourCurvature[i];

                        currElementBegin = contour[i];
                        currElementEnd = contour[i];
                        prevCurvature = contourCurvature[i];
                        arcSegment.emplace_back(contour[i]);
                    }
                    else
                    {
                        //TODO Если до текущего сегмента дуги был другой сегмент дуги, который очень маленький,
                        // то что лучше, прибавить его к текущему сегменту или добавить как участок прямой?
                        // Пока что добавляю его к текущему сегменту
                        currElementEnd = contour[i];

                        arcSegment.emplace_back(contour[i]);

                        currArcSegmentNumber++;
                        currSumOfArcSegmentCurvatures += contourCurvature[i];

                        prevCurvature = contourCurvature[i];
                    }
                }
            }
        }
    }

    if (currArcSegmentNumber > 0)
    {
        addArcToTheModel(arcSegment, roadModel, currSumOfArcSegmentCurvatures / currArcSegmentNumber, isRightContour);
    }
    else if (currLineSegmentNumber > 0)
    {
        if (isRightContour)
        {
            roadModel.addElementToRight(currElementBegin, currElementEnd);
        }
        else
        {
            roadModel.addElementToLeft(currElementBegin, currElementEnd);
        }
    }
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



