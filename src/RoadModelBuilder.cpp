//
// Created by alechh on 21.02.2022.
//

#include <vector>
#include "CurvatureCalculator.h"
#include <opencv2/opencv.hpp>
#include "RoadModelBuilder.h"
#include "RoadModel.h"
#include "Utils.h"
#include "RoadModelTracker.h"


/**
 * A circle center search function that describes an arc segment.
 * The idea is as follows: there is a tangent line to the arc segment, then its perpendicular line is searched.
 * Then there is a point at a distance R from the point of contact — this point will be the center of the circle.
 * @param segment -- points of the arc segment.
 * @param R -- radius of the circle that describes the segment
 * @return
 */
cv::Point RoadModelBuilder::calculateCenterOfTheArc(const std::vector<cv::Point> &segment, double R)
{
    cv::Point center;

    if (segment.size() < 3)
    {
        //std::cerr << "cv::Point calculateCenterOfTheArc -- segment.size() < 3 (=" << segment.size() << ")" << std::endl;
        return center;
    }

    int pMinusIndex, pPlusIndex;
    cv::Point pMinus, pPlus, centerPoint;
    double h;
    calculatePPlusAndPMinus(segment, pPlus, pMinus, pPlusIndex, pMinusIndex, centerPoint, h);

    cv::Point2f firstDerivative = Utils::calculateFirstDerivative(pPlus, pMinus, pPlusIndex, pMinusIndex, h);

    // TODO Возможно, есть более красивый и правильные признак того, что производную надо взять с минусом
    if (pPlus.y > pMinus.y)
    {
        firstDerivative.x = -firstDerivative.x;
    }

    // A * x + B * y + C = 0
    std::vector<double> coefficientsOfTheTangent = Utils::calculateCoefficientsOfTheTangent(centerPoint,
                                                                                            firstDerivative);

    // A' * x + B' * y + C' = 0
    std::vector<double> coefficientsOfThePerpendicularLine = Utils::calculateCoefficientsOfThePerpendicularLine(
            coefficientsOfTheTangent, centerPoint);

    double A, B, C;
    A = coefficientsOfThePerpendicularLine[0];
    B = coefficientsOfThePerpendicularLine[1];
    C = coefficientsOfThePerpendicularLine[2];

    // B берётся с минусом из-за того, что в OpenCV координата y растет вниз
    std::tuple<cv::Point, cv::Point> candidatesForCenter = Utils::calculatingPointsOfStraightLineAtCertainDistanceFromGivenPoint(
            A, -B, C, R, centerPoint);

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
RoadModelBuilder::addArcToTheModel(RoadModelTracker &modelTracker, std::vector<cv::Point> &arcSegment,
                                   double &currSumOfArcSegmentCurvatures,
                                   bool isRightContour)
{
    const double curvature = currSumOfArcSegmentCurvatures / arcSegment.size();

    double radiusOfTheCircle = 1.0 / curvature;
    cv::Point center = calculateCenterOfTheArc(arcSegment, radiusOfTheCircle);

    double startAngle, endAngle;

    calculationStartAndEndAnglesOfTheArc(startAngle, endAngle, arcSegment, center, radiusOfTheCircle);

    if (isRightContour)
    {
        modelTracker.trackRightSide(CircularArc(center, radiusOfTheCircle, startAngle, endAngle, arcSegment));
    }
    else
    {
        modelTracker.trackLeftSide(CircularArc(center, radiusOfTheCircle, startAngle, endAngle, arcSegment));
    }

    arcSegment.clear();
    currSumOfArcSegmentCurvatures = 0;

    cv::Mat drawing(800, 1500, 16, cv::Scalar(0, 0, 0));
    modelTracker.roadModel->drawModelPoints(drawing);
    cv::imshow("drawing", drawing);
    cv::waitKey(0);
}


void RoadModelBuilder::buildRoadModelBasedOnTheSingleContour(RoadModelTracker &modelTracker,
                                                             const std::vector<cv::Point> &contour,
                                                             const std::vector<double> &contourCurvature,
                                                             bool isRightContour)
{
    /**
     * Построение модели дороги (а точнее нашей полосы движения) по одному контуру.
     */

    double currSumOfArcSegmentCurvatures = 0;
    std::vector<cv::Point> arcSegment;
    std::vector<cv::Point> lineSegment;

    const int MIN_LINE_SEGMENT_SIZE = 30; // 50
    const int MIN_ARC_SEGMENT_SIZE = 30;

    const double CURVATURE_THRESHOLD = 0.005; // это порог кривизны (почему он такой, не знает никто). Если кривизна <= этого порога, то считаем эту часть контура прямой
    const double CURVATURE_DELTA = 100; // 0.1 это для дельта-окрестности кривизны (если |prevCurvature - currCurvature| <= CURVATURE_DELTA, то currCurvature относится к текущему участку

    double prevCurvature = contourCurvature[0]; // это предыдущее значение, чтобы выделять участки контура с одним и тем же значением кривизны для построения модели
    cv::Point prevContourPoint = contour[0];
    const int CONTOUR_POINTS_DELTA = 100;

    cv::Mat drawing(800, 1500, 16, cv::Scalar(0, 0, 0));

    for (int i = 0; i < contour.size(); ++i)
    {
        // если встретилась точка контура, которая далеко от предыдущей, то это точно начался другой сегмент
        if (std::abs(contour[i].x - prevContourPoint.x) > CONTOUR_POINTS_DELTA ||
            std::abs(contour[i].y - prevContourPoint.y) > CONTOUR_POINTS_DELTA)
        {
            if (!arcSegment.empty())
            {
                if (arcSegment.size() < MIN_ARC_SEGMENT_SIZE && !lineSegment.empty())
                {
                    addArcSegmentPointsToLineSegment(arcSegment, lineSegment, currSumOfArcSegmentCurvatures);
                }
                else
                {
                    addArcToTheModel(modelTracker, arcSegment, currSumOfArcSegmentCurvatures, isRightContour);
                }
            }

            if (!lineSegment.empty())
            {
                addLineSegmentToModel(modelTracker, lineSegment, isRightContour);
            }
        }

        if (contourCurvature[i] <= CURVATURE_THRESHOLD)
        {
            cv::circle(drawing, contour[i], 1, cv::Scalar(0, 0, 255));
        }
        else
        {
            cv::circle(drawing, contour[i], 1, cv::Scalar(255, 0, 0));
        }
        cv::imshow("contour", drawing);
        cv::waitKey(25);

        if (contourCurvature[i] <= CURVATURE_THRESHOLD) // если это часть прямой
        {
            // если встретилась ситуация, что сегмент прямой и дуги до этого были маленькие,
            // то есть они никуда не добавились
            if (!lineSegment.empty() && !arcSegment.empty() && prevCurvature > CURVATURE_THRESHOLD)
            {
                // если сегмент дуги достаточно большой, то добавим в модель предыдущие сегменты прямой и дуги
                if (arcSegment.size() > MIN_ARC_SEGMENT_SIZE)
                {
                    addLineSegmentToModel(modelTracker, lineSegment, isRightContour);
                    addArcToTheModel(modelTracker, arcSegment, currSumOfArcSegmentCurvatures, isRightContour);
                }
                else
                {
                    addArcSegmentPointsToLineSegment(arcSegment, lineSegment, currSumOfArcSegmentCurvatures);
                }
            }

            // если до прямой была дуга и текущий сегмент прямой уже достаточно большой
            if (!arcSegment.empty() && lineSegment.size() >= MIN_LINE_SEGMENT_SIZE)
            {
                if (arcSegment.size() < MIN_ARC_SEGMENT_SIZE) // если сегмент дуги маленький, то добавляем его в прямой
                {
                    addArcSegmentPointsToLineSegment(arcSegment, lineSegment, currSumOfArcSegmentCurvatures);
                }
                    // если сегмент дуги достаточно большой, то добавляем его в модель
                else
                {
                    addArcToTheModel(modelTracker, arcSegment, currSumOfArcSegmentCurvatures,
                                     isRightContour);
                }
            }
            lineSegment.emplace_back(contour[i]);
        }
        else // если это дуга окружности
        {
            // если встретилась ситуация, что сегмент дуги и прямой до этого были маленькие,
            // то есть они никуда не добавились, тогда считаем их одним сегментом прямой
            if (!lineSegment.empty() && !arcSegment.empty() && prevCurvature <= CURVATURE_THRESHOLD)
            {
                if (arcSegment.size() < MIN_ARC_SEGMENT_SIZE)
                {
                    if (lineSegment.size() > MIN_LINE_SEGMENT_SIZE)
                    {
                        addArcSegmentPointsToLineSegment(arcSegment, lineSegment, currSumOfArcSegmentCurvatures);
                    }
                    else
                    {
                        addLineSegmentPointsToArcSegment(lineSegment, arcSegment, currSumOfArcSegmentCurvatures);
                    }
                }
                else
                {
                    if (lineSegment.size() > MIN_LINE_SEGMENT_SIZE)
                    {
                        addArcToTheModel(modelTracker, arcSegment, currSumOfArcSegmentCurvatures, isRightContour);
                        addLineSegmentToModel(modelTracker, lineSegment, isRightContour);
                    }
                    else
                    {
                        addLineSegmentPointsToArcSegment(lineSegment, arcSegment, currSumOfArcSegmentCurvatures);
                    }
                }
            }

            if (!lineSegment.empty()) // если до дуги шел участок прямой
            {
                // если в сегменте прямой "мало" точек, то добавим его к сегменту дуги
                if (lineSegment.size() <= MIN_LINE_SEGMENT_SIZE)
                {
                    addLineSegmentPointsToArcSegment(lineSegment, arcSegment, currSumOfArcSegmentCurvatures);
                }
                    // если в сегменте прямой достаточно точек и сегмент дуги уже достаточно большой,
                    // то добавляем сегмент прямой к модели
                else if (arcSegment.size() >= MIN_ARC_SEGMENT_SIZE)
                {
                    addLineSegmentToModel(modelTracker, lineSegment, isRightContour);
                }
            }

            if (contourCurvature[i] != std::numeric_limits<double>::infinity())
            {
                // если продолжается текущий участок
                if (std::abs(contourCurvature[i] - prevCurvature) <= CURVATURE_DELTA)
                {
                    arcSegment.emplace_back(contour[i]);
                    currSumOfArcSegmentCurvatures += contourCurvature[i];
                }
//                else // если встретился новый участок уровня кривизны
//                {
//                    if (arcSegment.size() >= MIN_ARC_SEGMENT_SIZE)
//                    {
//                        addArcToTheModel(modelTracker, arcSegment, currSumOfArcSegmentCurvatures,
//                                         isRightContour);
//
//                        arcSegment.emplace_back(contour[i]);
//                        currSumOfArcSegmentCurvatures = contourCurvature[i];
//                    }
//                    else
//                    {
//                        //TODO Если до текущего сегмента дуги был другой сегмент дуги, который очень маленький
//
//                        // Вариант 1: добавляю его к текущему сегменту дуги
////                        arcSegment.emplace_back(contour[i]);
////                        currSumOfArcSegmentCurvatures += contourCurvature[i];
//
//                        // Вариант 2: добавляю его как прямой отрезок
//                        addLineSegmentToModel(modelTracker, arcSegment, isRightContour);
//
//                        arcSegment.emplace_back(contour[i]);
//                        currSumOfArcSegmentCurvatures = contourCurvature[i];
//                    }
//                }
            }
        }
        prevCurvature = contourCurvature[i];
        prevContourPoint = contour[i];
    }

    if (!arcSegment.empty())
    {
        addArcToTheModel(modelTracker, arcSegment, currSumOfArcSegmentCurvatures,
                         isRightContour);
    }
    else if (!lineSegment.empty())
    {
        addLineSegmentToModel(modelTracker, lineSegment, isRightContour);

    }
}


/**
 * Function for calculating the angle shift if the arc segment is above the arc center.
 * This is necessary for the correct determination of the initial and final angle of the arc.
 * @param firstPointOfTheSegment -- first point of the arc segment
 * @param circleCenter -- center of the circle that describes the arc
 * @return
 */
double
RoadModelBuilder::calculateAngleShiftUpper(const cv::Point &firstPointOfTheSegment, const cv::Point &circleCenter)
{
    cv::Point T(firstPointOfTheSegment.x,
                circleCenter.y); // проекция первой точки сегмента на горизонтальную прямую, проходящую через середину окружности
    double distAC = Utils::distanceBetweenPoints(firstPointOfTheSegment, circleCenter);
    double distAT = Utils::distanceBetweenPoints(firstPointOfTheSegment, T);

    if (distAT > distAC)
    {
        return 0;
    }

    double angleC = asin(distAT / distAC) * 180 / CV_PI; // sin C = distAT / distAC (противолежащий катет / гипотенуза)

    return angleC;
}

/**
 * Function for calculating the angle shift if the arc segment is below the arc center.
 * This is necessary for the correct determination of the initial and final angle of the arc.
 * @param firstPointOfTheSegment -- first point of the arc segment
 * @param circleCenter -- center of the circle that describes the arc
 * @return
 */
double RoadModelBuilder::calculateAngleShiftLower(const cv::Point &lastPointOfTheSegment, const cv::Point &circleCenter,
                                                  double radiusOfTheCircle)
{
    double shift;
    cv::Point P(circleCenter.x + radiusOfTheCircle, circleCenter.y);

    cv::Point middlePoint((lastPointOfTheSegment.x + P.x) / 2, (lastPointOfTheSegment.y + P.y) / 2);

    double distCMiddlePoint = Utils::distanceBetweenPoints(circleCenter, middlePoint);

    double angleLastPoint = asin(distCMiddlePoint / radiusOfTheCircle) * 180 / CV_PI;

    return 180 - 2 * angleLastPoint;
}

/**
 * A function for calculating the start and end angles for an arc that describes a segment.
 * @param startAngle
 * @param endAngle
 * @param segment
 * @param center
 * @param radiusOfTheCircle
 * @return
 */
void RoadModelBuilder::calculationStartAndEndAnglesOfTheArc(double &startAngle, double &endAngle,
                                                            const std::vector<cv::Point> &segment,
                                                            const cv::Point &center, double radiusOfTheCircle)
{
    double angleC = calculateAngleOfTheArc(segment, center, radiusOfTheCircle);

    cv::Point middleSegmentPoint = segment[segment.size() / 2];

    if (middleSegmentPoint.y < center.y) // если сегмент находится в верхней части плоскости
    {
        double shiftAngle = RoadModelBuilder::calculateAngleShiftUpper(segment[0], center);

        startAngle = 180 + shiftAngle;
    }
    else // если сегмент находится в нижней части плоскости
    {
        int indexOfLastArcSegment = segment.size() - 1;
        cv::Point lastSegmentPoint = segment[indexOfLastArcSegment];

        double distR = Utils::distanceBetweenPoints(lastSegmentPoint, center);

        double shiftAngle = calculateAngleShiftLower(lastSegmentPoint, center, radiusOfTheCircle);

        startAngle = shiftAngle;
    }

    endAngle = startAngle + angleC;
}

/**
 * Calculating the angle C in triangle ABC, where
 * A is the first point of the segment,
 * B is the last point of the segment,
 * C is the center of the circle that describes this segment
 * @param segment -- vector of points of the current segment
 * @param center -- center of the circle that describes the segment
 * @param radiusOfTheCircle -- radius of the circle
 * @return
 */
double RoadModelBuilder::calculateAngleOfTheArc(const std::vector<cv::Point> &segment, const cv::Point &center,
                                                double radiusOfTheCircle)
{
    cv::Point M = Utils::calculateMidpoint(segment[0], segment[segment.size() - 1]);

    double distMCenter = Utils::distanceBetweenPoints(M, center);

    if (distMCenter > radiusOfTheCircle)
    {
        // the hypotenuse should be larger than the catheter
        return 0;
    }

    double angleB = asin(distMCenter / radiusOfTheCircle);

    //converting to the degree
    angleB *= 180 / CV_PI;

    double angleC = 180 - 2 * angleB;

    return angleC;
}

/**
 * Get points pPlus, pMinus and its indices in segment.
 * These points and indices are used to calculate the derivative at the point that is between pPlus and pMinus.
 * @param segment -- vector of the points of the segment
 * @param pPlus -- next point from the center
 * @param pMinus -- previous point to the center
 * @param pPlusIndex -- index of the pPlus point
 * @param pMinusIndex -- index of the pMinus point
 * @param h -- distance between pMinus and center point of the segment (= distance between pPlus and center point)
 */
void
RoadModelBuilder::calculatePPlusAndPMinus(const std::vector<cv::Point> &segment, cv::Point &pPlus, cv::Point &pMinus,
                                          int &pPlusIndex,
                                          int &pMinusIndex, cv::Point &centerPoint, double &h)
{
    int indexOfTheCenter = segment.size() / 2;

    centerPoint = segment[indexOfTheCenter];

    pMinusIndex = indexOfTheCenter - 1;
    pPlusIndex = indexOfTheCenter + 1;

    pMinus = segment[pMinusIndex];
    pPlus = segment[pPlusIndex];

    // Нужно, чтобы точки были различными
    double distPMinusCenter = abs(pMinus.x - centerPoint.x);
    double distPPlusCenter = abs(pPlus.x - centerPoint.x);

    const int MAX_DELTA = 10; // если точки "разбегаются" дальше, чем на MAX_DELTA, то опускаем глаза на то, что они должны быть равноотстоящими, оставим их просто различными
    int currDelta = 1;

    while ((distPMinusCenter != distPPlusCenter || pMinus.x == pPlus.x) && currDelta < MAX_DELTA)
    {
        if (pMinusIndex == 0 && pPlusIndex == segment.size() - 1)
        {
            break;
        }

        if (distPMinusCenter < distPPlusCenter)
        {
            --pMinusIndex;
            pMinus = segment[pMinusIndex];
            distPMinusCenter = abs(pMinus.x - centerPoint.x);
            ++currDelta;
        }
        else
        {
            ++pPlusIndex;
            pPlus = segment[pPlusIndex];
            distPPlusCenter = abs(pPlus.x - centerPoint.x);
            ++currDelta;
        }
    }

    if (currDelta == MAX_DELTA) // это значит, что не удалось найти равноотстоящие точки на расстоянии MAX_DELTA
    {
        pMinusIndex = indexOfTheCenter - 1;
        pPlusIndex = indexOfTheCenter + 1;

        pMinus = segment[pMinusIndex];
        pPlus = segment[pPlusIndex];

        bool hasPMinusChanged = false;
        while (pPlus == pMinus)
        {
            if (!hasPMinusChanged)
            {
                --pMinusIndex;
                pMinus = segment[pMinusIndex];
                hasPMinusChanged = true;
            }
            else
            {
                ++pPlusIndex;
                pPlus = segment[pPlusIndex];
                hasPMinusChanged = false;
            }
        }
    }

    h = 2 * abs(pPlus.x - centerPoint.x);
}

void
RoadModelBuilder::addLineSegmentToModel(RoadModelTracker &modelTracker, std::vector<cv::Point> &lineSegment,
                                        bool isRightContour)
{
    std::cout << "lineSegment.size() = " << lineSegment.size() << std::endl;
    if (isRightContour)
    {
        modelTracker.trackRightSide(LineSegment(lineSegment[0], lineSegment[lineSegment.size() - 1]));
    }
    else
    {
        modelTracker.trackLeftSide(LineSegment(lineSegment[0], lineSegment[lineSegment.size() - 1]));
    }

    lineSegment.clear();

    cv::Mat drawing(800, 1500, 16, cv::Scalar(0, 0, 0));
    modelTracker.roadModel->drawModelPoints(drawing);
    cv::imshow("drawing", drawing);
    cv::waitKey(0);
}

void
RoadModelBuilder::buildRoadModel(RoadModelTracker &modelTracker, const std::vector<std::vector<cv::Point>> &contours,
                                 const std::vector<std::vector<double>> &contoursCurvatures, const int COLS)
{
    for (int contourNumber = 0; contourNumber < contours.size(); ++contourNumber)
    {
        bool isRightContour;
        if (contours[contourNumber][0].x < COLS / 2)
        {
            isRightContour = false;
        }
        else
        {
            isRightContour = true;
        }

        RoadModelBuilder::buildRoadModelBasedOnTheSingleContour(modelTracker, contours[contourNumber],
                                                                contoursCurvatures[contourNumber],
                                                                isRightContour);
    }

    // when all contours are processed, tell tracker that road model has been built, then tracker starts track it
    modelTracker.modelHasBeenConstructed();
}

void RoadModelBuilder::addLineSegmentPointsToArcSegment(std::vector<cv::Point> &lineSegment,
                                                        std::vector<cv::Point> &arcSegment,
                                                        double &currSumOfArcSegmentCurvatures)
{
    if (!arcSegment.empty())
    {
        const double avgCurvature = currSumOfArcSegmentCurvatures / arcSegment.size();
        currSumOfArcSegmentCurvatures += avgCurvature * lineSegment.size();
    }

    for (const auto &lineSegmentPoint: lineSegment)
    {
        arcSegment.emplace_back(lineSegmentPoint);
    }
    lineSegment.clear();
}

void RoadModelBuilder::addArcSegmentPointsToLineSegment(std::vector<cv::Point> &arcSegment,
                                                        std::vector<cv::Point> &lineSegment,
                                                        double &currSumOfArcSegmentCurvatures)
{
    for (const auto &arcSegmentPoint: arcSegment)
    {
        lineSegment.emplace_back(arcSegmentPoint);
    }
    arcSegment.clear();
    currSumOfArcSegmentCurvatures = 0;
}

