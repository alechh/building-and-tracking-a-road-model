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

void drawArcSegment(const std::vector<cv::Point> &arcSegment, const cv::Point &center)
{
    cv::Mat segmentPicture(800, 1500, 16, cv::Scalar(0, 0, 0));
    for (const auto &point: arcSegment)
    {
        cv::circle(segmentPicture, point, 1, cv::Scalar(255, 0, 0));
    }

    cv::circle(segmentPicture, center, 1, cv::Scalar(255, 255, 0));

    cv::imshow("arcSegment", segmentPicture);
    cv::waitKey(0);
    cv::destroyWindow("arcSegment");
}

/**
 * Calculating the center of the circle in the following way:
 * 1. Calculation of the the midpoints of the sides of the triangle
 * 2. Calculation of the coefficients of the mid-perpendiculars
 * 3. Calculating the point of their intersection
 * @param segment
 * @param radius
 * @return
 */
cv::Point calculateCenterOfTheArc2(const std::vector<cv::Point> &segment, double radius)
{
    const cv::Point &p1 = segment[0];
    const cv::Point &p2 = segment[segment.size() - 1];
    const cv::Point &middleSegment = segment[segment.size() / 2];

    //drawArcSegment(segment, middleSegment);

    std::vector<double> perpendicular1(3), perpendicular2(3);

    // уравнение прямой через 2 точки p1 и middlePoint
    std::vector<double> sideOfTriangle1(3);
    sideOfTriangle1[0] = middleSegment.y - p1.y;
    sideOfTriangle1[1] = -(-middleSegment.x + p1.x);
    sideOfTriangle1[2] = p1.y * (middleSegment.x - p1.x) - p1.x * (middleSegment.y - p1.y);

    // уравнение прямой через 2 точки p2 и middlePoint
    std::vector<double> sideOfTriangle2(3);
    sideOfTriangle2[0] = middleSegment.y - p2.y;
    sideOfTriangle2[1] = -(-middleSegment.x + p2.x);
    sideOfTriangle2[2] = p2.y * (middleSegment.x - p2.x) - p2.x * (middleSegment.y - p2.y);

    // серединные точки на сторонах треугольника
    cv::Point middle1 = Utils::calculateMidpoint(p1, middleSegment);
    cv::Point middle2 = Utils::calculateMidpoint(p2, middleSegment);

    // серединные перпендикуляры
    perpendicular1 = Utils::calculateCoefficientsOfThePerpendicularLine(sideOfTriangle1, middle1);
    perpendicular2 = Utils::calculateCoefficientsOfThePerpendicularLine(sideOfTriangle2, middle2);

    // делим на коэффициент при y
    perpendicular1[0] /= perpendicular1[1];
    perpendicular1[2] /= perpendicular1[1];
    perpendicular1[1] = 1;

    // делим на коэффициент при y
    perpendicular2[0] /= perpendicular2[1];
    perpendicular2[2] /= perpendicular2[1];
    perpendicular2[1] = 1;

    // точка пересечения двух прямых perpendicular1 и perpendicular2
    double x = (perpendicular2[2] - perpendicular1[2]) / (perpendicular1[0] - perpendicular2[0]);
    double y = -(-perpendicular1[0] * x - perpendicular1[2]);

    return cv::Point(x, y);
}


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
    if (segment.size() < 3)
    {
        //std::cerr << "cv::Point calculateCenterOfTheArc -- segment.size() < 3 (=" << segment.size() << ")" << std::endl;
        return cv::Point();
    }

    int pMinusIndex, pPlusIndex;
    cv::Point pMinus, pPlus, centerPoint;
    double h;
    calculatePPlusAndPMinus(segment, pPlus, pMinus, pPlusIndex, pMinusIndex, centerPoint, h);

    //drawArcSegment(segment, centerPoint, pPlus, pMinus);

    cv::Point2f firstDerivative = Utils::calculateFirstDerivative(pPlus, pMinus, pPlusIndex, pMinusIndex, h);

    // Возможно, есть более красивый и правильный признак того, что производную надо взять с минусом
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

    cv::Point center = Utils::chooseAmongTwoCandidatesForCenter(segment, candidatesForCenter);
    //return (begin + end) / 2;
    return center;
}

void drawArcSegment(const std::vector<cv::Point> &arcSegment, const cv::Point &endPoint, const cv::Point &center)
{
    cv::Mat segmentPicture(800, 1500, 16, cv::Scalar(0, 0, 0));
    for (const auto &point: arcSegment)
    {
        if (point == arcSegment[0] || point == endPoint)
        {
            cv::circle(segmentPicture, point, 1, cv::Scalar(0, 255, 0));
        }
        else
        {
            cv::circle(segmentPicture, point, 1, cv::Scalar(255, 0, 0));
        }
    }

    cv::circle(segmentPicture, center, 1, cv::Scalar(255, 255, 0));

    cv::imshow("segmentPicture", segmentPicture);
    cv::waitKey(0);
}


/**
 * Adding an arc segment to the road model.
 * If a segment has less than three points, then it will be added to the model as a segment
 * @param arcSegment -- vector of the points of the arc segment
 * @param roadModel -- model of the road
 * @param curvature -- curvature of the segment
 */
bool
RoadModelBuilder::addArcToTheModel(RoadModelTracker &modelTracker, std::vector<cv::Point> &arcSegment,
                                   double &currSumOfArcSegmentCurvatures,
                                   bool isRightContour)
{
//    std::cout << "ARC SEGMENT:" << std::endl;
//    for (int i = 0; i < arcSegment.size() - 3; ++i)
//    {
//        cv::Mat drawing(800, 1500, 16, cv::Scalar(0, 0, 0));
//        for (int j = 0; j < arcSegment.size() - 1; ++j)
//        {
//            if (arcSegment[j] == arcSegment[i - 3] || arcSegment[j] == arcSegment[i] ||
//                arcSegment[j] == arcSegment[i + 3])
//            {
//                cv::circle(drawing, arcSegment[j], 2, cv::Scalar(0, 255, 0));
//            }
//            else
//            {
//                cv::circle(drawing, arcSegment[j], 2, cv::Scalar(255, 0, 0));
//            }
//        }
//
//        if (i > 2)
//        {
//            checkingChangeOfContourDirection2(arcSegment[i - 3], arcSegment[i], arcSegment[i + 3]);
//        }
//
//        cv::imshow("angle", drawing);
//        if (cv::waitKey(0) == 107)
//        {
//            cv::destroyWindow("angle");
//            break;
//        }
//        cv::destroyWindow("angle");
//    }
//    std::cout << "END ARC SEGMENT" << std::endl;
//
//    cv::Point &p1 = arcSegment[0];
//    cv::Point &p2 = arcSegment[arcSegment.size() - 1];
//    cv::Point &middle = arcSegment[arcSegment.size() / 2];
//    if (Utils::distanceBetweenPoints(p1, p2) < Utils::distanceBetweenPoints(p1, middle))
//    {
//        std::cout << "this arc segment is bad" << std::endl;
//
//        std::vector<cv::Point> newArcSegment;
//        newArcSegment.emplace_back(arcSegment[0]);
//        double currDistanceToFirstPoint = 0;
//
//        for (int i = 1; i < arcSegment.size(); ++i)
//        {
//            if (Utils::distanceBetweenPoints(newArcSegment[0], arcSegment[i]) > currDistanceToFirstPoint)
//            {
//                newArcSegment.emplace_back(arcSegment[i]);
//                currDistanceToFirstPoint = Utils::distanceBetweenPoints(newArcSegment[0], arcSegment[i]);
//            }
//            else
//            {
//                break;
//            }
//        }
//
//        arcSegment = std::move(newArcSegment);
//    }

    //const double curvature = currSumOfArcSegmentCurvatures / arcSegment.size();
    //double radiusOfTheCircle = 1.0 / curvature;

    const double radiusOfTheCircle = calculateRadiusOfTheArcUsingContour(arcSegment);

    if (std::isnan(radiusOfTheCircle) || radiusOfTheCircle == std::numeric_limits<double>::infinity() ||
        radiusOfTheCircle == 0)
    {
        return false;
    }

    /** TODO
     * Если радиус слишком большой для маленького сегмента, это значит, что такой сегмент на самом деле очень похож
     * на сегмент прямой
     */
    const int MAXIMUM_RATIO_OF_RADIUS_TO_LENGTH = 5;
    double arcSegmentLength = Utils::distanceBetweenPoints(arcSegment[0], arcSegment[arcSegment.size() - 1]);
    if (radiusOfTheCircle / arcSegmentLength > MAXIMUM_RATIO_OF_RADIUS_TO_LENGTH)
    {
        return false;
    }

    //const cv::Point center = calculateCenterOfTheArc(arcSegment, radiusOfTheCircle);
    const cv::Point center = calculateCenterOfTheArc2(arcSegment, radiusOfTheCircle);

    //drawArcSegment(arcSegment, center);

    if (checkIfArcSegmentIsStraight(arcSegment))
    {
        return false;
    }

    double startAngle, endAngle;

    calculationStartAndEndAnglesOfTheArc(startAngle, endAngle, arcSegment, center, radiusOfTheCircle);

    //calculationStartAndEndAnglesOfTheArc2(startAngle, endAngle, arcSegment, center, radiusOfTheCircle);

    if (std::isnan(startAngle) || startAngle == std::numeric_limits<double>::infinity())
    {
        return false;
    }

    if (std::isnan(endAngle) || endAngle == std::numeric_limits<double>::infinity())
    {
        return false;
    }

    const int MAX_VALUE = 5000;
    if (std::abs(center.x) > MAX_VALUE || std::abs(center.y) > MAX_VALUE || radiusOfTheCircle > MAX_VALUE)
    {
        return false;
    }

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
    modelTracker.getRoadModelPointer()->drawModel(drawing);
    //std::cout << "arcSegment added" << std::endl;
//    cv::imshow("drawing", drawing);
//    cv::waitKey(0);

    return true;
}


void drawContourPoints(cv::Mat &drawing, const cv::Point &point, double curvature, const double CURVATURE_THRESHOLD)
{
    if (curvature <= CURVATURE_THRESHOLD)
    {
        cv::circle(drawing, point, 1, cv::Scalar(0, 0, 255));
    }
    else
    {
        cv::circle(drawing, point, 1, cv::Scalar(255, 0, 0));
    }
//    cv::imshow("contour", drawing);
//    cv::waitKey(25);
}


void RoadModelBuilder::buildRoadModelBasedOnTheSingleContour(RoadModelTracker &modelTracker,
                                                             const std::vector<cv::Point> &contour,
                                                             const std::vector<double> &contourCurvature,
                                                             bool isRightContour,
                                                             double MULTIPLIER_OF_NUMBER_OF_CONTOUR_POINTS = 1)
{
    /**
     * Построение модели дороги (а точнее нашей полосы движения) по одному контуру.
     */

    double currSumOfArcSegmentCurvatures = 0;
    std::vector<cv::Point> arcSegment;
    std::vector<cv::Point> lineSegment;

    const int MIN_LINE_SEGMENT_SIZE = 30; // 50
    const int MIN_ARC_SEGMENT_SIZE = 10; // 20

    const double CURVATURE_THRESHOLD = 0.002; // Это порог кривизны (почему он такой, не знает никто). Если кривизна <= этого порога, то считаем эту часть контура прямой

    cv::Mat drawing(800, 1500, 16, cv::Scalar(0, 0, 0));

    // Всегда первую точку контура будем считать прямой
    lineSegment.emplace_back(contour[0]);
    double prevCurvature = 0;

    for (int i = 1; i < contour.size() * MULTIPLIER_OF_NUMBER_OF_CONTOUR_POINTS; ++i)
    {
        //std::cout << "i = " << i << "\tpoint = " << contour[i] << std::endl;

        //FIXME
        if (i > 2 && checkingChangeOfContourDirection2(contour[i - 3], contour[i], contour[i + 3]))
        {
            addArcAndLineSegmentsToModel(modelTracker, arcSegment, currSumOfArcSegmentCurvatures, MIN_ARC_SEGMENT_SIZE,
                                         lineSegment, isRightContour);


            setValuesForFirstPointOfTheContour(lineSegment, prevCurvature, contour[i]);
            continue;
        }

        // если встретилась точка контура, которая далеко от предыдущей, то это точно начался другой сегмент
        if (i > 0 && checkingForStartOfAnotherContour(contour[i - 1], contour[i]))
        {
            addArcAndLineSegmentsToModel(modelTracker, arcSegment, currSumOfArcSegmentCurvatures, MIN_ARC_SEGMENT_SIZE,
                                         lineSegment, isRightContour);


            setValuesForFirstPointOfTheContour(lineSegment, prevCurvature, contour[i]);
            continue;
        }

        drawContourPoints(drawing, contour[i], contourCurvature[i], CURVATURE_THRESHOLD);

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
                    if (!addArcToTheModel(modelTracker, arcSegment, currSumOfArcSegmentCurvatures, isRightContour))
                    {
                        addLineSegmentToModel(modelTracker, arcSegment, isRightContour);
                        currSumOfArcSegmentCurvatures = 0;
                    }
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
                    if (!addArcToTheModel(modelTracker, arcSegment, currSumOfArcSegmentCurvatures, isRightContour))
                    {
                        addLineSegmentToModel(modelTracker, arcSegment, isRightContour);
                        currSumOfArcSegmentCurvatures = 0;
                    }
                }
            }
            lineSegment.emplace_back(contour[i]);
        }
        else // если это дуга окружности
        {
            if (lineSegment.size() > MIN_LINE_SEGMENT_SIZE && arcSegment.size() > MIN_ARC_SEGMENT_SIZE)
            {
                addLineSegmentToModel(modelTracker, lineSegment, isRightContour);
            }

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
                        if (!addArcToTheModel(modelTracker, arcSegment, currSumOfArcSegmentCurvatures, isRightContour))
                        {
                            addLineSegmentToModel(modelTracker, arcSegment, isRightContour);
                            currSumOfArcSegmentCurvatures = 0;
                        }
                        addLineSegmentToModel(modelTracker, lineSegment, isRightContour);
                    }
                    else
                    {
                        addLineSegmentPointsToArcSegment(lineSegment, arcSegment, currSumOfArcSegmentCurvatures);
                    }
                }
            }
            if (contourCurvature[i] != std::numeric_limits<double>::infinity())
            {
                arcSegment.emplace_back(contour[i]);
                currSumOfArcSegmentCurvatures += contourCurvature[i];
            }
        }
        prevCurvature = contourCurvature[i];
    }

    if (!arcSegment.empty())
    {
        if (!addArcToTheModel(modelTracker, arcSegment, currSumOfArcSegmentCurvatures, isRightContour))
        {
            addLineSegmentToModel(modelTracker, arcSegment, isRightContour);
            currSumOfArcSegmentCurvatures = 0;
        }
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
        std::cerr << "RoadModelBuilder::calculateAngleShiftUpper: sin > 1" << std::endl;
        return 0;
    }

    double angleC = asin(distAT / distAC) * 180 / CV_PI; // sin C = distAT / distAC (противолежащий катет / гипотенуза)

    if (firstPointOfTheSegment.x > circleCenter.x)
    {
        angleC = 180 - angleC;
    }

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

//    if (distCMiddlePoint > radiusOfTheCircle)
//    {
//        std::cerr << "RoadModelBuilder::calculateAngleShiftLower: sin > 1!!" << std::endl;
//    }
//    double angleLastPoint = asin(distCMiddlePoint / radiusOfTheCircle) * 180 / CV_PI;
//    return 180 - 2 * angleLastPoint;

    double angleLastPoint = Utils::calculateAngleOfTriangle(lastPointOfTheSegment, circleCenter, P);
    return angleLastPoint;

}


cv::Point getLastArcPoint(const std::vector<cv::Point> &arcSegment, const double radius, const cv::Point &center)
{
    const double DISTANCE_DELTA = 3;
    cv::Point lastPoint = arcSegment[arcSegment.size() - 1];

    double currDistance = Utils::distanceBetweenPoints(lastPoint, center);
    if (currDistance <= radius)
    {
        return lastPoint;
    }

    cv::Point directionalVector(center.x - lastPoint.x, -(center.y - lastPoint.y));

    double currDistanceError = std::abs(Utils::distanceBetweenPoints(lastPoint, center) - radius);
    //std::cout << "currDistanceError (lastPoint) = " << currDistanceError << std::endl;

    double t = 0;

    //TODO check it
    // Нахожу параметр, чтобы найти точку на направляющей прямой, которая отдалена от центра на расстояние радиуса
    // Это раскрытие модуля |t-a| < 1
    t = 1 - (radius - sqrt(directionalVector.x * directionalVector.x + directionalVector.y * directionalVector.y));
    cv::Point newPoint(directionalVector.x * t + lastPoint.x, directionalVector.y * t + lastPoint.y);

    currDistanceError = std::abs(Utils::distanceBetweenPoints(newPoint, center) - radius);
    //std::cout << "currDistanceError (newPoint) = " << currDistanceError << std::endl;

//    cv::Point newPoint = lastPoint;
//    while (currDistanceError > DISTANCE_DELTA)
//    {
//        t += 0.01;
//
//        newPoint.x = directionalVector.x * t + lastPoint.x;
//        newPoint.y = directionalVector.y * t + lastPoint.y;
//        currDistanceError = std::abs(Utils::distanceBetweenPoints(newPoint, center) - radius);
//
//        std::cout << "currDistanceError = " << currDistanceError << std::endl;
//    }

//    std::vector<cv::Point> newArcSegment = std::move(arcSegment);
//    newArcSegment.emplace_back(newPoint);

    return newPoint;
    //drawArcSegment(newArcSegment, newArcSegment[newArcSegment.size() - 1], center);
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
        // TODO
        cv::Point firstPointOfTheArc;
        if (/* segment[0].y > segment[segment.size() - 1].y && */
            (segment[0].y > center.y || segment[0].x < segment[segment.size() - 1].x))
        {
            firstPointOfTheArc = segment[0];
        }
        else
        {
            firstPointOfTheArc = segment[segment.size() - 1];
        }
        //double shiftAngle = RoadModelBuilder::calculateAngleShiftUpper(segment[0], center);
        double shiftAngle = RoadModelBuilder::calculateAngleShiftUpper(firstPointOfTheArc, center);

        startAngle = 180 + shiftAngle;
    }
    else // если сегмент находится в нижней части плоскости
    {
        int indexOfLastArcSegment = segment.size() - 1;
        const int DELTA = 5;

        //TODO
        cv::Point lastSegmentPoint;
        //cv::Point lastSegmentPoint = segment[indexOfLastArcSegment];
        //cv::Point lastSegmentPoint = segment[0];
        if (segment[0].x > segment[segment.size() - 1].x &&
            (segment[0].y < center.y || segment[0].y > segment[segment.size() - 1].y))
        {
            lastSegmentPoint = segment[0];
        }
        else
        {
            lastSegmentPoint = segment[segment.size() - 1];
        }

        double distanceError = std::abs(Utils::distanceBetweenPoints(lastSegmentPoint, center) - radiusOfTheCircle);
        if (distanceError > DELTA)
        {
            lastSegmentPoint = getLastArcPoint(segment, radiusOfTheCircle, center);
        }
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
        //return 0;
        radiusOfTheCircle = Utils::distanceBetweenPoints(segment[0], center);
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
    modelTracker.getRoadModelPointer()->drawModel(drawing);
    //std::cout << "lineSegment added" << std::endl;
//    cv::imshow("drawing", drawing);
//    cv::waitKey(0);
}

void
RoadModelBuilder::buildRoadModel(RoadModelTracker &modelTracker, const std::vector<std::vector<cv::Point>> &contours,
                                 const std::vector<std::vector<double>> &contoursCurvatures, int COLS,
                                 double MULTIPLIER_OF_NUMBER_OF_CONTOUR_POINTS)
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
                                                                isRightContour, MULTIPLIER_OF_NUMBER_OF_CONTOUR_POINTS);
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

double RoadModelBuilder::calculateRadiusOfTheArcUsingContour(const std::vector<cv::Point> &arcSegment)
{
    cv::Point prev, middlePoint, next;

    middlePoint = arcSegment[arcSegment.size() / 2];
    prev = arcSegment[0];
    next = arcSegment[arcSegment.size() - 1];

    // если точки совпадают
    if (prev == middlePoint || middlePoint == next || next == prev)
    {
        return 0;
    }

    double a, b, c; // стороны треугольника
    a = sqrt(pow(prev.x - middlePoint.x, 2) + pow(prev.y - middlePoint.y, 2));
    b = sqrt(pow(middlePoint.x - next.x, 2) + pow(middlePoint.y - next.y, 2));
    c = sqrt(pow(next.x - prev.x, 2) + pow(next.y - prev.y, 2));

    double p = (a + b + c) / 2;
    double S = sqrt(p * (p - a) * (p - b) * (p - c)); // площадь треугольника по формуле Геррона

    double R = (a * b * c) / (4 * S); // радиус описанной окружности

    return R;
}

bool RoadModelBuilder::checkingForStartOfAnotherContour(const cv::Point &prevPoint, const cv::Point &currPoint)
{
    const int CONTOUR_POINTS_DELTA = 100;

    if (std::abs(currPoint.x - prevPoint.x) > CONTOUR_POINTS_DELTA ||
        std::abs(currPoint.y - prevPoint.y) > CONTOUR_POINTS_DELTA)
    {
//        addArcAndLineSegmentsToModel(modelTracker, arcSegment, currSumOfArcSegmentCurvatures, MIN_ARC_SEGMENT_SIZE,
//                                     lineSegment, isRightContour);
        return true;
    }
    return false;
}

bool RoadModelBuilder::checkingChangeOfContourDirection2(const cv::Point &prevPrevPoint, const cv::Point &prevPoint,
                                                         const cv::Point &currPoint)
{
    double angle = Utils::calculateAngleOfTriangle(prevPrevPoint, prevPoint, currPoint);

    if (angle == 0)
    {
        return false;
    }

    //std::cout << angle << std::endl;

    const double ANGLE_THRESHOLD = 90; //40

    if (angle <= ANGLE_THRESHOLD)
    {
        //std::cout << "angle changed" << std::endl;
//        std::cout << "\t" <<prevPrevPoint << std::endl;
//        std::cout << "\t" <<prevPoint << std::endl;
//        std::cout << "\t" <<currPoint << std::endl;
//        cv::waitKey(0);
        return true;
    }

    return false;
}

bool RoadModelBuilder::checkingChangeOfContourDirection(const cv::Point &prevPrevPoint, const cv::Point &currPoint)
{
    bool changed = false;

    static cv::Point prevDirectionalVector;
    cv::Point directionalVector(currPoint.x - prevPrevPoint.x, currPoint.y - prevPrevPoint.y);

    cv::Point change = prevDirectionalVector + directionalVector;

    std::cout << "change" << change << std::endl;
//    if (cv::waitKey(25) == 107)
//    {
//        cv::waitKey(0);
//    }

    if (std::abs(change.y - 1) <= 1 && std::abs(change.x - 1) <= 1 && prevDirectionalVector != cv::Point())
    {
        std::cout << "direction changed (not sights)" << std::endl;
//        cv::waitKey(0);

        prevDirectionalVector = cv::Point();
        changed = true;
    }
    else if (directionalVector.x * prevDirectionalVector.x < 0 || directionalVector.y * prevDirectionalVector.y < 0)
    {
        std::cout << "direction changed (sights)" << std::endl;
//        cv::waitKey(0);

        prevDirectionalVector = cv::Point();

        changed = true;
    }
    else
    {
        prevDirectionalVector = directionalVector;
    }
    return changed;
}

void RoadModelBuilder::addArcAndLineSegmentsToModel(RoadModelTracker &modelTracker, std::vector<cv::Point> &arcSegment,
                                                    double &currSumOfArcSegmentCurvatures,
                                                    const int MIN_ARC_SEGMENT_SIZE,
                                                    std::vector<cv::Point> &lineSegment, const bool isRightContour)
{
    if (!arcSegment.empty())
    {
        if (arcSegment.size() < MIN_ARC_SEGMENT_SIZE && !lineSegment.empty())
        {
            addArcSegmentPointsToLineSegment(arcSegment, lineSegment, currSumOfArcSegmentCurvatures);
            addLineSegmentToModel(modelTracker, lineSegment, isRightContour);
        }
        else
        {
            if (!addArcToTheModel(modelTracker, arcSegment, currSumOfArcSegmentCurvatures, isRightContour))
            {
                addLineSegmentToModel(modelTracker, arcSegment, isRightContour);
                currSumOfArcSegmentCurvatures = 0;
            }
        }
    }

    if (!lineSegment.empty())
    {
        addLineSegmentToModel(modelTracker, lineSegment, isRightContour);
    }


    assert(arcSegment.empty());
    assert(lineSegment.empty());
    assert(currSumOfArcSegmentCurvatures == 0);
}

void RoadModelBuilder::setValuesForFirstPointOfTheContour(std::vector<cv::Point> &lineSegment,
                                                          double &prevCurvature, const cv::Point &currPoint)
{
    lineSegment.emplace_back(currPoint);
    prevCurvature = 0;
}

/**
 * Calculation of the ratio of the change of the x coordinate and the y coordinate
 * @param arcSegment
 * @return
 */
bool RoadModelBuilder::checkIfArcSegmentIsStraight(const std::vector<cv::Point> &arcSegment)
{
    int minX = arcSegment[0].x;
    int maxX = arcSegment[0].x;
    int minY = arcSegment[0].y;
    int maxY = arcSegment[0].y;

    for (int i = 1; i < arcSegment.size(); ++i)
    {
        if (arcSegment[i].x < minX)
        {
            minX = arcSegment[i].x;
        }

        if (arcSegment[i].x > maxX)
        {
            maxX = arcSegment[i].x;
        }

        if (arcSegment[i].y < minY)
        {
            minY = arcSegment[i].y;
        }

        if (arcSegment[i].y > maxY)
        {
            maxY = arcSegment[i].y;
        }
    }

    const double MIN_RATIO_OF_X_TO_Y = 0.3;
    double ratio = static_cast<double>(maxX - minX) / (maxY - minY);
    //std::cout << "ratio = " << ratio << std::endl;

    if (ratio < MIN_RATIO_OF_X_TO_Y)
    {
        return true;
    }
    return false;
}

void RoadModelBuilder::calculationStartAndEndAnglesOfTheArc2(double &startAngle, double &endAngle,
                                                             const std::vector<cv::Point> &segment,
                                                             const cv::Point &center, double radiusOfTheCircle)
{

    //TODO
    const cv::Point &startPoint = segment[0];
    cv::Point endPoint = segment[segment.size() - 1];

    double distanceError = std::abs(Utils::distanceBetweenPoints(endPoint, center) - radiusOfTheCircle);
    std::cout << "distance error = " << distanceError << std::endl;
    if (distanceError > 5)
    {
        endPoint = getLastArcPoint(segment, radiusOfTheCircle, center);
    }

    double t1 = acos(static_cast<double>(startPoint.x - center.x) / radiusOfTheCircle);
//    t1 *= 180.0 / CV_PI;
//    t1 = 360 - t1;

    double t2 = acos(static_cast<double>(endPoint.x - center.x) / radiusOfTheCircle);
//    t2 *= 180.0 / CV_PI;
//    t2 = 360 - t2;

    startAngle = t1;
    endAngle = t2;

    //--------------------------
    cv::Point p0(radiusOfTheCircle * cos(t1) + center.x, radiusOfTheCircle * sin(t1) + center.y);
    cv::Point p1(radiusOfTheCircle * cos(t2) + center.x, radiusOfTheCircle * sin(t2) + center.y);

    std::cout << "newAngles" << std::endl;
    drawArcSegment(segment, p0);
    drawArcSegment(segment, p1);
}

