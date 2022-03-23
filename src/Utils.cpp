//
// Created by alechh on 17.11.2021.
//

#include "Utils.h"
#include <vector>
#include <opencv2/opencv.hpp>


std::vector<double> Utils::calculate_curvature(const std::vector<cv::Point> &vecContourPoints, int step = 1)
{
    std::vector<double> vecCurvature(vecContourPoints.size());

    if (vecContourPoints.size() < step)
    {
        return vecCurvature;
    }

    auto frontToBack = vecContourPoints.front() - vecContourPoints.back();
    bool isClosed = ((int)std::max(std::abs(frontToBack.x), std::abs(frontToBack.y))) <= 1;

    cv::Point2f pplus, pminus;
    cv::Point2f f1stDerivative, f2ndDerivative;
    for (int i = 0; i < vecContourPoints.size(); i++)
    {
        const cv::Point2f& pos = vecContourPoints[i];

        int maxStep = step;
        if (!isClosed)
        {
            maxStep = std::min(std::min(step, i), (int)vecContourPoints.size() - 1 - i);
            if (maxStep == 0)
            {
                vecCurvature[i] = std::numeric_limits<double>::infinity();
                continue;
            }
        }

        int iminus = i - maxStep;
        int iplus = i + maxStep;
        pminus = vecContourPoints[iminus < 0 ? iminus + vecContourPoints.size() : iminus];
        pplus = vecContourPoints[iplus > vecContourPoints.size() ? iplus - vecContourPoints.size() : iplus];

        f1stDerivative.x = (pplus.x - pminus.x) / float(iplus - iminus);
        f1stDerivative.y = (pplus.y - pminus.y) / float(iplus - iminus);
        f2ndDerivative.x = (pplus.x - 2 * pos.x + pminus.x) / float(float(iplus - iminus) / 2 * float(iplus - iminus) / 2);
        f2ndDerivative.y = (pplus.y - 2 * pos.y + pminus.y) / float(float(iplus - iminus) / 2 * float(iplus - iminus) / 2);

        double curvature2D;
        double divisor = f1stDerivative.x * f1stDerivative.x + f1stDerivative.y * f1stDerivative.y;
        if (std::abs(divisor) > 10e-15)  // 10e-8
        {
            curvature2D =  std::abs(f2ndDerivative.y * f1stDerivative.x - f2ndDerivative.x * f1stDerivative.y) /
                           pow(divisor, 3.0 / 2.0 )  ;
        }
        else
        {
            curvature2D = std::numeric_limits<double>::infinity();
        }

        vecCurvature[i] = curvature2D;
    }
    return vecCurvature;
}


/**
 * Removing contours that have a small number of points
 * @param contours
 * @param min_contours_size
 * @return number of deleted contours
 */
int Utils::remove_small_contours(std::vector< std::vector<cv::Point> > & contours, const int min_contours_size)
{
    std::vector< std::vector<cv::Point> > new_contours;

    for (int i = 0; i < contours.size(); ++i)
    {
        if (contours[i].size() >= min_contours_size)
        {
            new_contours.emplace_back(contours[i]);
        }
    }

    int number_of_deleted_contours = contours.size() - new_contours.size();
    contours = new_contours;

    return number_of_deleted_contours;
}


/**
 * Arithmetic mean of curvature std::vector<double>
 * @param curvature
 * @return
 */
double Utils::mean_curvature(const std::vector<double> &curvature)
{
    double res = 0;
    int count = 0;

    for (auto i : curvature)
    {
        if (i != 0 and i != std::numeric_limits<double>::infinity())
        {
            res += i;
            ++count;
        }
    }

    if (count != 0)
    {
        res /= count;
    }

    return res;
}

void Utils::sort_vector_of_vectors_of_points(std::vector<std::vector<cv::Point>>& contours)
{
    // сортируем вектор векторов по их размеру
    std::sort(contours.begin(),contours.end(),
              [](const std::vector<cv::Point> &v1, const std::vector<cv::Point> &v2)
              {
                  return v1.size() > v2.size();
              });
}

/**
 * Drawing contours on an image
 * @param contours
 * @param input_frame
 * @param number_of_contours -- number of contours to draw
 */
void Utils::draw_contours(const std::vector<std::vector<cv::Point>> &contours, cv::Mat &input_frame, int number_of_contours)
{
    if (number_of_contours > contours.size())
    {
        std::cerr << "Utils::draw_contours: number_of_contours must be less than contours.size()" << std::endl;
        return;
    }

    std::size_t boundary = (number_of_contours == 0) ? contours.size() : number_of_contours;

    for (std::size_t i = 0; i < boundary; ++i)
    {
        // TODO Почему-то не билдятся юнит-тесты, ругается на эту фукнцию, а main билдится нормально
        // cv::drawContours(input_frame, contours, i, cv::Scalar(255, 255, 255));
    }
}


void Utils::calculate_contours_curvature(std::vector<std::vector<double>> &contoursCurvature, const std::vector<std::vector<cv::Point>> &contours, int step = 1)
{
    for (int i = 0; i < contours.size(); ++i)
    {
        contoursCurvature[i] = Utils::calculate_curvature(contours[i], step);
    }
}

std::vector<double> Utils::calculate_curvature_2(const std::vector<cv::Point> &contour, int step)
{
    std::vector<double> contourCurvature(contour.size());

    for (int i = 1; i < contour.size() - 2;)
    {

        cv::Point prev, curr, next;
        prev = contour[i - 1];
        curr = contour[i];
        next = contour[i + 1];

        // если точки лежат на одной прямой
        if ((prev.x == curr.x && curr.x == next.x) || (prev.y == curr.y && curr.y == next.y))
        {
            contourCurvature[i] = 0;

            i++;
            if (i > contour.size() - 2)
            {
                break;
            }

            continue;
        }

        // если точки совпадают
        if (prev == curr || curr == next || next == prev)
        {
            contourCurvature[i] = 0;

            i++;
            if (i > contour.size() - 2)
            {
                break;
            }

            continue;
        }

        double a, b, c; // стороны треугольника
        a = sqrt(pow(prev.x - curr.x, 2) + pow(prev.y - curr.y, 2));
        b = sqrt(pow(curr.x - next.x, 2) + pow(curr.y - next.y, 2));
        c = sqrt(pow(next.x - prev.x, 2) + pow(next.y - prev.y, 2));

        double p = (a + b + c) / 2;
        double S = sqrt(p * (p - a) * (p - b) * (p - c)); // площадь треугольника по формуле Геррона

        double R = (a * b * c) / (4 * S); // радиус описанной окружности

        contourCurvature[i] = 1.0 / R; // кривизна = 1 / R

        // Если шаг != 1, то заполняем кривизну в точках, которые мы пропустим
        for (int k = i + 1; k < i + step; ++k)
        {
            if (k == contour.size())
            {
                break;
            }
            contourCurvature[k] = 1.0 / R;
        }

        i += step;

        // Если шаг != 1, то нужна проверка, что мы не вылетели за пределы массива
        // Причем последний элемент, на котором мы должны остановится, это contour[contour.size() - 2]
        if (i > contour.size() - 2)
        {
            break;
        }
    }

    return contourCurvature;
}


cv::Point2f Utils::getFirstDerivative(const cv::Point2f &pPlus, const cv::Point2f &pMinus, int iPlus, int iMinus)
{
    cv::Point2f firstDerivative;

    firstDerivative.x = (pPlus.x - pMinus.x) / static_cast<float>(iPlus - iMinus);
    firstDerivative.y = (pPlus.y - pMinus.y) / static_cast<float>(iPlus - iMinus);

    return firstDerivative;
}


/**
 * Get coefficients of the tangent equation
 * @param touchPoint -- the point of contact of a straight line
 * @param firstDerivative -- first derivative of the curve at the point of tangency in two directions
 * @return -- vector of the coefficinets A * x + B * y + C = 0.
 * coefficients[0] -- A
 * coefficients[1] -- B
 * coefficients[2] -- C
 */
std::vector<double> Utils::getCoefficientsOfTheTangent(const cv::Point &touchPoint, const cv::Point2f &firstDerivative)
{
    // y - y0 = f'(x0) * (x - x0)
    // f'(x0) * x - 1 * y + y0 - f'(x0) * x0 = 0

    std::vector<double> coefficients(3);
    coefficients[0] = firstDerivative.x;
    coefficients[1] = -1;
    coefficients[2] = touchPoint.y - firstDerivative.x * touchPoint.x;

    return coefficients;
}


/**
 * Get points pPlus, pMinus and its indices in segment.
 * These points and indices are used to calculate the derivative at the point that is between pPlus and pMinus.
 * @param segment -- vector of the points of the segment
 * @param pPlus -- next point from the center
 * @param pMinus -- previous point to the center
 * @param pPlusIndex -- index of the pPlus point
 * @param pMinusIndex -- index of the pMinus point
 */
void Utils::getPPlusAndPMinus(const std::vector<cv::Point> &segment, cv::Point &pPlus, cv::Point &pMinus, int &pPlusIndex, int &pMinusIndex, cv::Point &centerPoint)
{
    int indexOfTheCenter = segment.size() / 2;

    centerPoint = segment[indexOfTheCenter];

    pMinusIndex = indexOfTheCenter - 1;
    pPlusIndex = indexOfTheCenter + 1;

    pMinus = segment[pMinusIndex];
    pPlus = segment[pPlusIndex];

    // Нужно, чтобы точки были различными (зачем?)
    bool hasPMinusChanged = false;
    while (pMinus != pPlus)
    {
        if (pMinusIndex == 0 && pPlusIndex == segment.size() - 1)
        {
            break;
        }

        if (!hasPMinusChanged)
        {
            if (pMinusIndex != 0)
            {
                pMinusIndex--;
                pMinus = segment[pMinusIndex];
            }

            hasPMinusChanged = true;
        }
        else
        {
            if (pPlusIndex != segment.size() - 1)
            {
                pPlusIndex++;
                pPlus = segment[pPlusIndex];
            }

            hasPMinusChanged = false;
        }
    }
}

/**
 *
 * @param coefficientsOfTheLine -- vector of the coefficinets A * x + B * y + C = 0.
 *  coefficientsOfTheLine[0] -- A
 *  coefficientsOfTheLine[1] -- B
 *  coefficientsOfTheLine[2] -- C
 * @return -- vector of the coefficinets of the perpendicular line A' * x + B' * y + C' = 0.
 *  oefficients[0] -- A'
 *  oefficients[1] -- B'
 *  oefficients[2] -- C'
 */
std::vector<double> Utils::getCoefficientsOfThePerpendicularLine(const std::vector<double> &coefficientsOfTheLine, const cv::Point &touchPoint)
{
    // -B * x + A * y + (B * x0 - A * y0)
    std::vector<double> coefficientsOfThePerpendicularLine(3);

    coefficientsOfThePerpendicularLine[0] = -coefficientsOfTheLine[1];
    coefficientsOfThePerpendicularLine[1] = coefficientsOfTheLine[0];
    coefficientsOfThePerpendicularLine[2] = coefficientsOfTheLine[1] * touchPoint.x - coefficientsOfTheLine[0] * touchPoint.y;

    return coefficientsOfThePerpendicularLine;
}


/**
 * Solution of the equation (x - x0)^2 + (-B/A * x - C/A - y0)^2 = R^2
 * https://www.wolframalpha.com/input?i=%28x-x0%29%5E2+%2B+%28-B%2FA*x-C%2FA-y0%29%5E2+%3D+R%5E2
 * @return -- typle of the solutions x1, x2
 */
std::tuple<double, double> Utils::solutionOfTheSystemWithRespectToX(double A, double B, double C, double R, const cv::Point &point)
{
    double x1, x2;

    x1 = (sqrt(B*B*R*R - B*B*point.y*point.y - 2*A*B*point.x*point.y - 2*B*C*point.y + A*A*R*R - A*A*point.x*point.x - 2*A*C*point.x - C*C) / B - (A*C)/(B*B) - (A*point.y)/B + point.x) / ((A*A) / (B*B) + 1);
    x2 = (-sqrt(B*B*R*R - B*B*point.y*point.y - 2*A*B*point.x*point.y - 2*B*C*point.y + A*A*R*R - A*A*point.x*point.x - 2*A*C*point.x - C*C) / B - (A*C)/(B*B) - (A*point.y)/B + point.x) / ((A*A) / (B*B) + 1);

    return std::make_tuple(x1, x2);
}

/**
 * When solving a system of equations , y was expressed in terms of x as follows
 *  y = -A/B * x - C/A
 * @return
 */
int calculatingYThroughX(double x, double A, double B, double C)
{
    return -A / B * x - C / B;
}


std::tuple<cv::Point, cv::Point> Utils::calculatingPointsOfStraightLineAtCertainDistanceFromGivenPoint(double A, double B, double C, double R, const cv::Point &point)
{
    cv::Point firstSolution, secondSolution;

    std::tuple<double, double> xSolutions = Utils::solutionOfTheSystemWithRespectToX(A, B, C, R, point);

    firstSolution.x = std::get<0>(xSolutions);
    firstSolution.y = calculatingYThroughX(firstSolution.x, A, B, C);

    secondSolution.x = std::get<1>(xSolutions);
    secondSolution.y = calculatingYThroughX(secondSolution.x, A, B, C);

    std::tuple<cv::Point, cv::Point> resultPoints = std::make_tuple(firstSolution, secondSolution);

    return resultPoints;
}


double Utils::distanceBetweenPoints(const cv::Point &a, const cv::Point &b)
{
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}


cv::Point Utils::chooseAmongTwoCandidatesForCenter(const std::vector<cv::Point> &segment, const std::tuple<cv::Point, cv::Point> &candidatesForCenter)
{
    if (Utils::distanceBetweenPoints(segment[0], std::get<0>(candidatesForCenter)) < Utils::distanceBetweenPoints(segment[0], std::get<1>(candidatesForCenter)))
    {
        return std::get<0>(candidatesForCenter);
    }
    else
    {
        return std::get<1>(candidatesForCenter);
    }
}


cv::Point Utils::getMidpoint(const cv::Point &a, const cv::Point &b)
{
    cv::Point midPoint;
    midPoint.x = (b.x + a.x) / 2;
    midPoint.y = (b.y + a.y) / 2;

    return midPoint;
}


double Utils::getAngleOfTheArc(double &startAngle, double &endAngle, const std::vector<cv::Point> &segment, const cv::Point &center, double radiusOfTheCircle)
{
    cv::Point M = Utils::getMidpoint(segment[0], segment[segment.size() - 1]);

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


double Utils::calculateAngleShift(const cv::Point &A, const cv::Point &C)
{
    cv::Point T(A.x, C.y);
    double distAC = Utils::distanceBetweenPoints(A, C);
    double distAT = Utils::distanceBetweenPoints(A, T);

    if (distAT > distAC)
    {
        return 0;
    }

    double angleC = asin(distAT / distAC) * 180 / CV_PI;

    return angleC;
}


double Utils::calculationStartAndEndAnglesOfTheArc(double &startAngle, double &endAngle, const std::vector<cv::Point> &segment, const cv::Point &center, double radiusOfTheCircle)
{
    double angleC = Utils::getAngleOfTheArc(startAngle, endAngle, segment, center, radiusOfTheCircle);

    double shiftAngle = Utils::calculateAngleShift(segment[0], center);

    startAngle = 180 + shiftAngle;

    endAngle = startAngle + angleC;
}