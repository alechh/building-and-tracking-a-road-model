//
// Created by alechh on 17.11.2021.
//

#include "RoadModel.h"
#include "Utils.h"
#include <vector>
#include <opencv2/opencv.hpp>


/**
 * Removing contours that have a small number of points
 * @param contours
 * @param minContoursSize
 * @return number of deleted contours
 */
int Utils::removeSmallContours(std::vector< std::vector<cv::Point> > &contours, const int minContoursSize)
{
    std::vector< std::vector<cv::Point> > newContours;

    for (int i = 0; i < contours.size(); ++i)
    {
        if (contours[i].size() >= minContoursSize)
        {
            newContours.emplace_back(contours[i]);
        }
    }

    int numberOfDeletedContours = contours.size() - newContours.size();
    contours = newContours;

    return numberOfDeletedContours;
}


/**
 * Arithmetic mean of curvature std::vector<double>
 * @param curvature
 * @return
 */
double Utils::meanCurvature(const std::vector<double> &curvature)
{
    double res = 0;
    int count = 0;

    for (auto i : curvature)
    {
        if (i != std::numeric_limits<double>::infinity() && i != 0)
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

void Utils::sortVectorOfVectorsOfPoints(std::vector<std::vector<cv::Point>> &contours)
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
 * @param dst
 * @param numberOfContours -- number of contours to draw
 */
void Utils::drawContours(const std::vector<std::vector<cv::Point>> &contours, cv::Mat &dst, int numberOfContours)
{
    if (numberOfContours > contours.size())
    {
        std::cerr << "Utils::drawContours: numberOfContours must be less than contours.size()" << std::endl;
        return;
    }

    std::size_t boundary = (numberOfContours == 0) ? contours.size() : numberOfContours;

    for (std::size_t i = 0; i < boundary; ++i)
    {
        // TODO Почему-то не билдятся юнит-тесты, ругается на эту функцию, а main билдится нормально
        // cv::drawContours(dst, contours, i, cv::Scalar(255, 255, 255));
    }
}


/**
 * Selection of side points for the triangle depending on the step size.
 * It is assumed that the points should be at a distance of step /2 from the current point of the contour
 * @param prev
 * @param next
 * @param contour
 * @param step -- size of the step
 * @param currIndex -- index of the current point of the contour
 */
void Utils::selectionOfPointsForTriangleDependingOnTheStep(cv::Point &prev, cv::Point &next,
                                                           const std::vector<cv::Point> &contour, const int step,
                                                           const int currIndex)
{
    const int neighborhoodOfTheStep = step / 2;

    int tempIndexPrev = currIndex;
    while (tempIndexPrev > 0 && std::abs(tempIndexPrev - currIndex) < neighborhoodOfTheStep)
    {
        tempIndexPrev--;
    }

    int tempIndexNext = currIndex;
    while (tempIndexNext < contour.size() - 1 && std::abs(tempIndexNext - currIndex) < neighborhoodOfTheStep)
    {
        tempIndexNext++;
    }

    prev = contour[tempIndexPrev];
    next = contour[tempIndexNext];
}


cv::Point2f
Utils::calculateFirstDerivative(const cv::Point2f &pPlus, const cv::Point2f &pMinus, int iPlus, int iMinus, double h)
{
    cv::Point2f firstDerivative;

    firstDerivative.x = (pPlus.x - pMinus.x) / h;
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
std::vector<double>
Utils::calculateCoefficientsOfTheTangent(const cv::Point &touchPoint, const cv::Point2f &firstDerivative)
{
    // y - y0 = f'(x0) * (x - x0)   <=>
    // f'(x0) * x - 1 * y + y0 - f'(x0) * x0 = 0

    std::vector<double> coefficients(3);
    coefficients[0] = firstDerivative.x;
    coefficients[1] = -1;
    coefficients[2] = -touchPoint.y - firstDerivative.x * touchPoint.x;

    return coefficients;
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
std::vector<double> Utils::calculateCoefficientsOfThePerpendicularLine(const std::vector<double> &coefficientsOfTheLine,
                                                                       const cv::Point &touchPoint)
{
    // -B * x + A * y + (B * x0 - A * y0) -- a perpendicular line passing through a point (x0, y0)
    std::vector<double> coefficientsOfThePerpendicularLine(3);

    coefficientsOfThePerpendicularLine[0] = coefficientsOfTheLine[1];
    coefficientsOfThePerpendicularLine[1] = -coefficientsOfTheLine[0];
    coefficientsOfThePerpendicularLine[2] = -coefficientsOfTheLine[1] * touchPoint.x - coefficientsOfTheLine[0] * touchPoint.y;

    return coefficientsOfThePerpendicularLine;
}


/**
 * Solution of the equation (x - x0)^2 + (-A/B * x - C/B - y0)^2 = R^2
 * https://www.wolframalpha.com/input?i=sqrt%28%28x-x0%29%5E2+%2B+%28-A%2FB*x-C%2FB-y0%29%5E2%29+%3D+R
 * @return -- tuple of the solutions x1, x2
 */
std::tuple<double, double>
Utils::solutionOfTheSystemWithRespectToX(const double A, const double B, const double C, const double R,
                                         const cv::Point &point)
{
    double x1, x2;

    double x0, y0;
    x0 = point.x;
    y0 = point.y;

    double denominator = 2 * (A * A + B * B);

    x1 = (-sqrt(pow(2 * A * B * y0 + 2 * A * C - 2 * B * B * x0, 2) -
                4 * (A * A + B * B) * (-B * B * R * R + B * B * x0 * x0 + B * B * y0 * y0 + 2 * B * C * y0 + C * C)) -
          2 * A * B * y0 - 2 * A * C + 2 * B * B * x0) / denominator;
    x2 = (sqrt(pow(2 * A * B * y0 + 2 * A * C - 2 * B * B * x0, 2) -
               4 * (A * A + B * B) * (-B * B * R * R + B * B * x0 * x0 + B * B * y0 * y0 + 2 * B * C * y0 + C * C)) -
          2 * A * B * y0 - 2 * A * C + 2 * B * B * x0) / denominator;

    return std::make_tuple(x1, x2);
}

/**
 * When solving a system of equations, y was expressed in terms of x as follows
 *  y = -A/B * x - C/A
 * @return
 */
int Utils::calculatingYThroughX(double x, double A, double B, double C)
{
    return (-A / B * x - C / B);
}


/**
 * A function for calculating two points on a straight line located at a distance R from a given point.
 * The line is given by the equation Ax + By + C = 0
 * @param A
 * @param B
 * @param C
 * @param R -- distance from the given point
 * @param point -- the point from which the distance is calculated
 * @return
 */
std::tuple<cv::Point, cv::Point>
Utils::calculatingPointsOfStraightLineAtCertainDistanceFromGivenPoint(const double A, const double B, const double C,
                                                                      const double R, const cv::Point &point)
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


/**
 * Distance between two given points
 * @param A
 * @param B
 * @return
 */
double Utils::distanceBetweenPoints(const cv::Point &A, const cv::Point &B)
{
    return sqrt(pow(A.x - B.x, 2) + pow(A.y - B.y, 2));
}


/**
 * Choosing from two points closest to the edge of the segment.
 * We assume that the segment is slightly rounded, so we are interested in the point that is closer to the end of
 * this segment. This point is the center of the circle describing this segment.
 * @param segment
 * @param candidatesForCenter
 * @return
 */
cv::Point Utils::chooseAmongTwoCandidatesForCenter(const std::vector<cv::Point> &segment,
                                                   const std::tuple<cv::Point, cv::Point> &candidatesForCenter)
{
    if (Utils::distanceBetweenPoints(segment[0], std::get<0>(candidatesForCenter)) <
        Utils::distanceBetweenPoints(segment[0], std::get<1>(candidatesForCenter)))
    {
        return std::get<0>(candidatesForCenter);
    }
    else
    {
        return std::get<1>(candidatesForCenter);
    }
}


/**
 * A function that finds a point located in the middle between two given points
 * @param a
 * @param b
 * @return
 */
cv::Point Utils::getMidpoint(const cv::Point &a, const cv::Point &b)
{
    cv::Point midPoint;
    midPoint.x = (b.x + a.x) / 2;
    midPoint.y = (b.y + a.y) / 2;

    return midPoint;
}