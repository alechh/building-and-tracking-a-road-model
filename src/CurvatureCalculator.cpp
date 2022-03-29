//
// Created by alechh on 29.03.2022.
//

#include <opencv2/opencv.hpp>
#include <vector>
#include "Utils.h"
#include "CurvatureCalculator.h"

/**
 * The first way to calculate the curvature (through numerical calculation of derivatives)
 * @param vecContourPoints
 * @param step
 * @return
 */
void CurvatureCalculator::calculateCurvature(std::vector<double> &contourCurvature, const std::vector<cv::Point> &vecContourPoints,
                                             int step = 1)
{
    if (vecContourPoints.size() < step)
    {
        return;
    }

    auto frontToBack = vecContourPoints.front() - vecContourPoints.back();
    bool isClosed = ((int)std::max(abs(frontToBack.x), abs(frontToBack.y))) <= 1;

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
                contourCurvature[i] = std::numeric_limits<double>::infinity();
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

        contourCurvature[i] = curvature2D;
    }
}

/**
 * The second way to calculate the curvature.
 * The adjacent 3 points of the contour are viewed as a triangle, and a circle describing this triangle is searched for.
 * The inverse of the radius of this circle is the curvature
 * @param contour
 * @param step
 * @return
 */
void CurvatureCalculator::calculateCurvature2(std::vector<double> &contourCurvature, const std::vector<cv::Point> &contour,
                                              const int step)
{
    for (int i = 1; i < contour.size() - 2; ++i)
    {
        cv::Point prev, curr, next;

        curr = contour[i];
        if (step != 1)
        {
            Utils::selectionOfPointsDependingOnTheStep(prev, next, contour, step, i);
        }
        else
        {
            prev = contour[i - 1];
            next = contour[i + 1];
        }


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
    }
}