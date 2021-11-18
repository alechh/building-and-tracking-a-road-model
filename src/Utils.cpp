//
// Created by alechh on 17.11.2021.
//

#include "Utils.h"

std::vector<double> Utils::calculate_curvature(const std::vector<cv::Point> &vecContourPoints, int step)
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
