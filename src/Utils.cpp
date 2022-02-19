//
// Created by alechh on 17.11.2021.
//

#include "Utils.h"
#include <vector>
#include <opencv2/opencv.hpp>

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
double Utils::mean_curvature(const std::vector<double>& curvature)
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
        std::cerr << "number_of_contours must be less than contours.size()" << std::endl;
        return;
    }

    std::size_t boundary = (number_of_contours == 0) ? contours.size() : number_of_contours;

    for (std::size_t i = 0; i < boundary; ++i)
    {
        cv::drawContours(input_frame, contours, i, cv::Scalar(255, 255, 255));
    }
}


void Utils::calculate_contours_curvature(std::vector<std::vector<double>> &contoursCurvature, const std::vector<std::vector<cv::Point>> &contours)
{
    for (int i = 0; i < contours.size(); ++i)
    {
        contoursCurvature[i] = Utils::calculate_curvature(contours[i]);
    }
}
