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


cv::Point2f Utils::getFirstDerivative(const cv::Point2f &pplus, const cv::Point2f &pminus, int iplus, int iminus)
{
    cv::Point2f firstDerivative;

    firstDerivative.x = (pplus.x - pminus.x) / static_cast<float>(iplus - iminus);
    firstDerivative.y = (pplus.y - pminus.y) / static_cast<float>(iplus - iminus);

    return firstDerivative;
}


/**
 * Get coefficients of the tangent equation
 * @param touchPoint -- the point of contact of a straight line
 * @param firstDerivative -- first derivative of the curve at the point of tangency in two directions
 * @return -- vector of the coefficinets A * y + B * x + C = 0.
 * coefficients[0] -- A
 * coefficients[1] -- B
 * coefficients[2] -- C
 */
std::vector<double> Utils::getCoefficientsOfTheTangent(const cv::Point &touchPoint, const cv::Point2f &firstDerivative)
{
    // y - y0 = f'(x0) * (x - x0)

    std::vector<double> coefficients(3);
    coefficients[0] = 1;
    coefficients[1] = -firstDerivative.x;
    coefficients[2] = firstDerivative.x * touchPoint.x - touchPoint.y;

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
        if (!hasPMinusChanged)
        {
            pMinusIndex--;
            pMinus = segment[pMinusIndex];

            hasPMinusChanged = true;
        }
        else
        {
            pPlusIndex++;
            pPlus = segment[pPlusIndex];

            hasPMinusChanged = false;
        }
    }
}


