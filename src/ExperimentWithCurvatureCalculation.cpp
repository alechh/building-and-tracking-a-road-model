//
// Created by alechh on 21.02.2022.
//

#include <opencv2/opencv.hpp>
#include "ExperimentWithCurvatureCalculation.h"
#include "RoadModel.h"


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

RoadModel ExperimentWithCurvatureCalculation::buildRoadModelBasedOnTheSingleContour(const std::vector<cv::Point> &contour, const std::vector<double> &contourCurvature)
{
    /**
     * Построение модели дороги (а точнее нашей полосы движения) по одному контуру.
     * Пока строим ПРАВУЮ часть модели
     */
    RoadModel roadModel;

    cv::Point lineSegmentBegin = cv::Point(-1, -1);
    cv::Point lineSegmentEnd = cv::Point(-1, -1);

    double curvatureThreshold = 0.5; // это порог кривизны. Если кривизна ниже этого порога, то считаем эту часть контура прямой
    for (int i = 0; i < contour.size(); ++i)
    {
        if (std::abs(contourCurvature[i]) < curvatureThreshold)
        {
            // Это часть прямой
            if (lineSegmentBegin == cv::Point(-1, -1))
            {
                // если это начало прямого отрезка
                lineSegmentBegin = contour[i];
            }
            lineSegmentEnd = contour[i];
        }
        else
        {
            // Это дуга окружности
            if (lineSegmentEnd == cv::Point(-1, -1))
            {
                // если до этого не шел прямой отрезок
                if (contourCurvature[i] != std::numeric_limits<double>::infinity())
                {
                    roadModel.addElementToRight(contour[i], contourCurvature[i]);
                }
            }
            else
            {
                // если закончился прямой отрезок и встилась часть контура, которой соответствует дуга окружности
                roadModel.addElementToRight(lineSegmentBegin, lineSegmentEnd);

                lineSegmentBegin = cv::Point(-1, -1);
                lineSegmentEnd = cv::Point(-1, -1);

                if (contourCurvature[i] != std::numeric_limits<double>::infinity())
                {
                    roadModel.addElementToRight(contour[i], contourCurvature[i]);
                }
            }
        }
    }

    return roadModel;
}

void ExperimentWithCurvatureCalculation::drawRoadModel(cv::Mat &src, const RoadModel &roadModel)
{
    /**
     * Отрисовка модели дороги (пока только ПРАВОЙ её части).
     */

    /**
     * Вопрос: почему этот метод находится не в классе RoadModel ???
     */
     std::shared_ptr<ModelElement> currModelElement(roadModel.rightHead);

     while(currModelElement)
     {
         currModelElement->drawModelElement(src);
         currModelElement = currModelElement->next;
     }
}

