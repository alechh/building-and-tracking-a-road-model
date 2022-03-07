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


cv::Point getCenterOfTheArc(const cv::Point &begin, const cv::Point &end)
{
    return (begin + end) / 2;
}


RoadModel ExperimentWithCurvatureCalculation::buildRoadModelBasedOnTheSingleContour(const std::vector<cv::Point> &contour, const std::vector<double> &contourCurvature)
{
    /**
     * Построение модели дороги (а точнее нашей полосы движения) по одному контуру.
     * Пока строим ПРАВУЮ часть модели
     */
    RoadModel roadModel;

    cv::Point currElementBegin = contour[0];
    cv::Point currElementEnd = contour[0];

    int currLineSegmentNumber = 0;
    int currArcSegmentNumber = 0;

    double curvatureThreshold = 0.5; // это порог кривизны. Если кривизна ниже этого порога, то считаем эту часть контура прямой

    double prevCurvature = contourCurvature[0]; // это предыдущее значение, чтобы выделять участки контура с одним и тем же значением кривизны для построения модели
    const double delta = 0; // это для дельта-окрестности кривизны (если prevCurvature - delta <= currCurvature < prevCurvature + delta, то currCurvature относится к текущему участку

    for (int i = 0; i < contour.size(); ++i)
    {
        if (std::abs(contourCurvature[i]) < curvatureThreshold) // если это часть прямой
        {
            if (currArcSegmentNumber > 0) // если до прямой этого была дуга
            {
                cv::Point center = getCenterOfTheArc(currElementBegin, currElementEnd);
                roadModel.addElementToRight(center, 1.0 / prevCurvature);

                std::cout << currArcSegmentNumber << std::endl;

                currArcSegmentNumber = 0;
            }

            currLineSegmentNumber++;
            currElementEnd = contour[i];
        }
        else // если это дуга окружности
        {
            if (currLineSegmentNumber > 0) // если до дуги шел участок прямой
            {
                roadModel.addElementToRight(currElementBegin, currElementEnd);

                currLineSegmentNumber = 0;

                currElementBegin = contour[i];
                currElementEnd = contour[i];
                prevCurvature = contourCurvature[i];
                continue;
            }

            if (contourCurvature[i] != std::numeric_limits<double>::infinity())
            {
                currArcSegmentNumber++;

                if (std::abs(contourCurvature[i] - prevCurvature) <= delta) // если продолжается текущий участок
                {
                    currElementEnd = contour[i];
                }
                else // если встретился новый участок уровня кривизны
                {
                    cv::Point center = getCenterOfTheArc(currElementBegin, currElementEnd); // пока очень неточно вычисляем центр
                    // мб центр считать динамически сдвигать

                    roadModel.addElementToRight(center, 1.0 / prevCurvature);

                    std::cout << currArcSegmentNumber << std::endl;

                    currArcSegmentNumber = 0;

                    currElementBegin = contour[i];
                    currElementEnd = contour[i];
                    prevCurvature = contourCurvature[i];
                }
            }
        }
    }

    if (currArcSegmentNumber > 0)
    {
        cv::Point center = getCenterOfTheArc(currElementBegin, currElementEnd);
        roadModel.addElementToRight(center, 1.0 / prevCurvature);

        std::cout << currArcSegmentNumber << std::endl;
    }

    if (currLineSegmentNumber > 0)
    {
        roadModel.addElementToRight(currElementBegin, currElementEnd);
    }

    return roadModel;
}
