//
// Created by alechh on 17.11.2021.
//

#ifndef TEST_FUNCTIONS_FOR_VKR_UTILS_H
#define TEST_FUNCTIONS_FOR_VKR_UTILS_H


#include <vector>
#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>
#include "RoadModel.h"

class Utils
{
public:
    static int removeSmallContours(std::vector<std::vector<cv::Point> > &contours, int minContoursSize);

    static double meanCurvature(const std::vector<double> &curvature);

    static cv::Point getMidpoint(const cv::Point &a, const cv::Point &b);

    static void sortVectorOfVectorsOfPoints(std::vector<std::vector<cv::Point>> &contours);

    static void drawContours(const std::vector<std::vector<cv::Point>> &contours, cv::Mat &dst, int numberOfContours);

    static cv::Point2f
    calculateFirstDerivative(const cv::Point2f &pPlus, const cv::Point2f &pMinus, int iPlus, int iMinus, double h);

    static std::vector<double>
    calculateCoefficientsOfTheTangent(const cv::Point &touchPoint, const cv::Point2f &firstDerivative);

    static std::vector<double>
    calculateCoefficientsOfThePerpendicularLine(const std::vector<double> &coefficientsOfTheLine,
                                                const cv::Point &touchPoint);

    static std::tuple<double, double>
    solutionOfTheSystemWithRespectToX(double A, double B, double C, double R, const cv::Point &point);

    static std::tuple<cv::Point, cv::Point>
    calculatingPointsOfStraightLineAtCertainDistanceFromGivenPoint(double A, double B, double C, double R,
                                                                   const cv::Point &point);

    static cv::Point chooseAmongTwoCandidatesForCenter(const std::vector<cv::Point> &segment,
                                                       const std::tuple<cv::Point, cv::Point> &candidatesForCenter);

    static double distanceBetweenPoints(const cv::Point &A, const cv::Point &B);

    static void selectionOfPointsForTriangleDependingOnTheStep(cv::Point &prev, cv::Point &next,
                                                               const std::vector<cv::Point> &contour, int step,
                                                               int currIndex);

    static void drawContourPointsDependingOnItsCurvature(cv::Mat &dst, const std::vector<cv::Point> &contour,
                                                         const std::vector<double> &contourCurvature);

private:
    static int calculatingYThroughX(double x, double A, double B, double C);

};


#endif //TEST_FUNCTIONS_FOR_VKR_UTILS_H
