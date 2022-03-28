//
// Created by alechh on 17.11.2021.
//

#ifndef TEST_FUNCTIONS_FOR_VKR_UTILS_H
#define TEST_FUNCTIONS_FOR_VKR_UTILS_H


#include <vector>
#include <opencv2/core/types.hpp>

class Utils {
public:
    static int removeSmallContours(std::vector< std::vector<cv::Point> > & contours, const int minContoursSize);

    static void
    calculateCurvature(std::vector<double> &contourCurvature, const std::vector<cv::Point> &vecContourPoints,
                       int step);

    static double meanCurvature(const std::vector<double>& curvature);

    static void sortVectorOfVectorsOfPoints(std::vector<std::vector<cv::Point>>& contours);

    static void draw_contours(const std::vector<std::vector<cv::Point>>& contours, cv::Mat& input_frame, int number_of_contours);

    static void calculateContoursCurvature(std::vector<std::vector<double>> &contoursCurvature, const std::vector<std::vector<cv::Point>> &contours, int step);

    static void
    calculateCurvature2(std::vector<double> &contourCurvature, const std::vector<cv::Point> &contour,
                        const int step);

    static cv::Point2f
    getFirstDerivative(const cv::Point2f &pPlus, const cv::Point2f &pMinus, int iPlus, int iMinus, double h);

    static void
    getPPlusAndPMinus(const std::vector<cv::Point> &segment, cv::Point &pPlus, cv::Point &pMinus, int &pPlusIndex,
                      int &pMinusIndex, cv::Point &centerPoint, double &h);

    static std::vector<double> getCoefficientsOfTheTangent(const cv::Point &touchPoint, const cv::Point2f &firstDerivative);

    static std::vector<double> getCoefficientsOfThePerpendicularLine(const std::vector<double> &coefficientsOfTheLine, const cv::Point &touchPoint);

    static std::tuple<double, double> solutionOfTheSystemWithRespectToX(double A, double B, double C, double R, const cv::Point &point);

    static std::tuple<cv::Point, cv::Point> calculatingPointsOfStraightLineAtCertainDistanceFromGivenPoint(double A, double B, double C, double R, const cv::Point &point);

    static cv::Point chooseAmongTwoCandidatesForCenter(const std::vector<cv::Point> &segment, const std::tuple<cv::Point, cv::Point> &candidatesForCenter);

    static void calculationStartAndEndAnglesOfTheArc(double &startAngle, double &endAngle, const std::vector<cv::Point> &segment, const cv::Point &center, double radiusOfTheCircle);

    static double distanceBetweenPoints(const cv::Point &A, const cv::Point &B);

private:
    static cv::Point getMidpoint(const cv::Point &a, const cv::Point &b);

    static double getAngleOfTheArc(const std::vector<cv::Point> &segment, const cv::Point &center, double radiusOfTheCircle);

    static double calculateAngleShiftUpper(const cv::Point &firstPointOfTheSegment, const cv::Point &circleCenter);

    static double calculateAngleShiftLower(const cv::Point &lastPointOfTheSegment, const cv::Point &circleCenter, double radiusOfTheCircle);

    static void selectionOfPointsDependingOnTheStep(cv::Point &prev, cv::Point &next, const std::vector<cv::Point> &contour, const int step, const int currIndex);

};


#endif //TEST_FUNCTIONS_FOR_VKR_UTILS_H
