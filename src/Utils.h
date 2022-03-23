//
// Created by alechh on 17.11.2021.
//

#ifndef TEST_FUNCTIONS_FOR_VKR_UTILS_H
#define TEST_FUNCTIONS_FOR_VKR_UTILS_H


#include <vector>
#include <opencv2/core/types.hpp>

class Utils {
public:
    static int remove_small_contours(std::vector< std::vector<cv::Point> > & contours, const int min_contours_size);

    static std::vector<double> calculate_curvature(std::vector<cv::Point> const& vecContourPoints, int step);

    static double mean_curvature(const std::vector<double>& curvature);

    static void sort_vector_of_vectors_of_points(std::vector<std::vector<cv::Point>>& contours);

    static void draw_contours(const std::vector<std::vector<cv::Point>>& contours, cv::Mat& input_frame, int number_of_contours);

    static void calculate_contours_curvature(std::vector<std::vector<double>> &contoursCurvature, const std::vector<std::vector<cv::Point>> &contours, int step);

    static std::vector<double> calculate_curvature_2(const std::vector<cv::Point> &contour, int step);

    static cv::Point2f getFirstDerivative(const cv::Point2f &pPlus, const cv::Point2f &pMinus, int iPlus, int iMinus);

    static void getPPlusAndPMinus(const std::vector<cv::Point> &segment, cv::Point &pPlus, cv::Point &pMinus, int &pPlusIndex, int &pMinusIndex, cv::Point &centerPoint);

    static std::vector<double> getCoefficientsOfTheTangent(const cv::Point &touchPoint, const cv::Point2f &firstDerivative);

    static std::vector<double> getCoefficientsOfThePerpendicularLine(const std::vector<double> &coefficientsOfTheLine, const cv::Point &touchPoint);

    static std::tuple<double, double> solutionOfTheSystemWithRespectToX(double A, double B, double C, double R, const cv::Point &point);

    static std::tuple<cv::Point, cv::Point> calculatingPointsOfStraightLineAtCertainDistanceFromGivenPoint(double A, double B, double C, double R, const cv::Point &point);

    static cv::Point chooseAmongTwoCandidatesForCenter(const std::vector<cv::Point> &segment, const std::tuple<cv::Point, cv::Point> &candidatesForCenter);

    static double calculationStartAndEndAnglesOfTheArc(double &startAngle, double &endAngle, const std::vector<cv::Point> &segment, const cv::Point &center, double radiusOfTheCircle);

private:
    static cv::Point getMidpoint(const cv::Point &a, const cv::Point &b);

    static double distanceBetweenPoints(const cv::Point &a, const cv::Point &b);

    static double getAngleOfTheArc(double &startAngle, double &endAngle, const std::vector<cv::Point> &segment, const cv::Point &center, double radiusOfTheCircle);

    static double calculateAngleShift(const cv::Point &A, const cv::Point &C);
};


#endif //TEST_FUNCTIONS_FOR_VKR_UTILS_H
