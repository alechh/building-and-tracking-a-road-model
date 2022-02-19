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

    static std::vector<double> calculate_curvature(std::vector<cv::Point> const& vecContourPoints, int step = 1);

    static double mean_curvature(const std::vector<double>& curvature);

    static void sort_vector_of_vectors_of_points(std::vector<std::vector<cv::Point>>& contours);

    static void draw_contours(const std::vector<std::vector<cv::Point>>& contours, cv::Mat& input_frame, int number_of_contours);

    static void calculate_contours_curvature(std::vector<std::vector<double>> &contoursCurvature, const std::vector<std::vector<cv::Point>> &contours);
};


#endif //TEST_FUNCTIONS_FOR_VKR_UTILS_H
