//
// Created by alechh on 17.11.2021.
//

#ifndef TEST_FUNCTIONS_FOR_VKR_UTILS_H
#define TEST_FUNCTIONS_FOR_VKR_UTILS_H


#include <vector>
#include <opencv2/core/types.hpp>

class Utils {
public:
    std::vector<double> calculate_curvature(std::vector<cv::Point> const& vecContourPoints, int step);
};


#endif //TEST_FUNCTIONS_FOR_VKR_UTILS_H
