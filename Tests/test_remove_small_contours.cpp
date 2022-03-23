//
// Created by alechh on 06.12.2021.
//
#include "gtest/gtest.h"
#include "Utils.h"

TEST(RemoveSmallContours, CheckContoursSize)
{
    const int initial_contour_size = 100;
    std::vector< std::vector<cv::Point> > contours(initial_contour_size);

    srand(time(0));
    for (auto & contour : contours)
    {
        int curr_contour_size = rand() % 150;

        std::vector<cv::Point> curr_contour(curr_contour_size);

        for (int j = 0; j < curr_contour_size; ++j)
        {
            curr_contour[j] = cv::Point(0, 0);
        }

        contour = curr_contour;
    }

    const int min_contour_size = 30;
    int number_of_deleted_contours = Utils::removeSmallContours(contours, min_contour_size);

    EXPECT_EQ(initial_contour_size - number_of_deleted_contours, contours.size());

    for (const auto & i: contours)
    {
        EXPECT_GE(i.size(), min_contour_size);
    }
}
