//
// Created by alechh on 14.04.2022.
//

#ifndef TEST_FUNCTIONS_FOR_VKR_REALDATATESTER_H
#define TEST_FUNCTIONS_FOR_VKR_REALDATATESTER_H

struct FrameContour
{
    int numberOfFrame;
    std::vector<std::vector<cv::Point>> contour;

    FrameContour();

    FrameContour(int numberOfFrame, std::vector<std::vector<cv::Point>> contour);
};


class RealDataTester
{
public:
    static void extractContourFromVideo(const std::string &PATH);

    std::vector<FrameContour> readContourFromTxtFile(const std::string &PATH);

    static void buildRoadModelByContour(const std::string &PATH);

private:
    static cv::Point2f extractPointFromString(const std::string &s);

    static void
    addMissingPoints(std::vector<cv::Point2f> &currContour, const cv::Point2f &currPoint, const cv::Point2f &prevPoint);

    void scalePoint(cv::Point2f &p);
};


#endif //TEST_FUNCTIONS_FOR_VKR_REALDATATESTER_H
