//
// Created by alechh on 14.04.2022.
//

#ifndef TEST_FUNCTIONS_FOR_VKR_REALDATATESTER_H
#define TEST_FUNCTIONS_FOR_VKR_REALDATATESTER_H

struct FrameContours
{
    int numberOfFrame;
    std::vector<std::vector<cv::Point>> contours;

    FrameContours(int numberOfFrame, std::vector<std::vector<cv::Point>> contour);
};


class RealDataTester
{
public:
    static void buildRoadModelByContour(const std::string &PATH);

private:
    static cv::Point2f extractPointFromString(const std::string &s);

    static void
    addMissingPoints(std::vector<cv::Point> &currContour, const cv::Point2f &currPoint, const cv::Point2f &prevPoint);

    static void scalePoint(cv::Point2f &p);

    static std::vector<FrameContours> readContourFromTxtFile(const std::string &PATH);

    static void extractContourFromVideo(const std::string &PATH);

    static void chooseContourByFrameNumber(std::vector<std::vector<cv::Point>> &contour,
                                           const std::vector<FrameContours> &frameContours, int FRAME_NUMBER);

    static void removeDuplicatePointsFromContour(std::vector<cv::Point> &contour, int FRAME_NUMBER);

    static void removeClumpedPoints(std::vector<cv::Point> &contour);

    static void manualFindAndMarkDuplicatePoint(const std::vector<cv::Point> &contour, int NUMBER_OF_CONTOUR);

    static void removeDuplicatePointsFromContours(std::vector<std::vector<cv::Point>> &contours, int FRAME_NUMBER);

    static void removeLocallyIdenticalContourPoints(std::vector<cv::Point> &contour);
};


#endif //TEST_FUNCTIONS_FOR_VKR_REALDATATESTER_H
