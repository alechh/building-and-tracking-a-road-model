//
// Created by alechh on 14.04.2022.
//

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <fstream>
#include <iostream>
#include <utility>
#include "RealDataTester.h"
#include "Drawer.h"
#include "CurvatureCalculator.h"
#include "RoadModelBuilder.h"
#include "RoadModelTracker.h"

void RealDataTester::extractContourFromVideo(const std::string &PATH)
{
    cv::VideoCapture newFrameCap(PATH);
    if (!newFrameCap.isOpened())
    {
        return;
    }

    //    for (int i = 0; i < 700; ++i)
    //    {
    //        cv::Mat tmp_fr1;
    //        newFrameCap >> tmp_fr1;
    //    }

    int frame_width = newFrameCap.get(cv::CAP_PROP_FRAME_WIDTH);

    //обрезка капота
    int frame_height = newFrameCap.get(cv::CAP_PROP_FRAME_HEIGHT) - 300;
    int num = 0;
    bool empty = false;
    cv::Mat frame;
    while (!empty)
    {
        num++;

//        if (num > 500)
//        {
//          break;
//        }

        cv::Mat gray1;
        newFrameCap >> gray1;
        empty = gray1.rows == 0;

        if (empty)
        {
            break;
        }

        //cv::resize(gray1, gray1, cv::Size(gray1.cols / 2, gray1.rows / 2));
        cv::cvtColor(gray1, frame, cv::COLOR_BGR2GRAY);
        cv::Point2f center(frame.cols / 2.0, frame.rows / 2.0);
        cv::Mat rotationMatrix = cv::getRotationMatrix2D(center, -10, 1.0);
        cv::Mat rotatedImage;

        //убираем ролл
        cv::warpAffine(frame, rotatedImage, rotationMatrix, frame.size());
        rotatedImage = rotatedImage(cv::Rect(0, 0, frame_width, frame_height));
        //cv::imshow("frame", rotatedImage);

        //порог для выделения границ дороги и разметки
        cv::threshold(rotatedImage, rotatedImage, 117, 255, cv::THRESH_BINARY);

        //выделение границ
        cv::Canny(rotatedImage, rotatedImage, 100, 300, 5);

        //поиск и отрисовка контуров
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(rotatedImage, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        cv::Mat drawing = cv::Mat::zeros(rotatedImage.size(), CV_8UC3);
        for (size_t i = 0; i < contours.size(); i++)
        {
            cv::Scalar color = cv::Scalar(0, 0, 255);
            if (contours[i].size() < 120)
            {
                continue;
            }
            cv::drawContours(drawing, contours, static_cast<int>(i), color, 2, cv::LINE_8, hierarchy, 0);
        }

        cv::imshow("frame", drawing);
        cv::waitKey(1);
    }
    newFrameCap.release();
}

std::vector<FrameContour> RealDataTester::readContourFromTxtFile(const std::string &PATH)
{
    const int ROWS = 800;
    const int COLS = 1500;
    const float RESIZE_FACTOR = 1.0 / 100;

    std::vector<FrameContour> vectorOfFrameContours;

    std::ifstream in;

    in.open(PATH);

    std::vector<std::vector<cv::Point>> contours;
    int currFrameNumber = -1;

    std::string currString;
    while (getline(in, currString))
    {
        if (currFrameNumber == -1)
        {
            currFrameNumber = std::stoi(currString.substr(0, currString.find(':') - 1));
        }

        if (currString == "}") // end of the contours of the current frame
        {
            vectorOfFrameContours.emplace_back(FrameContour(currFrameNumber, std::move(contours)));

            currFrameNumber = -1;
            continue;
        }

        std::vector<cv::Point> currContour;

        int indexOfOpenBracket = -1;

        cv::Point2f prevPoint;
        for (int i = 0; i < currString.length(); ++i)
        {
            if (currString[i] == '(')
            {
                indexOfOpenBracket = i;
            }
            else if (currString[i] == ')')
            {
                std::string stringPoint = currString.substr(indexOfOpenBracket,
                                                            i - indexOfOpenBracket + 1);
                cv::Point2f currPoint = extractPointFromString(stringPoint);

                scalePoint(currPoint);

//                if (prevPoint != cv::Point2f())
//                {
//                    addMissingPoints(currContour, currPoint, prevPoint);
//                }

                currContour.emplace_back(currPoint);

                prevPoint = currPoint;
            }
        }

        contours.emplace_back(std::move(currContour));
    }
    return vectorOfFrameContours;
}

/**
 * "(1.223, 2.344)" -> cv::Point2f(1.223, 2.344)
 */
cv::Point2f RealDataTester::extractPointFromString(const std::string &s)
{
    cv::Point2f res;

    int indexOfComma = s.find(',');
    int indexOfClosedBracket = s.length() - 1;

    res.x = std::stof(s.substr(1, indexOfComma + 1));

    res.y = std::stof(s.substr(indexOfComma + 2, indexOfClosedBracket - indexOfComma - 2));

    return res;
}

void RealDataTester::addMissingPoints(std::vector<cv::Point2f> &currContour, const cv::Point2f &currPoint,
                                      const cv::Point2f &prevPoint)
{
    const int DELTA = 200;

    cv::Point2f difference = currPoint - prevPoint;

    difference.x = std::abs(difference.x);
    difference.y = std::abs(difference.y);

    if (difference.x > DELTA || difference.y > DELTA)
    {
        cv::Point directionalVector(currPoint.x - prevPoint.x, currPoint.y - prevPoint.y);

        double t = 0.1;

        while (std::abs(directionalVector.x * t + prevPoint.x - currPoint.x) > DELTA)
        {
            cv::Point2d newPoint;
            newPoint.x = directionalVector.x * t + prevPoint.x;
            newPoint.y = directionalVector.y * t + prevPoint.y;

            currContour.emplace_back(newPoint);

            t += 0.5;
        }
    }
}

void RealDataTester::scalePoint(cv::Point2f &p)
{
    const int SCALE_FACTOR = 15;
    const int SHIFT_X = 900;
    const int SHIFT_Y = 500;
    p.x *= SCALE_FACTOR;
    p.x += SHIFT_X;
    p.y *= -SCALE_FACTOR;
    p.y += SHIFT_Y;
}

void RealDataTester::buildRoadModelByContour(const std::string &PATH)
{

}

FrameContour::FrameContour() : numberOfFrame(0), contour(std::vector<std::vector<cv::Point>>())
{}

FrameContour::FrameContour(int numberOfFrame, std::vector<std::vector<cv::Point>> contour) :
        numberOfFrame(numberOfFrame), contour(std::move(contour))
{}
