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

std::vector<FrameContours> RealDataTester::readContourFromTxtFile(const std::string &PATH)
{
    std::vector<FrameContours> vectorOfFrameContours;

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
            vectorOfFrameContours.emplace_back(FrameContours(currFrameNumber, std::move(contours)));

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

                if (prevPoint != cv::Point2f())
                {
                    addMissingPoints(currContour, currPoint, prevPoint);
                }

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

void RealDataTester::addMissingPoints(std::vector<cv::Point> &currContour, const cv::Point2f &currPoint,
                                      const cv::Point2f &prevPoint)
{
    const int DELTA = 2;

    cv::Point2f difference = currPoint - prevPoint;

    difference.x = std::abs(difference.x);
    difference.y = std::abs(difference.y);

    if (difference.x > DELTA || difference.y > DELTA)
    {
        const cv::Point directionalVector(currPoint.x - prevPoint.x, currPoint.y - prevPoint.y);

        double t = 0;

        while (std::abs(directionalVector.x * t + prevPoint.x - currPoint.x) > DELTA ||
               std::abs(directionalVector.y * t + prevPoint.y - currPoint.y) > DELTA)
        {
            t += 0.01; // 0.05 тоже внешне норм

            cv::Point2d newPoint;
            newPoint.x = directionalVector.x * t + prevPoint.x;
            newPoint.y = directionalVector.y * t + prevPoint.y;

            currContour.emplace_back(newPoint);
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
    const int ROWS = 800;
    const int COLS = 1500;
    const int TYPE = 16;
    const float RESIZE_FACTOR = 0.75;

    const auto vectorOfFrameContours = readContourFromTxtFile(PATH);

    const int FRAME_NUMBER = 42;
    std::vector<std::vector<cv::Point>> contours;
    chooseContourByFrameNumber(contours, vectorOfFrameContours, FRAME_NUMBER);

    removeDuplicatePointsFromContour(contours[3]);

    std::shared_ptr<RoadModel> roadModelPointer = std::make_shared<RoadModel>();
    RoadModelTracker modelTracker(roadModelPointer);

    std::vector<std::vector<double>> contoursCurvatures(contours.size());

    //int betterStep = contours[contourNumber].size() / 10;
    int betterStep = 1;

    for (int i = 0; i < contours.size(); ++i)
    {
        std::vector<double> tempCurvatures(contours[i].size());
        CurvatureCalculator::calculateCurvature2(tempCurvatures, contours[i], betterStep);

        contoursCurvatures[i] = std::move(tempCurvatures);
    }

    RoadModelBuilder::buildRoadModel(modelTracker, contours, contoursCurvatures, COLS);

    cv::Mat roadModelPicture(ROWS, COLS, TYPE, cv::Scalar(255, 255, 255));
    cv::Mat curvatureOnContourPicture(ROWS, COLS, TYPE, cv::Scalar(0, 0, 0));
    cv::Mat contourPointsPicture(ROWS, COLS, TYPE, cv::Scalar(0, 0, 0));

    for (int contourNumber = 3; contourNumber < contours.size(); ++contourNumber)
    {
        Drawer::drawContourPointsDependingOnItsCurvature(curvatureOnContourPicture,
                                                         contours[contourNumber],
                                                         contoursCurvatures[contourNumber]);
    }

    cv::resize(curvatureOnContourPicture, curvatureOnContourPicture, cv::Size(), RESIZE_FACTOR, RESIZE_FACTOR);
    cv::resize(roadModelPicture, roadModelPicture, cv::Size(), RESIZE_FACTOR, RESIZE_FACTOR);
    cv::resize(contourPointsPicture, contourPointsPicture, cv::Size(), RESIZE_FACTOR, RESIZE_FACTOR);


        //cv::imshow("contourDependingOnItsCurvature", curvatureOnContourPicture);
//        roadModelPointer->drawModelPoints(contourPointsPicture);
//        cv::imshow("modelPoints", contourPointsPicture);
        cv::waitKey(0);
}

void RealDataTester::chooseContourByFrameNumber(std::vector<std::vector<cv::Point>> &contour,
                                                const std::vector<FrameContours> &frameContours, const int FRAME_NUMBER)
{
    for (const auto &frameContour: frameContours)
    {
        if (frameContour.numberOfFrame == FRAME_NUMBER)
        {
            contour = frameContour.contours;
            return;
        }
    }
}

void RealDataTester::removeDuplicatePointsFromContour(std::vector<cv::Point> &contour)
{
    // for frame #42
    const int BEGIN_1 = 104;
    const int END_1 = 300;
    const int BEGIN_2 = 1777;
    const int END_2 = 1997;
    const int BEGIN_3 = 3537;
    const int END_3 = 6604;
    const int BEGIN_4 = 9498;

    std::vector<cv::Point> newContour;

    for (int i = 0; i < contour.size(); ++i)
    {
        if (i == BEGIN_1)
        {
            i = END_1;
            continue;
        }
        if (i == BEGIN_2)
        {
            i = END_2;
            continue;
        }
        if (i == BEGIN_3)
        {
            i = END_3;
            continue;
        }
        if (i == BEGIN_4)
        {
            break;
        }

        newContour.emplace_back(contour[i]);
    }

    contour = std::move(newContour);

    removeClumpedPoints(contour);
}

void RealDataTester::removeClumpedPoints(std::vector<cv::Point> &contour)
{
    std::vector<cv::Point> newContour;

    cv::Point prevPoint = contour[0];
    newContour.emplace_back(prevPoint);


    for (int i = 1; i < contour.size(); ++i)
    {
        if (contour[i] == prevPoint)
        {
            continue;
        }

        prevPoint = contour[i];
        newContour.emplace_back(prevPoint);
    }

    contour = std::move(newContour);
}

void RealDataTester::manualFindAndMarkDuplicatePoint(const std::vector<cv::Point> &contour, const int NUMBER_OF_CONTOUR)
{
    cv::Mat drawing(400, 500, 16, cv::Scalar(0, 0, 0));
    for (int i = 0; i < contour.size(); ++i)
    {
        cv::circle(drawing, cv::Point(contour[i].x - 700, contour[i].y), 1, cv::Scalar(255, 255, 255));

        cv::Mat resizedDrawing(drawing.clone());

        cv::resize(drawing, resizedDrawing, cv::Size(), 3, 3);

        cv::imshow("resizedDrawing", resizedDrawing);

        if (cv::waitKey(1) == 107)
        {
            std::cout << NUMBER_OF_CONTOUR << ": " << i << std::endl;
        }
    }

    drawing.release();
}


FrameContours::FrameContours(int numberOfFrame, std::vector<std::vector<cv::Point>> contour) :
        numberOfFrame(numberOfFrame), contours(std::move(contour))
{}
