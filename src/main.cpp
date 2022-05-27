#include "opencv2/imgproc.hpp"
#include <vector>
#include "RoadModel.h"
#include <iostream>
#include <string>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "Utils.h"
#include "RoadModelBuilder.h"
#include "ContourBuilder.h"
#include "CurvatureCalculator.h"
#include "Drawer.h"
#include "RoadModelTracker.h"
#include "RealDataTester.h"


void
calculateMeanError(double &minCurvatureError, int &minStepCurvature, const double EXACT_CURVATURE, const int CUR_STEP,
                   const std::vector<double> &contourCurvature)
{
    double meanCurvature = 0;
    int count = 0;
    for (auto i: contourCurvature)
    {
        if (i != 0)
        {
            meanCurvature += i;
            ++count;
        }
    }

    meanCurvature /= count;

    if (std::abs(meanCurvature - EXACT_CURVATURE) < minCurvatureError)
    {
        minCurvatureError = std::abs(meanCurvature - EXACT_CURVATURE);
        minStepCurvature = CUR_STEP;
    }
}

void buildRoadModelByContour()
{
    const int ROWS = 500;
    const int COLS = 1000;
    const int TYPE = 16;

    const std::shared_ptr<RoadModel> roadModelPointer = std::make_shared<RoadModel>();
    RoadModelTracker modelTracker(roadModelPointer);

    for (int t = 0; t < 1; ++t) // < 460
    {
        const std::vector<std::vector<cv::Point>> contours = ContourBuilder::getRightAndLeftContours(80, 100, 150, 300,
                                                                                                     t);
        cv::Mat contoursTImage(ROWS, COLS, TYPE, cv::Scalar(0, 0, 0));

        for (const auto &i: contours)
        {
            for (const auto &j: i)
            {
                cv::circle(contoursTImage, j, 1, cv::Scalar(255, 255, 255));
            }
        }

        const double EXACT_CURVATURE = 0.01;

        Drawer::drawContoursOnImage(contours);

        std::vector<std::vector<double>> contoursCurvatures(contours.size());

        cv::Mat roadModelPicture(ROWS, COLS, TYPE, cv::Scalar(255, 255, 255));

        double minCurvatureError = 10000;
        int minStepCurvature = 0;

        for (int iStep = 10; iStep <= 10; ++iStep)
        {
            //std::cout << "iStep = " << iStep << std::endl << contours[0].size() << std::endl;

            cv::Mat curvatureOnContourPicture(ROWS, COLS, TYPE, cv::Scalar(0, 0, 0));
            cv::Mat contourPointsPicture(ROWS, COLS, TYPE, cv::Scalar(0, 0, 0));

            for (int i = 0; i < contours.size(); ++i)
            {
                int betterStep = contours[i].size() / 10;

                std::vector<double> tempCurvatureContour(contours[i].size());
                CurvatureCalculator::calculateCurvature2(tempCurvatureContour, contours[i], betterStep);

                contoursCurvatures[i] = std::move(tempCurvatureContour);


//                Drawer::drawContourPointsDependingOnItsCurvature(curvatureOnContourPicture, contours[i],
//                                                                 contoursCurvatures[i], 1);
            }

            Drawer::drawContoursPointByPoint(roadModelPicture, contours, false);

            RoadModelBuilder::buildRoadModel(modelTracker, contours, contoursCurvatures, COLS, 1);

            modelTracker.getRoadModelPointer()->drawModel(roadModelPicture);
            cv::imshow("model", roadModelPicture);
            cv::waitKey(0);
            imwrite(cv::format("../images/result.jpg", t), roadModelPicture);

            //roadModelPointer->printInformationOfTheModel();

            //imwrite(cv::format("../images/curvatureOnContourPicture/curvatureContour%d.jpg", iStep),
            //        curvatureOnContourPicture);

            roadModelPointer->drawModelPoints(contourPointsPicture);
            //imwrite("../images/showModelPoints/modelPoints.jpg", contourPointsPicture);

            roadModelPicture.release();
        }
    }


//    std::cout << std::endl << "Min curvature error = " << minCurvatureError << "\niStep = " << minStepCurvature
//              << std::endl;
}


void testModelBuildingOnRealData()
{
    const std::string PATH_VIDEO = "../videos/realData.MP4";
    const std::string PATH_TXT = "../videos/bv_points.txt";
//    RealDataTester::extractContourFromVideo(PATH_VIDEO);

    RealDataTester::buildRoadModelByContour(PATH_TXT);
}


void testDirectionalVector()
{
    cv::Mat src = cv::imread("../videos/testDirectionalVector.png");
    cv::Mat srcGray(src.clone());
    cv::cvtColor(src, srcGray, cv::COLOR_BGR2GRAY);

    std::vector<std::vector<cv::Point>> contour;
    cv::findContours(srcGray, contour, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

    cv::Mat drawing(src.rows, src.cols, src.type(), cv::Scalar(255, 255, 255));

//    for (int i = 0; i < 10; ++i)
//    {
//        contour.emplace_back(cv::Point(i, 0));
//    }
//    contour.emplace_back(cv::Point(9, 1));
//    contour.emplace_back(cv::Point(9, 2));
//    contour.emplace_back(cv::Point(9, 1));

    cv::Point prevPoint = contour[1][0];
    cv::Point prevPrevPoint = contour[1][0];

    for (int i = 0; i < contour[1].size(); ++i)
    {
        cv::Point directionalVector(contour[1][i].x - prevPrevPoint.x, contour[1][i].y - prevPrevPoint.y);
        std::cout << directionalVector << std::endl;

        cv::circle(drawing, contour[1][i], 1, cv::Scalar(0, 0, 0));
        cv::imshow("drawing", drawing);
        if (cv::waitKey(25) == 107)
        {
            cv::waitKey(0);
        }


        prevPrevPoint = prevPoint;
        prevPoint = contour[1][i];
    }
}

void testAngelOfTriangleCalculation()
{
    cv::Point prevPrevPoint(0, 0);
    cv::Point prevPoint(1, -1);
    cv::Point currPoint(1, 0);

    std::cout << Utils::calculateAngleOfTriangle(prevPrevPoint, prevPoint, currPoint) << std::endl;
}


void testExtractingEllipse()
{
    cv::Mat drawing(500, 1000, 16, cv::Scalar(0, 0, 0));
    cv::Mat drawing2(500, 1000, 16, cv::Scalar(0, 0, 0));
    cv::ellipse(drawing, cv::Point(500, 250), cv::Size(100, 100), 0, 90, 225, cv::Scalar(255, 255, 255));

    cv::cvtColor(drawing, drawing, cv::COLOR_BGR2GRAY);

    std::vector<std::vector<cv::Point>> contour;

    cv::findContours(drawing, contour, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

    std::cout << contour[0].size() << std::endl;



    for (const auto &point: contour[0])
    {
        cv::circle(drawing2, point, 1, cv::Scalar(255, 129, 21));
    }

    cv::imshow("drawing", drawing2);
    cv::waitKey(0);
}


void testDataExtraction()
{
    const std::string PATH_TXT = "../output.txt";

    RealDataTester::buildRoadModelByContour(PATH_TXT);
}


int main()
{
    //buildRoadModelByContour();
    //testModelBuildingOnRealData();
    //testDirectionalVector();
    //testAngelOfTriangleCalculation();
    testDataExtraction();

    //testExtractingEllipse();
    return 0;
}