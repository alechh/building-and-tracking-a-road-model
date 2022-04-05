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

    std::vector<std::vector<cv::Point> > contours;
    for (int t = 0; t < 460; ++t)
    {
        std::cout << t << std::endl;
        std::vector<std::vector<cv::Point> > contoursT = ContourBuilder::getRightAndLeftContours(80, 100, 150, 300, t);
        cv::Mat contoursTImage(ROWS, COLS, TYPE, cv::Scalar(0, 0, 0));

        for (const auto &i: contoursT)
        {
            for (const auto & j : i)
            {
                cv::circle(contoursTImage, j, 1, cv::Scalar(255, 255, 255));
            }
        }

        cv::imshow("image", contoursTImage);
        int k = cv::waitKey(25);
    }


    const double EXACT_CURVATURE = 0.01;

    Drawer::drawContoursOnImage(contours);

    std::vector<std::vector<double> > contoursCurvatures(contours.size());

    cv::Mat roadModelPicture(ROWS, COLS, TYPE, cv::Scalar(255, 255, 255));

    double minCurvatureError = 10000;
    int minStepCurvature = 0;

    for (int iStep = 10; iStep <= 10; ++iStep)
    {
        //std::cout << "iStep = " << iStep << std::endl << contours[0].size() << std::endl;

        RoadModel roadModel;

        cv::Mat curvatureOnContourPicture(ROWS, COLS, TYPE, cv::Scalar(0, 0, 0));
        cv::Mat contourPointsPicture(ROWS, COLS, TYPE, cv::Scalar(0, 0, 0));

        for (int i = 0; i < contours.size(); ++i)
        {
            int betterStep = contours[i].size() / 10;

            std::vector<double> tempCurvatureContour(contours[i].size());
            CurvatureCalculator::calculateCurvature2(tempCurvatureContour, contours[i], betterStep);

            contoursCurvatures[i] = std::move(tempCurvatureContour);

            bool isRightContour;
            if (contours[i][0].x < COLS / 2)
            {
                isRightContour = false;
            }
            else
            {
                isRightContour = true;
            }

            RoadModelBuilder::buildRoadModelBasedOnTheSingleContour(roadModel, contours[i],
                                                                    contoursCurvatures[i],
                                                                    isRightContour);

            Drawer::drawContourPointsDependingOnItsCurvature(curvatureOnContourPicture, contours[i],
                                                             contoursCurvatures[i]);

            calculateMeanError(minCurvatureError, minStepCurvature, EXACT_CURVATURE, iStep, contoursCurvatures[i]);
        }

        roadModel.drawModel(roadModelPicture);
        imwrite("../images/frameRoadModel.jpg", roadModelPicture);

        roadModel.printInformationOfTheModel();

        imwrite(cv::format("../images/curvatureOnContourPicture/curvatureContour%d.jpg", iStep),
                curvatureOnContourPicture);

        roadModel.drawModelPoints(contourPointsPicture);
        imwrite("../images/showModelPoints/modelPoints.jpg", contourPointsPicture);
    }

    std::cout << std::endl << "Min curvature error = " << minCurvatureError << "\niStep = " << minStepCurvature
              << std::endl;
}


int main()
{
    buildRoadModelByContour();

    return 0;
}