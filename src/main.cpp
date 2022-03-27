#include <iostream>
#include <string>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "Utils.h"
#include "ExperimentWithCurvatureCalculation.h"
#include "contourBuilder.h"

void calculateMeanError(double &minCurvatureError, int &minStepCurvature, const int CUR_STEP, const std::vector<double> &contourCurvature)
{
    double meanCurvature = 0;
    int count = 0;
    const double EXACT_CURVATURE = 0.01;
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

    std::vector< std::vector<cv::Point> > contours;

    contours.emplace_back(contourBuilder::getSimpleRightContour2());

    contourBuilder::saveContoursOnImage(contours);

    std::vector< std::vector<double> > contoursCurvatures(contours.size());

    cv::Mat roadModelPicture(ROWS, COLS, TYPE, cv::Scalar(0, 0, 0));

    double minCurvatureError = 10000;
    int minStepCurvature = 0;

    for (int iStep = 190; iStep <= 190; ++iStep)
    {
        //std::cout << "iStep = " << iStep << std::endl << contours[0].size() << std::endl;

        RoadModel roadModel;

        cv::Mat curvatureOnContourPicture(ROWS, COLS, TYPE, cv::Scalar(0, 0, 0));
        cv::Mat contourPointsPicture(ROWS, COLS, TYPE, cv::Scalar(0, 0, 0));

        for (int i = 0; i < contours.size(); ++i)
        {
            int betterStep = contours[i].size() / 10;

            std::vector<double> tempCurvatureContour(contours[i].size());
            Utils::calculateCurvature2(tempCurvatureContour, contours[i], betterStep);

            contoursCurvatures[i] = std::move(tempCurvatureContour);

            ExperimentWithCurvatureCalculation::buildRoadModelBasedOnTheSingleContour(roadModel, contours[i], contoursCurvatures[i]);

            ExperimentWithCurvatureCalculation::showCurvatureOnImage(curvatureOnContourPicture, contours[i], contoursCurvatures[i]);

            calculateMeanError(minCurvatureError, minStepCurvature, iStep, contoursCurvatures[i]);
        }

        roadModel.drawModel(roadModelPicture);
        imwrite("../images/frameRoadModel.jpg", roadModelPicture);

        //roadModel.printInformationOfTheRightSide();

        imwrite("../images/curvatureOnContourPicture/curvatureContour.jpg", curvatureOnContourPicture);

        roadModel.drawModelPoints(contourPointsPicture);
        imwrite("../images/showModelPoints/modelPoints.jpg", contourPointsPicture);
    }

    std::cout << std::endl << "Min curvature error = " << minCurvatureError << "\niStep = " << minStepCurvature << std::endl;
}


int main()
{
    buildRoadModelByContour();

    return 0;
}
