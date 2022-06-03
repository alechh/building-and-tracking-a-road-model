#include <vector>
#include "RoadModel.h"
#include <iostream>
#include <string>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc.hpp"
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


void testDataExtraction(const cv::Mat &reversePerspectiveTransformMatrix)
{
    const std::string PATH_TXT = "../output.txt";

    RealDataTester::buildRoadModelByContour(PATH_TXT);
}


std::vector<std::vector<cv::Point>> extractContoursFromMat(const cv::Mat &src, cv::Mat &reversePerspectiveTransformMatrix);


void restorePoint(cv::Mat &reverseMatrix, std::vector<cv::Point2f> &modelPoints)
{
    // src.rows = 1080
    // src.cols = 1920

    //modelPoint.y += 1080 / 2;

    // bird view
    std::vector<cv::Point2f> resModelPoints;
    cv::perspectiveTransform(modelPoints, resModelPoints, reverseMatrix);

    modelPoints = std::move(resModelPoints);

    //modelPoint.x += 810;

}


void translateImg(cv::Mat &img, int offsetx, int offsety)
{
    cv::Mat trans_mat = (cv::Mat_<double>(2,3) << 1, 0, offsetx, 0, 1, offsety);
    warpAffine(img, img, trans_mat, img.size());
}



void testBirdViewReverse(cv::Mat &src);
std::vector<std::vector<cv::Point>> extractContours(const cv::Mat &src);

void makeVideoPresentation()
{
    const std::string PATH = "../video.mp4";
    const double POINTS_FACTOR = 1;

    cv::VideoCapture capture(PATH);
    if (!capture.isOpened())
    {
        std::cerr << "Error" << std::endl;
        return;
    }

    cv::Mat src, srcCopy;
    int frameNumber = 0;

    cv::VideoWriter videoWriter("../output.mp4", cv::VideoWriter::fourcc('M','J','P','G'), 10, cv::Size(1920, 1080));

    while (true)
    {
//        if (frameNumber < 224)
//        {
//            capture >> src;
//            ++frameNumber;
//            continue;
//        }
        std::cout << frameNumber++ << std::endl;

        capture >> src;

        //testBirdViewReverse(src);

        src.copyTo(srcCopy);

        cv::Mat reversePerspectiveTransformMatrix;

        std::vector<std::vector<cv::Point>> contours = extractContoursFromMat(src, reversePerspectiveTransformMatrix);

        if (contours[0].empty())
        {
            continue;
        }

        for (auto &contour : contours)
        {
            RealDataTester::removeClumpedPoints(contour);
        }

        std::vector<std::vector<double>> contoursCurvatures(contours.size());

        CurvatureCalculator::calculateCurvatureForAllContours(contoursCurvatures, contours, 11);

        cv::Mat curvaturePicture(src.rows / 2 - 100, src.cols, src.type(), cv::Scalar(255, 255, 255));
        Drawer::drawContoursPointsDependingOnItsCurvatures(curvaturePicture, contours, contoursCurvatures, false, POINTS_FACTOR);

        //cv::imshow("curvature", curvaturePicture);

        std::shared_ptr<RoadModel> roadModelPointer = std::make_shared<RoadModel>();
        RoadModelTracker modelTracker(roadModelPointer);

        RoadModelBuilder::buildRoadModel(modelTracker, contours, contoursCurvatures, 1920,
                                         POINTS_FACTOR);

        //cv::Mat roadModelPicture(src.rows / 2 - 100, src.cols, src.type(), cv::Scalar(255, 255, 255));
        cv::Mat roadModelPicture(src.rows, src.cols, src.type(), cv::Scalar(0, 0, 0));
        cv::Mat roadModelPicture2(src.rows, src.cols, src.type(), cv::Scalar(0, 0, 0));

        modelTracker.getRoadModelPointer()->drawModel(roadModelPicture);

        warpPerspective(roadModelPicture, roadModelPicture2, reversePerspectiveTransformMatrix, roadModelPicture2.size());



        //std::vector<cv::Point2f> modelPoints = modelTracker.getRoadModelPointer()->getModelPoints();

        //restorePoint(reversePerspectiveTransformMatrix, modelPoints);

//        for (const auto &point : modelPoints)
//        {
//            cv::circle(roadModelPicture, cv::Point(point.x + 810, point.y), 1, cv::Scalar(0, 0, 0));
//        }

        translateImg(roadModelPicture2, 0, 540);

       cv::Mat_<cv::Vec3b> _I = roadModelPicture2;
       cv::Mat_<cv::Vec3b> _ISrc = srcCopy;

        for( int i = 0; i < roadModelPicture2.rows; ++i)
            for( int j = 0; j < roadModelPicture2.cols; ++j )
            {
                if (_I(i, j) != cv::Vec3b(0, 0, 0))
                {
                    _ISrc(i, j) = _I(i, j);
                }
            }

        videoWriter.write(srcCopy);
        cv::imshow("model", srcCopy);

        int k = cv::waitKey(25);
        if (k == 27)
        {
            break;
        }
    }
}


cv::Mat build_bird_view(const cv::Mat& frame_roi, cv::Mat &reversePerspectiveMatrix)
{
    cv::Point pt1(673, 2);
    cv::Point pt2(980, 2);
    cv::Point pt3(1, 177);
    cv::Point pt4(1186, 177);

    int delta = 60; // 40 for PATH1 , 50 for PATH2

    int x_offset = 7; // 50 for PATH1, 0 for PATH2

    cv::Point pt11(pt1.x, pt1.y);
    cv::Point pt22(pt2.x, pt2.y);
    cv::Point pt33(pt1.x, pt3.y);
    cv::Point pt44(pt2.x, pt4.y);

    std::vector<cv::Point2f> dst_corners(4), source(4);

    source[0] = pt1;
    source[1] = pt2;
    source[2] = pt3;
    source[3] = pt4;

    dst_corners[0] = pt11;
    dst_corners[1] = pt22;
    dst_corners[2] = pt33;
    dst_corners[3] = pt44;

    cv::Mat M = getPerspectiveTransform(source, dst_corners);

    reversePerspectiveMatrix = getPerspectiveTransform(dst_corners, source);

    cv::Mat warped_image(frame_roi.rows, frame_roi.cols, frame_roi.type());
    warpPerspective(frame_roi, warped_image, M, warped_image.size()); // do perspective transformation

    //imshow("Warped Image", warped_image);
    return warped_image;
}


std::vector<std::vector<cv::Point>> extractContoursFromMat(const cv::Mat &src, cv::Mat &reversePerspectiveTransformMatrix)
{
    cv::Mat srcContrast, srcBlur;

    cv::medianBlur(src, srcBlur, 5);

    srcBlur.convertTo(srcContrast, -1, 2, 0);

    cv::Rect roi = cv::Rect(0, src.rows / 2, src.cols, src.rows / 2 - 100);

    cv::Mat roiImg;
    roiImg = srcContrast(roi);

    cv::Mat bird = build_bird_view(roiImg, reversePerspectiveTransformMatrix);

    cv::Rect rectBird = cv::Rect(810, 0, 200, bird.rows);

    cv::Mat roiBird = bird(rectBird);

    cv::Mat src_canny;

    cv::Mat src_hsv = cv::Mat(roiBird.cols, roiBird.rows, 8, 3);
    std::vector<cv::Mat> splitedHsv = std::vector<cv::Mat>();
    cvtColor(roiBird, src_hsv, cv::COLOR_RGB2HSV);
    split(src_hsv, splitedHsv);

    int sensivity = 20;

    int S_WHITE_MIN = 0;
    int V_WHITE_MIN = 255 - sensivity;

    int S_WHITE_MAX = sensivity;
    int V_WHITE_MAX = 255;


    for (int y = 0; y < src_hsv.cols; y++)
    {
        for (int x = 0; x < src_hsv.rows; x++)
        {
            // получаем HSV-компоненты пикселя
            int H = static_cast<int>(splitedHsv[0].at<uchar>(x, y));        // Тон
            int S = static_cast<int>(splitedHsv[1].at<uchar>(x, y));        // Интенсивность
            int V = static_cast<int>(splitedHsv[2].at<uchar>(x, y));        // Яркость

            if (!(S_WHITE_MIN <= S && S <= S_WHITE_MAX && V_WHITE_MIN <= V && V <= V_WHITE_MAX))
            {
                src_hsv.at<cv::Vec3b>(x, y)[0] = 0;
                src_hsv.at<cv::Vec3b>(x, y)[1] = 0;
                src_hsv.at<cv::Vec3b>(x, y)[2] = 0;
            }
        }
    }

    Canny(src_hsv, src_canny, 300, 320);

    //cv::imshow("src_canny", src_canny);

    std::vector<std::vector<cv::Point>> contours;

    cv::findContours(src_canny, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

    int contourSizeBefor = contours.size();

    for (auto iter = contours.begin(); iter != contours.end(); ++iter)
    {
        if ((*iter).size() < 30)
        {
            contours.erase(iter);
            --iter;
        }
    }

    //std::cout << "remove contours : " << contourSizeBefor - contours.size() << std::endl;

    std::vector<std::vector<cv::Point>> niceContours(1);


    for (const auto &contour : contours)
    {
        for (const auto &point : contour)
        {
            niceContours[0].emplace_back(point);
        }
    }
    //return contours;
    return niceContours;
}



void testBirdViewReverse(cv::Mat &src)
{
    cv::Point pt1(673, 2);
    cv::Point pt2(980, 2);
    cv::Point pt3(1, 177);
    cv::Point pt4(1186, 177);

    int delta = 60; // 40 for PATH1 , 50 for PATH2

    int x_offset = 7; // 50 for PATH1, 0 for PATH2

    cv::Point pt11(pt1.x, pt1.y);
    cv::Point pt22(pt2.x, pt2.y);
    cv::Point pt33(pt1.x, pt3.y);
    cv::Point pt44(pt2.x, pt4.y);

    std::vector<cv::Point2f> dst_corners(4), source(4);

    source[0] = pt1;
    source[1] = pt2;
    source[2] = pt3;
    source[3] = pt4;

    dst_corners[0] = pt11;
    dst_corners[1] = pt22;
    dst_corners[2] = pt33;
    dst_corners[3] = pt44;

    std::vector<cv::Point> points(100);
    std::vector<cv::Point> warpPoints;
    for (int i = 0; i < 100; ++i)
    {
        points.emplace_back(cv::Point(src.cols / 2 + i, src.rows / 2));
    }

    for (const auto &point : points)
    {
        cv::circle(src, point, 1, cv::Scalar(0, 0, 0));
    }

    cv::Mat M = getPerspectiveTransform(source, dst_corners);

    cv::Mat reversePerspectiveMatrix = getPerspectiveTransform(dst_corners, source);

    cv::Mat warped_image(src.rows, src.cols, src.type());
    warpPerspective(src, warped_image, M, warped_image.size()); // do perspective transformation

    cv::perspectiveTransform(points, warpPoints, M);

    for (const auto &point : warpPoints)
    {
        cv::circle(src, point, 1, cv::Scalar(0, 0, 0));
    }

    cv::Mat reverseWarped(src.rows, src.cols, src.type());
    warpPerspective(warped_image, reverseWarped, M, warped_image.size());

    imshow("src", src);
    //imshow("Warped Image", reverseWarped);
    cv::waitKey(0);
}


int main()
{
    makeVideoPresentation();
    return 0;
}