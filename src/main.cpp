#include <iostream>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include "Utils.h"

using namespace cv;


/**
 * Get ROI of the Image
 * @param src -- Mat image
 * @return Mat dst -- Rect(0, y_coordinate, IMAGE_WIDTH, IMAGE_HEIGHT - y_coordinate)
 */
Mat get_horizontal_roi(const Mat& src)
{
    Mat dst;
    int delta = 0; // for PATH1 100, for PATH2 0, for PATH3 0 -- чтобы обрезать капот на первом видео
    int y_coordinate = 780; // for PATH 430, for PATH2 500, for PATH3 585, for PATH4 (resize 0.4) 600
    Rect roi(0, y_coordinate, src.cols, src.rows - y_coordinate - delta);
    dst = src(roi);

    return dst;
}


Mat get_vertical_roi(const Mat& src)
{
    Mat dst;
    int delta = 120; // 50 for PATH1, 70 (50?) for PATH2, 40 for PATH3, 45 for PATH4 (resize 0.4)
    int x_offset = 30; // 30 for PATH1, 0 (20?) for PATH2, 10 for PATH3, 20 for PATH4 (resize 0.4)
    int x_coordinate = src.cols / 2 - x_offset;
    Rect roi(x_coordinate - delta, 0, 2 * delta, src.rows);
    dst = src(roi);

    return dst;
}


Point get_vanishing_point()
{
    return {595, 422};  // for PATH1
}


Mat build_bird_view(const Mat& frame_roi)
{

    Point pt1(0, frame_roi.rows);
    Point pt2(frame_roi.cols, frame_roi.rows);
    Point pt3(0, 0);
    Point pt4(frame_roi.cols, 0);

    int delta = 90; // 40 for PATH1 , 50 for PATH2, 58 for PATH3, 40 for PATH4 (resize 0.4)

    int x_offset = 20; // 50 for PATH1, 0 for PATH2, 7 for PATH3, 20 for PATH4 (resize 0.4)

    Point pt11(frame_roi.cols / 2 - delta - x_offset, frame_roi.rows);
    Point pt22(frame_roi.cols / 2 + delta - x_offset, frame_roi.rows);
    Point pt33(0, 0);
    Point pt44(frame_roi.cols, 0);

    std::vector<Point2f> dst_corners(4), source(4);

    source[0] = pt1;
    source[1] = pt2;
    source[2] = pt3;
    source[3] = pt4;

    dst_corners[0] = pt11;
    dst_corners[1] = pt22;
    dst_corners[2] = pt33;
    dst_corners[3] = pt44;

    Mat M = getPerspectiveTransform(source, dst_corners);

    Mat warped_image(frame_roi.rows, frame_roi.cols, frame_roi.type());
    warpPerspective(frame_roi, warped_image, M, warped_image.size()); // do perspective transformation

    //imshow("Warped Image", warped_image);
    return warped_image;
}

/**
 * Pausing video by pressing "k" button
 * @param k
 */
void pause(int k)
{
    if (k == 107)
    {
        while (true)
        {
            int l = waitKey(0);
            if (l == 107)
            {
                return;
            }
        }

    }
}


Mat find_white_color(const Mat& src)
{
    Mat src_hsv = Mat(src.cols, src.rows, 8, 3);
    std::vector<Mat> splitedHsv = std::vector<Mat>();
    cvtColor(src, src_hsv, COLOR_RGB2HSV);
    split(src_hsv, splitedHsv);

    int sensivity = 100; // 120

    int S_WHITE_MIN = 0;
    int V_WHITE_MIN = 255 - sensivity;

    int S_WHITE_MAX = sensivity;
    int V_WHITE_MAX = 255;


    for (int y = 0; y < src_hsv.cols; y++)
    {
        for (int x = 0; x < src_hsv.rows; x++)
        {
            // получаем HSV-компоненты пикселя
            int S = static_cast<int>(splitedHsv[1].at<uchar>(x, y));        // Интенсивность
            int V = static_cast<int>(splitedHsv[2].at<uchar>(x, y));        // Яркость

            if (!(S_WHITE_MIN <= S && S <= S_WHITE_MAX && V_WHITE_MIN <= V && V <= V_WHITE_MAX))
            {
                src_hsv.at<Vec3b>(x, y)[0] = 0;
                src_hsv.at<Vec3b>(x, y)[1] = 0;
                src_hsv.at<Vec3b>(x, y)[2] = 0;
            }
        }
    }
    return src_hsv;
}


void find_lines_birdview(const std::string& PATH)
{
    VideoCapture capture(PATH);
    if (!capture.isOpened())
    {
        std::cerr << "Error" << std::endl;
        return;
    }

    Mat frame;

    while (true)
    {
        capture >> frame;

        Mat frame_birdview = build_bird_view(frame);

        Mat vertical_roi_birdview = get_vertical_roi(frame_birdview);

        Mat white_hsv = find_white_color(vertical_roi_birdview);

        Mat frame_canny;
        Canny(white_hsv, frame_canny, 170, 200);

        //imshow("source", frame);
        //imshow("birdview", frame_birdview);
        imshow("vertical_roi_birdview", vertical_roi_birdview);
        imshow("canny", frame_canny);

        int k = waitKey(24);
        pause(k);

        if (k == 27)
        {
            break;
        }
    }
}


namespace simple_find
{
    Mat get_roi(const Mat &src, int x, int y, int width, int height)
    {
        Mat dst;
        Rect roi(x, y, width, height);
        dst = src(roi);
        return dst;
    }
}


/**
 * Canny + findContours
 * @param PATH
 */
void simple_find_lines(const std::string& PATH)
{
    VideoCapture capture(PATH);
    if (!capture.isOpened())
    {
        std::cerr << "Error" << std::endl;
        return;
    }

    Mat frame;

    while (true)
    {
        capture >> frame;

        int y = 2 * frame.rows / 3 + 30;
        Mat frame_roi = simple_find::get_roi(frame, 0, y, frame.cols, frame.rows - y);

        Mat frame_canny;
        Canny(frame_roi, frame_canny, 100, 200);

        std::vector< std::vector<Point> > contours;
        findContours(frame_canny, contours, RETR_LIST, CHAIN_APPROX_NONE);

        // сортируем вектор векторов по их размеру
        std::sort(contours.begin(),contours.end(),
                  [](const std::vector<Point> &v1, const std::vector<Point> &v2)
                  {
                    return v1.size() > v2.size();
                  });

        Mat frame_contours(frame_canny.rows, frame_canny.cols, frame.type(), Scalar(0, 0, 0));

        for (int i = 0; i < 2; ++i)
        {
            drawContours(frame_contours, contours, i, Scalar(255, 255, 255));
        }

        //imshow("canny", frame_canny);
        imshow("contours", frame_contours);

        int k = waitKey(24);
        pause(k);

        if (k == 27)
        {
            break;
        }
    }
}


/**
 * Arithmetic mean of curvature std::vector<double>
 * @param curvature
 * @return
 */
double mean_curvature(const std::vector<double>& curvature)
{
    double res = 0;
    int count = 0;

    for (auto i : curvature)
    {
        if (i != 0 and i != std::numeric_limits<double>::infinity())
        {
            res += i;
            ++count;
        }
    }

    if (count != 0)
    {
        res /= count;
    }

    return res;
}


void test_curvature_calculations_on_video(const std::string& PATH, double resize = 1)
{
    VideoCapture capture(PATH);
    if (!capture.isOpened())
    {
        std::cerr << "Error" << std::endl;
        return;
    }

     VideoWriter outputVideo;

    Mat frame;
    while (true)
    {
        capture >> frame;

        cv::resize(frame, frame, cv::Size(), resize, resize);

        //imshow("frame", frame);

        Mat frame_roi = get_horizontal_roi(frame);

        Mat frame_birdview = build_bird_view(frame_roi);

        //imshow("bird", frame_birdview);

        Mat frame_birdview_vertical_roi = get_vertical_roi(frame_birdview);

        Mat frame_birdview_vertical_white_hsv = find_white_color(frame_birdview_vertical_roi);

        //imshow("white", frame_birdview_vertical_white_hsv);

        Mat frame_canny;
        Canny(frame_birdview_vertical_white_hsv, frame_canny, 280, 360); // 280 360

        std::vector< std::vector<Point> > contours;
        findContours(frame_canny, contours, RETR_LIST, CHAIN_APPROX_TC89_KCOS);

        const int min_contour_size = 30;
        Utils::remove_small_contours(contours, min_contour_size);

        Mat frame_contours(frame_canny.rows, frame_canny.cols, frame.type(), Scalar(0, 0, 0));
        for (int i = 0; i < contours.size(); ++i)
        {
            drawContours(frame_contours, contours, i, Scalar(255, 255, 255));
        }

        std::vector< std::vector<double> > contoursCurvature(contours.size());

        for (int i = 0; i < contoursCurvature.size(); ++i)
        {
            contoursCurvature[i] = Utils::calculate_curvature(contours[i]);
        }

        //cv::resize(frame_contours, frame_contours, cv::Size(), resize, resize);

//        for (int i = 0; i < contoursCurvature.size(); ++i)
//        {
//            double curr_curvature_mean = mean_curvature(contoursCurvature[i]);
//
//            std::string text = std::to_string(curr_curvature_mean);
//            int fontFace = FONT_HERSHEY_PLAIN;
//            double fontScale = 1;
//            int thickness = 1;
//            int baseline=0;
//            Size textSize = getTextSize(text, fontFace, fontScale, thickness, &baseline);
//            baseline += thickness;
//
//            int offset = 30;
//            Point textOrg(contours[i].begin()->x * resize - offset,contours[i].begin()->y * resize - offset);  // Position of the text
//            putText(frame_contours, text, textOrg, fontFace, fontScale,
//                    Scalar(0, 255, 0), thickness, FILLED);
//            std::cout << curr_curvature_mean << std::endl;
//        }

        //imshow("source", frame);
        //ximshow("canny", frame_canny);
        //imshow("frame_birdview_vertical_roi", frame_birdview_vertical_roi);

        //cv::resize(frame_contours, frame_contours, cv::Size(), 1.0 / resize, 1.0 / resize);
        imshow("frame_contours", frame_contours);


        if (!outputVideo.isOpened())
        {
            Size S = Size(frame_contours.cols, frame_contours.rows);
            int ex = static_cast<int>(capture.get(CAP_PROP_FOURCC));
            outputVideo.open("../result.mp4", ex, 10, S);
        }

        outputVideo.write(frame_contours);
        //save_video(capture, frame_birdview_vertical_roi);

        int k = waitKey(24);
        pause(k);
        if (k == 27)
        {
            break;
        }
    }
}


int main()
{
    const std::string PATH1 = "../videos/video.mp4";
    const std::string PATH2 = "../videos/video2.mp4";
    const std::string PATH3 = "../videos/video3.mp4";
    const std::string PATH4 = "../videos/video4_short.mp4";

    /**
     * Различие в PATH
     * 1) В фукнции get_horizontal_roi переменная y_coordinate
     * 2) В функции get_horizontal_roi переменная delta
     * 3) В функции get_bird_view переменная delta
     * 4) В функции get_vertical_roi переменная delta
     * 5) В функции get_vertical_roi переменная x_offset
     * 6) В функции build_bird_view переменная x_offset
     */

    test_curvature_calculations_on_video(PATH4, 0.5);  // 0.4;

    return 0;
}
