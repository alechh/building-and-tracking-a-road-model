#include <iostream>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;


/**
 * Get ROI of the Image
 * @param src -- Mat image
 * @return Mat dst -- Rect(0, y_coordinate, IMAGE_WIDTH, IMAGE_HEIGHT - y_coordinate)
 */
Mat get_roi(const Mat& src)
{
    Mat dst;
    int delta = 0; // for PATH1 100, for PATH2 0 -- чтобы обрезать капот на первом видео
    int y_coordinate = 500; // for PATH 430, for PATH2 500
    Rect roi(0, y_coordinate, src.cols, src.rows - y_coordinate - delta);
    dst = src(roi);

    return dst;
}


Point get_vanishing_point()
{
    return Point(595, 422);  // for PATH1
}


Mat get_bird_view(const Mat& frame)
{
    Mat frame_roi = get_roi(frame);

    Point pt1(0, frame_roi.rows);
    Point pt2(frame_roi.cols, frame_roi.rows);
    Point pt3(0, 0);
    Point pt4(frame_roi.cols, 0);

    int delta = 50; // 71 for PATH1, 50 for PATH2

    Point pt11(frame_roi.cols / 2 - delta, frame_roi.rows);
    Point pt22(frame_roi.cols / 2 + delta, frame_roi.rows);
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


void find_lines(const std::string& PATH)
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

        Mat frame_birdview = get_bird_view(frame);

        Mat frame_canny;
        Canny(frame_birdview, frame_canny, 130, 200);

        imshow("source", frame);
        imshow("birdview", frame_birdview);
        //imshow("canny", frame_canny);


        int k = waitKey(24);
        if (k == 27)
        {
            break;
        }
    }
}


void play_video(const std::string& PATH)
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

        imshow("frame", frame);

        int k = waitKey(24);
        if (k == 107)
        {
            while (true)
            {
                int l = waitKey(0);
                if (l == 107)
                {
                    break;
                }
            }

        }
        if (k == 27)
        {
            break;
        }
    }
}

std::vector<double> calculate_curvature(std::vector<cv::Point> const& vecContourPoints, int step)
{
    std::vector<double> vecCurvature(vecContourPoints.size());

    if (vecContourPoints.size() < step)
    {
        return vecCurvature;
    }

    auto frontToBack = vecContourPoints.front() - vecContourPoints.back();
    bool isClosed = ((int)std::max(std::abs(frontToBack.x), std::abs(frontToBack.y))) <= 1;

    cv::Point2f pplus, pminus;
    cv::Point2f f1stDerivative, f2ndDerivative;
    for (int i = 0; i < vecContourPoints.size(); i++)
    {
        const cv::Point2f& pos = vecContourPoints[i];

        int maxStep = step;
        if (!isClosed)
        {
            maxStep = std::min(std::min(step, i), (int)vecContourPoints.size() - 1 - i);
            if (maxStep == 0)
            {
                vecCurvature[i] = std::numeric_limits<double>::infinity();
                continue;
            }
        }

        int iminus = i - maxStep;
        int iplus = i + maxStep;
        pminus = vecContourPoints[iminus < 0 ? iminus + vecContourPoints.size() : iminus];
        pplus = vecContourPoints[iplus > vecContourPoints.size() ? iplus - vecContourPoints.size() : iplus];

        f1stDerivative.x = (pplus.x - pminus.x) / float(iplus - iminus);
        f1stDerivative.y = (pplus.y - pminus.y) / float(iplus - iminus);
        f2ndDerivative.x = (pplus.x - 2 * pos.x + pminus.x) / float(float(iplus - iminus) / 2 * float(iplus - iminus) / 2);
        f2ndDerivative.y = (pplus.y - 2 * pos.y + pminus.y) / float(float(iplus - iminus) / 2 * float(iplus - iminus) / 2);

        double curvature2D;
        double divisor = f1stDerivative.x * f1stDerivative.x + f1stDerivative.y * f1stDerivative.y;
        if (std::abs(divisor) > 10e-15)  // 10e-8
        {
            curvature2D =  std::abs(f2ndDerivative.y * f1stDerivative.x - f2ndDerivative.x * f1stDerivative.y) /
                           pow(divisor, 3.0 / 2.0 )  ;
        }
        else
        {
            curvature2D = std::numeric_limits<double>::infinity();
        }

        vecCurvature[i] = curvature2D;
    }
    return vecCurvature;
}


void draw_contours_using_curvature(Mat &dst, const std::vector<Point>& contour, const std::vector<double>& curvature)
{
    for (int i = 0; i < contour.size(); i++)
    {
        circle(dst, contour[i], 2, Scalar(0, 0, int(curvature[i]) % 255));
    }
}


void test_calculation_of_curvature(const std::string& PATH)
{
    VideoCapture capture(PATH);
    if (!capture.isOpened())
    {
        std::cerr << "Error" << std::endl;
        return;
    }

    Mat frame, frame_canny;
    while (true)
    {
        capture >> frame;

        Canny(frame, frame_canny, 130, 200);

        std::vector< std::vector<Point> > contours;
        findContours(frame_canny, contours, RETR_LIST, CHAIN_APPROX_NONE);

        Mat frame_contours(frame.rows, frame.cols, frame.type(), Scalar(255, 255, 255));

        for (int i = 0; i < contours.size(); i++)
        {
            std::vector<double> cucc_curvature = calculate_curvature(contours[i], 1);
            draw_contours_using_curvature(frame_contours, contours[i], cucc_curvature);
        }

        imshow("frame_contours", frame_contours);

        int k = waitKey(24);
        if (k == 27)
        {
            break;
        }
    }
}


std::vector<Point> get_circle_contour(const int& R)
{
    std::vector<Point> contour;
    double t = 0;
    double epsilon = 10e-4;
    while (t < 2 * CV_PI)
    {
        contour.emplace_back(500 + R * cos(t),300 + R * sin(t));
        t += epsilon;
        //std::cout << "t = " << t << std::endl;
    }
    return contour;
}


void test_circle_curvature()
{
    // Curvature of circle = 1/R, where R -- radius of the circle

    const int radius = 10000000;

    Mat image_contours(600, 1000, 16);

    std::vector<double> circle_curvature;

    std::vector< std::vector<Point> > contours;
    std::vector<Point> circle_contour = get_circle_contour(radius);
    contours.emplace_back(circle_contour);
    std::cout << "Circle contour size = " << circle_contour.size() << std::endl;

//        for (const auto& i : circle_contour)
//        {
//            std::cout << i << std::endl;
//        }

    drawContours(image_contours, contours, 0, Scalar(255, 255, 255));
    circle_curvature = calculate_curvature(circle_contour, 1);

    double max_error = 0, min_error = 1;
    int count_inf = 0, count_zero = 0;

    for (auto i : circle_curvature)
    {
        if (i == circle_curvature[0])
        {
            continue;
        }

        //std::cout << "i = " << i << std::endl;

        if (i == std::numeric_limits<double>::infinity())
        {
            count_inf++;
            continue;
        }
        if (i == 0)
        {
            count_zero++;
            continue;
        }

        double error = std::abs(1.0 / radius - i);

        if (error > max_error)
        {
            max_error = error;
        }
        if (error < min_error)
        {
            min_error = error;
        }
        //std::cout << "Error = " << error << std::endl;
    }
    std::cout <<  "Max Error = " << max_error << std::endl;
    std::cout << "Min Error = " << min_error << std::endl;
    std::cout << std::endl << "Number of inf = " << count_inf << std::endl;
    std::cout << "Number of zero = " << count_zero << std::endl;
    std::cout << "Number of non inf/zero = " << circle_contour.size() - count_zero - count_inf << std::endl;
}


int main()
{
    const std::string PATH1 = "../videos/video.mp4";
    const std::string PATH2 = "../videos/video2.mp4";

    /**
     * Различие в PATH
     * 1) В фукнции get_roi переменная y_coordinate
     * 2) В функции get_roi переменная delta
     * 3) В функции get_bird_view переменная delta
     */

    //find_lines(PATH2);
    //play_video(PATH1);
    //calculation_of_curvature(PATH2);
    test_circle_curvature();

    return 0;
}
