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

std::vector<double> getCurvature(std::vector<cv::Point> const& vecContourPoints, int step)
{
    std::vector< double > vecCurvature( vecContourPoints.size() );

    if (vecContourPoints.size() < step)
        return vecCurvature;

    auto frontToBack = vecContourPoints.front() - vecContourPoints.back();
    bool isClosed = ((int)std::max(std::abs(frontToBack.x), std::abs(frontToBack.y))) <= 1;

    cv::Point2f pplus, pminus;
    cv::Point2f f1stDerivative, f2ndDerivative;
    for (int i = 0; i < vecContourPoints.size(); i++ )
    {
        const cv::Point2f& pos = vecContourPoints[i];

        int maxStep = step;
        if (!isClosed)
        {
            maxStep = std::min(std::min(step, i), (int)vecContourPoints.size()-1-i);
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


        f1stDerivative.x = (pplus.x -           pminus.x) / (iplus-iminus);
        f1stDerivative.y = (pplus.y -           pminus.y) / (iplus-iminus);
        f2ndDerivative.x = (pplus.x - 2*pos.x + pminus.x) / ((iplus-iminus)/2*(iplus-iminus)/2);
        f2ndDerivative.y = (pplus.y - 2*pos.y + pminus.y) / ((iplus-iminus)/2*(iplus-iminus)/2);

        double curvature2D;
        double divisor = f1stDerivative.x*f1stDerivative.x + f1stDerivative.y*f1stDerivative.y;
        if ( std::abs(divisor) > 10e-8 )
        {
            curvature2D =  std::abs(f2ndDerivative.y*f1stDerivative.x - f2ndDerivative.x*f1stDerivative.y) /
                           pow(divisor, 3.0/2.0 )  ;
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


void calculation_of_curvature(const std::string& PATH)
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
            std::vector<double> cucc_curvature = getCurvature(contours[i], 1);
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
    calculation_of_curvature(PATH2);
    return 0;
}
