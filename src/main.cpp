#include <iostream>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>
#include <algorithm>

using namespace cv;


/**
 * Get ROI of the Image
 * @param src -- Mat image
 * @return Mat dst -- Rect(0, y_coordinate, IMAGE_WIDTH, IMAGE_HEIGHT - y_coordinate)
 */
Mat get_horizontal_roi(const Mat& src)
{
    Mat dst;
    int delta = 0; // for PATH1 100, for PATH2 0 -- чтобы обрезать капот на первом видео
    int y_coordinate = 500; // for PATH 430, for PATH2 500
    Rect roi(0, y_coordinate, src.cols, src.rows - y_coordinate - delta);
    dst = src(roi);

    return dst;
}

/**
 * Get ROI of the Image
 * @param src -- Mat image
 * @return Mat dst -- Rect(x_coordinate - delta, 0, 2 * delta, src.rows)
 */
Mat get_vertical_roi(const Mat& src)
{
    Mat dst;
    int delta = 50; // 50 for PATH1, 70 (50?) for PATH2
    int x_offset = 20; // 30 for PATH1, 0 (20?) for PATH2
    int x_coordinate = src.cols / 2 - x_offset;
    Rect roi(x_coordinate - delta, 0, 2 * delta, src.rows);
    dst = src(roi);

    return dst;
}


Point get_vanishing_point()
{
    return {595, 422};  // for PATH1
}

/**
 * Getting a bird view from a video frame
 * @param frame -- input frame
 * @return
 */
Mat build_bird_view(const Mat& frame)
{
    Mat frame_roi = get_horizontal_roi(frame);

    Point pt1(0, frame_roi.rows);
    Point pt2(frame_roi.cols, frame_roi.rows);
    Point pt3(0, 0);
    Point pt4(frame_roi.cols, 0);

    int delta = 50; // 40 for PATH1 , 50 for PATH2

    int x_offset = 0; // 50 for PATH1, 0 for PATH2

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

/**
 * Search for white color on frame
 * @param src -- input Image
 * @return
 */
Mat find_white_color(const Mat& src)
{
    Mat src_hsv = Mat(src.cols, src.rows, 8, 3);
    std::vector<Mat> splitedHsv = std::vector<Mat>();
    cvtColor(src, src_hsv, COLOR_RGB2HSV);
    split(src_hsv, splitedHsv);

    int sensivity = 90; // 120

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

/**
 * Seacrh for white lines via bird view and Canny filter
 * @param PATH -- path to the video file
 */
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
    /**
     * Getting the frame's region of interest
     * @param src -- input image
     * @param x -- x coordinate of the upper left corner
     * @param y -- y coordinate of the upper left corner
     * @param width -- width of the roi
     * @param height -- height of the roi
     * @return
     */
    Mat get_roi(const Mat &src, int x, int y, int width, int height)
    {
        Mat dst;
        Rect roi(x, y, width, height);
        dst = src(roi);
        return dst;
    }
}

/**
 *  Simple line search via Canny filter and findContours()
 * @param PATH -- path to the video file
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


int main()
{
    const std::string PATH1 = "../videos/video.mp4";
    const std::string PATH2 = "../videos/video2.mp4";


    //find_lines_birdview(PATH2);
    simple_find_lines(PATH2);
    return 0;
}
