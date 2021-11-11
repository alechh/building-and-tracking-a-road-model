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
 * @param y_coordinate -- upper bound of ROI
 * @return Mat dst -- Rect(0, y_coordinate, IMAGE_WIDTH, IMAGE_HEIGHT - y_coordinate)
 */
Mat getRoi(Mat src, int y_coordinate = 430)
{
    Mat dst;
    Rect roi(0, y_coordinate, src.cols, src.rows - y_coordinate);
    dst = src(roi);

    return dst;
}


template <class T>
void bird_view(T PATH)
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

        Mat frame_roi = getRoi(frame);

        Point pt1(0, frame_roi.rows);
        Point pt2(frame_roi.cols, frame_roi.rows);
        Point pt3(0, 0);
        Point pt4(frame_roi.cols, 0);

        Point pt11(569, frame_roi.rows);
        Point pt22(711, frame_roi.rows);
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

        imshow("Warped Image", warped_image);

        int stop = waitKey(24);
        if (stop == 27)
        {
            break;
        }
    }
}


template <class T>
void test(T PATH)
{
    VideoCapture capture(PATH);
    if (!capture.isOpened())
    {
        std::cerr<<"Error"<<std::endl;
        return;
    }

    Mat src;

    while (true)
    {
        capture >> src;

        getRoi(src);

        int k = waitKey(24);
        if (k == 27)
        {
            break;
        }
    }
}


int main() {
    const std::string PATH = "../videos/video.mp4";

    //test(PATH);

    bird_view(PATH);
    return 0;
}
