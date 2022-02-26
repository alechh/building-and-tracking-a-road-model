#include <iostream>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>
#include "Utils.h"
#include "ExperimentWithCurvatureCalculation.h"

using namespace cv;


/**
 *
 * @param src
 * @param distance_in_pixels -- heigth of the roi. 0 means height of the roi = src.rows
 * @return
 */
Mat get_horizontal_roi(const Mat &src, int distance_in_pixels)
{
    if (distance_in_pixels > src.rows)
    {
        std::cerr << "distance_in_pixelc must be less than src.rows" << std::endl;
        return src;
    }

    // to get the whole frame at a value of 0
    if (distance_in_pixels == 0)
    {
        distance_in_pixels = src.rows;
    }

    Rect roi(0, src.rows - distance_in_pixels, src.cols, distance_in_pixels);

    Mat dst = src(roi);

    return dst;
}


Mat get_vertical_roi(const Mat& src)
{
    Mat dst;
    int delta = 500; // 50 for PATH1, 70 (50?) for PATH2, 40 for PATH3, 45 for PATH4 (resize 0.4)
    int x_offset = 120; // 30 for PATH1, 0 (20?) for PATH2, 10 for PATH3, 20 for PATH4 (resize 0.4)
    int x_coordinate = src.cols / 2 - x_offset;
    Rect roi(x_coordinate - delta, 0, 2 * delta, src.rows);
    dst = src(roi);

    return dst;
}

Mat build_bird_view(const Mat& frame_roi)
{
    // Это точки для вида сверху, когда ужимается низ. Получается очень мелкая картинка и очень размазанная.
//    Point pt1(0, frame_roi.rows);
//    Point pt2(frame_roi.cols, frame_roi.rows);
//    Point pt3(0, 0);
//    Point pt4(frame_roi.cols, 0);
//
//    int delta = 30; // 40 for PATH1 , 50 for PATH2, 58 for PATH3, 40 for PATH4 (resize 0.4), 30 for PATH5
//
//    int x_offset = 65; // 50 for PATH1, 0 for PATH2, 7 for PATH3, 20 for PATH4 (resize 0.4), 60 for PATH5
//
//    Point pt11(frame_roi.cols / 2 - delta - x_offset, frame_roi.rows);
//    Point pt22(frame_roi.cols / 2 + delta - x_offset, frame_roi.rows);
//    Point pt33(0, 0);
//    Point pt44(frame_roi.cols, 0);



    Point pt1(772, 565);
    Point pt2(961, 565);
    Point pt3(1489, 981);
    Point pt4(21, 981);

    // 1489 - 21 = 1468  -- длина нижнего основания трапеции
    // 1468 / 2 = 734
    // 734 / 2 = 367



    Point pt11(367, 0);
    Point pt22(734 + 367, 0);

    int delta = 50; // дельта, на которую нужно сдвинуть правую нижнюю точку, чтобы линии разметки были параллельны
    Point pt33(734 + 367 - delta, frame_roi.rows);

    Point pt44(367, frame_roi.rows);



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

    //imshow("warped_image", warped_image);

    return warped_image;
}

/**
 * Pausing video by pressing "k" button
 * @param k
 */
void pause(int k)
{
    if (k == 107 || k == 204) // 107 -- k (eng), 204 -- л (rus)
    {
        while (true)
        {
            int l = waitKey(0);
            if (l == 107 || l == 204)
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

    int sensivity = 50; // 100

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


void experiment_with_curvature_calculation(Mat &birdview)
{
    // Обрезка кадра по дистанции
    const int distance_in_pixels = 500; // дистанция в пикселях для эксперимента по вычислению кривизны на разных расстояниях от машины
    Mat frame_birdview_roi_distance = get_horizontal_roi(birdview, distance_in_pixels);

    // Вычисление кривизны

    Mat frame_birdview_vertical_white_hsv = find_white_color(frame_birdview_roi_distance);

    Mat frame_canny;
    Canny(frame_birdview_vertical_white_hsv, frame_canny, 280, 360); // 280 360

    std::vector< std::vector<Point> > contours;
    findContours(frame_canny, contours, RETR_LIST, CHAIN_APPROX_NONE );

    const int min_contour_size = 30;
    Utils::remove_small_contours(contours, min_contour_size);

    Utils::sort_vector_of_vectors_of_points(contours);

    Mat frame_contours(frame_canny.rows, frame_canny.cols, birdview.type(), Scalar(0, 0, 0));
    Utils::draw_contours(contours, frame_contours, 1);


    std::vector< std::vector<double> > contoursCurvature(contours.size());
    Utils::calculate_contours_curvature(contoursCurvature, contours);

    ExperimentWithCurvatureCalculation::drawArcsOnContour(frame_birdview_roi_distance, contours[0], contoursCurvature[0]);

    imshow("roi", frame_birdview_roi_distance);
}


void test_curvature_calculations_on_video(const std::string& PATH, double resize = 1)
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

        int numberOfCurrentFrame = capture.get(1);
        int numberOfFrames = capture.get(7);
        std::cout << "frame " << numberOfCurrentFrame << "/" << numberOfFrames << std::endl;


        if (resize != 1)
        {
            cv::resize(frame, frame, cv::Size(), resize, resize);
        }

        Mat frame_birdview = build_bird_view(frame);

        cv::resize(frame_birdview, frame_birdview, cv::Size(), 0.9, 0.9);

        Mat frame_birdview_roi = get_vertical_roi(frame_birdview);

        // Эскперимент
        experiment_with_curvature_calculation(frame_birdview_roi);

        //-----------

//        Mat frame_birdview_vertical_white_hsv = find_white_color(frame_birdview_roi);
//
//        Mat frame_canny;
//        Canny(frame_birdview_vertical_white_hsv, frame_canny, 280, 360); // 280 360
//
//        std::vector< std::vector<Point> > contours;
//        findContours(frame_canny, contours, RETR_LIST, CHAIN_APPROX_NONE );
//
//        const int min_contour_size = 30;
//        Utils::remove_small_contours(contours, min_contour_size);
//
//        Utils::sort_vector_of_vectors_of_points(contours);
//
//        Mat frame_contours(frame_canny.rows, frame_canny.cols, frame.type(), Scalar(0, 0, 0));
//        Utils::draw_contours(contours, frame_contours, 1);
//
//        std::vector< std::vector<double> > contoursCurvature(contours.size());
//        Utils::calculate_contours_curvature(contoursCurvature, contours);

        int k = waitKey(25);
        pause(k);
        if (k == 27)
        {
            break;
        }
    }
}

namespace testCheckContours
{
    Mat build_bird_view(const Mat& frame)
    {
        Point pt1(837, 765);
        Point pt2(1031, 765);
        Point pt3(1469, 990);
        Point pt4(202, 981);

        // 1469 - 202 = 1267  -- длина нижнего основания трапеции
        // 1267 / 2 = 633,5
        // 633 / 2 = 316

        Point pt11(316, 0);
        Point pt22(633 + 316, 0);

        int delta = 50; // дельта, на которую нужно сдвинуть правую нижнюю точку, чтобы линии разметки были параллельны
        Point pt33(633 + 316 - delta, frame.rows);

        Point pt44(316, frame.rows);

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

        Mat warped_image(frame.rows, frame.cols, frame.type());
        warpPerspective(frame, warped_image, M, warped_image.size()); // do perspective transformation

        //imshow("warped_image", warped_image);

        return warped_image;
    }

    Mat get_vertical_roi(const Mat& src)
    {
        Mat dst;
        int delta = 500; // 50 for PATH1, 70 (50?) for PATH2, 40 for PATH3, 45 for PATH4 (resize 0.4)
        int x_offset = 120; // 30 for PATH1, 0 (20?) for PATH2, 10 for PATH3, 20 for PATH4 (resize 0.4)
        int x_coordinate = src.cols / 2 - x_offset;
        Rect roi(x_coordinate - delta, 0, 2 * delta, src.rows);
        dst = src(roi);

        return dst;
    }

    Mat find_white_color(const Mat& src)
    {
        Mat src_hsv = Mat(src.cols, src.rows, 8, 3);
        std::vector<Mat> splitedHsv = std::vector<Mat>();
        cvtColor(src, src_hsv, COLOR_RGB2HSV);
        split(src_hsv, splitedHsv);

        int sensivity = 150; // 100

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
}

void checkContoursOnVideo(const std::string &PATH, const double resize = 1)
{
    /**
     * Это функция для тестирования выделения контура, где есть повороты. Только на этом видео много теней, вот если бы
     * их как-то отфильтровать, то было бы круто
     * Вид сверху и roi я уже сделал.
     */
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

        int numberOfCurrentFrame = capture.get(1);
        int numberOfFrames = capture.get(7);
        std::cout << "frame " << numberOfCurrentFrame << "/" << numberOfFrames << std::endl;

        if (resize != 1)
        {
            cv::resize(frame, frame, cv::Size(), resize, resize);
        }

        Mat frame_birdview = testCheckContours::build_bird_view(frame);

        cv::resize(frame_birdview, frame_birdview, cv::Size(), 0.9, 0.9);

        Mat frame_birdview_roi = testCheckContours::get_vertical_roi(frame_birdview);

        Mat frame_birdview_vertical_white_hsv = testCheckContours::find_white_color(frame_birdview_roi);

        imshow("birdview", frame_birdview_roi);
        imshow("birdview_white", frame_birdview_vertical_white_hsv);

        Mat frame_canny;
        Canny(frame_birdview_vertical_white_hsv, frame_canny, 280, 360); // 280 360

        std::vector< std::vector<Point> > contours;
        findContours(frame_canny, contours, RETR_LIST, CHAIN_APPROX_NONE );

        const int min_contour_size = 30;
        Utils::remove_small_contours(contours, min_contour_size);

        Utils::sort_vector_of_vectors_of_points(contours);

        Mat frame_contours(frame_canny.rows, frame_canny.cols, frame.type(), Scalar(0, 0, 0));
        Utils::draw_contours(contours, frame_contours, 1);

        //imshow("contours", frame_contours);

        int k = waitKey(25);
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
    const std::string PATH4 = "../videos/video4.mp4";  // resize = 0.4
    const std::string PATH5 = "../videos/video5.mp4";
    const std::string PATH6 = "../videos/video6.mp4"; // resize = 0.5

    /**
     * Различие в PATH
     * 1) В фукнции get_horizontal_roi переменная y_coordinate
     * 2) В функции get_horizontal_roi переменная delta
     * 3) В функции get_bird_view переменная delta
     * 4) В функции get_vertical_roi переменная delta
     * 5) В функции get_vertical_roi переменная x_offset
     * 6) В функции build_bird_view переменная x_offset
     */

    //test_curvature_calculations_on_video(PATH6, 0.5);
    checkContoursOnVideo(PATH4, 0.5);
    return 0;
}
