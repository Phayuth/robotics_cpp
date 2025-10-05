#include <opencv2/opencv.hpp>
#include <stdio.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

int main(int argc, char **argv) {
    if (argc != 2) {
        printf("usage: display_image.out <Image_Path>\n");
        return -1;
    }

    cv::Mat image;
    image = cv::imread(argv[1], cv::IMREAD_COLOR);

    cv::Point2d pc(image.cols / 2., image.rows / 2.);
    cv::circle(image, pc, 63, cv::Scalar(0, 255, 0), 3, cv::LineTypes::LINE_AA);

    pc.x = 100;
    pc.y = 100;
    cv::circle(image, pc, 63, cv::Scalar(0, 255, 0), 3, cv::LineTypes::LINE_AA);

    if (!image.data) {
        printf("No image data \n");
        return -1;
    }
    cv::namedWindow("Display Image", cv::WINDOW_NORMAL);
    cv::imshow("Display Image", image);
    cv::waitKey(0);

    return 0;
}