#include <iostream>
#include <opencv2/opencv.hpp>

int main(int argc, char **argv) {
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Error opening camera." << std::endl;
        return -1;
    }

    cv::Mat frame;
    bool success;
    cv::namedWindow("Camera", cv::WINDOW_NORMAL);

    while (true) {
        success = cap.read(frame);
        if (success) {
            cv::imshow("Camera", frame);
        }
        if (cv::waitKey(30) >= 0) {
            break;
        }
    }
    cap.release();
    cv::destroyAllWindows();

    std::cout << "Camera is closed." << std::endl;
    return 0;
}