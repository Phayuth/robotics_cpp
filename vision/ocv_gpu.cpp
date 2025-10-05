#include <opencv2/opencv.hpp>
#include <stdio.h>

double getPSNR(const cv::Mat &I1, const cv::Mat &I2) {
    cv::Mat s1;
    absdiff(I1, I2, s1);      // |I1 - I2|
    s1.convertTo(s1, CV_32F); // cannot make a square on 8 bits
    s1 = s1.mul(s1);          // |I1 - I2|^2

    cv::Scalar s = sum(s1); // sum elements per channel

    double sse = s.val[0] + s.val[1] + s.val[2]; // sum channels

    if (sse <= 1e-10) // for small values return zero
        return 0;
    else {
        double mse = sse / (double)(I1.channels() * I1.total());
        double psnr = 10.0 * log10((255 * 255) / mse);
        return psnr;
    }
}

// double getPSNR_CUDA(const cv::Mat &I1, const cv::Mat &I2) {
//     cv::cuda::GpuMat gI1, gI2, gs, t1, t2;

//     gI1.upload(I1);
//     gI2.upload(I2);

//     gI1.convertTo(t1, CV_32F);
//     gI2.convertTo(t2, CV_32F);

//     cv::cuda::absdiff(t1.reshape(1), t2.reshape(1), gs);
//     cv::cuda::multiply(gs, gs, gs);

//     cv::Scalar s = cv::cuda::sum(gs);
//     double sse = s.val[0] + s.val[1] + s.val[2];

//     if (sse <= 1e-10) // for small values return zero
//         return 0;
//     else {
//         double mse = sse / (double)(gI1.channels() * I1.total());
//         double psnr = 10.0 * log10((255 * 255) / mse);
//         return psnr;
//     }
// }

int main(int argc, char **argv) {
    std::cout << "OpenCV version: " << CV_VERSION << std::endl;
    std::cout << cv::cuda::getCudaEnabledDeviceCount() << std::endl; //

    cv::Mat I1;
    cv::Mat I2;

    // getPSNR_CUDA(I1, I2);

    std::cout << "Program ended." << std::endl;
    return 0;
}