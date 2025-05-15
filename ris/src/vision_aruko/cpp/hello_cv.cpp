#include <opencv2/opencv.hpp>

int main() {
    cv::Mat img = cv::imread("data/images/Lenna.png");

    cv::imshow("Webcam", img);
    cv::waitKey();
}
