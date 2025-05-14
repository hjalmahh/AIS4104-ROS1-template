#include <opencv2/opencv.hpp>

#include <iostream>

int main() {
    setLogLevel(cv::utils::logging::LOG_LEVEL_SILENT);

    cv::VideoCapture cap(0);

    if (!cap.isOpened()) {
        std::cerr << "Error: Unable to open camera" << std::endl;
        return 1;
    }

    namedWindow("Webcam", cv::WINDOW_AUTOSIZE);

    cv::Mat frame;
    while (true) {
        cap.read(frame);
        imshow("Webcam", frame);
        const auto key = cv::waitKey(1);
        if (key == 27 || key == 'q') {
            break;
        }
    }
}
