#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>


int main() {

    setLogLevel(cv::utils::logging::LOG_LEVEL_SILENT);

    bool flipImage = true;

    // Open the default camera
    cv::VideoCapture cap(0);

    if (!cap.isOpened()) {
        std::cerr << "Could not open camera" << std::endl;
        return 1;
    }

    // Define the dictionary of ArUco markers
    cv::aruco::Dictionary arucoDict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    // Define the parameters for marker detection
    cv::aruco::DetectorParameters detectorParams;

    // Create an Aruco detector
    cv::aruco::ArucoDetector detector(arucoDict, detectorParams);

    while (true) {
        cv::Mat image;
        cap >> image; // Capture a frame from the camera

        if (image.empty()) {
            std::cerr << "Failed to capture an image" << std::endl;
            break;
        }

        // Convert the image to grayscale
        cv::Mat gray;
        cvtColor(image, gray, cv::COLOR_BGR2GRAY);

        // Detect the markers
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f> > corners, rejected;
        detector.detectMarkers(gray, corners, ids, rejected);

        if (flipImage) {
            flip(image, image, 1);
        }

        // Draw the detected markers on the image
        if (!ids.empty()) {
            if (flipImage) {
                // Flip the corners to match the flipped image
                for (auto &corner: corners) {
                    for (auto &point: corner) {
                        point.x = static_cast<float>(image.cols) - point.x; // Flip the x-coordinate
                    }
                }
            }
            cv::aruco::drawDetectedMarkers(image, corners, ids);
        }

        // Display the result
        imshow("Webcam", image);

        // Exit on 'q' or ESC key
        const auto key = cv::waitKey(1);
        if (key == 27 || key == 'q') {
            break;
        } else if (key == 'f') {
            flipImage = !flipImage;
        }
    }

    return 0;
}
