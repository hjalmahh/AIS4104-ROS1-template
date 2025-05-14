#include <opencv2/opencv.hpp>
#include <iostream>


struct ApplicationData {
    const char *window_name = "Combined Images: Original | Grayscale | Blurred | Edges";
    cv::Mat image, grayImage, blurredImage, edges;
};

// Callback function for trackbar (does nothing but required)
void on_trackbar(int, void *data) {
    auto *appData = static_cast<ApplicationData *>(data);

    int blurKsize = cv::getTrackbarPos("Blur Kernel", appData->window_name);
    int cannyMin = cv::getTrackbarPos("Canny Min", appData->window_name);
    int cannyMax = cv::getTrackbarPos("Canny Max", appData->window_name);

    // Ensure the blur kernel size is odd and at least 1
    if (blurKsize % 2 == 0) blurKsize += 1;
    blurKsize = std::max(1, blurKsize);

    // Apply Gaussian blur
    cv::GaussianBlur(appData->grayImage, appData->blurredImage, cv::Size(blurKsize, blurKsize), 0);

    // Perform edge detection using Canny
    cv::Canny(appData->blurredImage, appData->edges, cannyMin, cannyMax);

    // Convert grayscale images to 3-channel for display
    cv::Mat grayDisplay, blurredDisplay, edgesDisplay;
    cv::cvtColor(appData->grayImage, grayDisplay, cv::COLOR_GRAY2BGR);
    cv::cvtColor(appData->blurredImage, blurredDisplay, cv::COLOR_GRAY2BGR);
    cv::cvtColor(appData->edges, edgesDisplay, cv::COLOR_GRAY2BGR);

    // Concatenate images horizontally
    cv::Mat combinedImage;
    cv::hconcat(std::vector{appData->image, grayDisplay, blurredDisplay, edgesDisplay}, combinedImage);

    // Display the combined image
    cv::imshow(appData->window_name, combinedImage);
}

int main() {
    setLogLevel(cv::utils::logging::LOG_LEVEL_SILENT);

    ApplicationData data;
    const std::string imagePath{"data/images/Lenna.png"};
    data.image = cv::imread(imagePath);

    // Check if the image was loaded successfully
    if (data.image.empty()) {
        std::cerr << "Error: Could not load image." << std::endl;
        return 1;
    }

    // Convert the image to grayscale
    cv::cvtColor(data.image, data.grayImage, cv::COLOR_BGR2GRAY);

    // Create a window
    cv::namedWindow(data.window_name, cv::WINDOW_AUTOSIZE);

    // Create trackbars for Gaussian blur and Canny edge detection
    cv::createTrackbar("Blur Kernel", data.window_name, nullptr, 20, on_trackbar, &data);
    cv::createTrackbar("Canny Min", data.window_name, nullptr, 255, on_trackbar, &data);
    cv::createTrackbar("Canny Max", data.window_name, nullptr, 255, on_trackbar, &data);

    cv::setTrackbarPos("Blur Kernel", data.window_name, 2);
    cv::setTrackbarPos("Canny Min", data.window_name, 50);
    cv::setTrackbarPos("Canny Max", data.window_name, 150);

    // Call the callback function once to initialize
    on_trackbar(0, &data);

    // Wait for user interaction
    while (true) {
        if (getWindowProperty(data.window_name, cv::WND_PROP_VISIBLE) < 1) {
            break; // exit when user presses X on window
        }

        const auto key = static_cast<char>(cv::waitKey(10));
        if (key == 'q' || key == 27) break; // Exit on 'q' or ESC key press
    }
}
