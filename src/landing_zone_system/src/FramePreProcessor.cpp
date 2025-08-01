#include "landing_zone_system/FramePreProcessor.hpp"

cv::Mat FramePreProcessor::run(const cv::Mat& input) {
    cv::Mat gray = toGray(input);
    return applyCLAHE(gray);
}

cv::Mat FramePreProcessor::toGray(const cv::Mat& input) {
    if (input.empty()) {
        // Recebeu frame vazio, retorna Mat vazio também
        return cv::Mat();
    }
    cv::Mat gray;
    if (input.channels() == 3) {
        cv::cvtColor(input, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = input.clone();
    }
    return gray;
}

cv::Mat FramePreProcessor::applyCLAHE(const cv::Mat& gray) {
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(4.0);
    cv::Mat claheOutput;
    clahe->apply(gray, claheOutput);
    return claheOutput;
}
