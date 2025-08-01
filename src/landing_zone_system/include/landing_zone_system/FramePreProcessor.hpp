#ifndef FRAME_PREPROCESSOR_HPP
#define FRAME_PREPROCESSOR_HPP

#include <opencv2/opencv.hpp>

class FramePreProcessor {
public:
    cv::Mat run(const cv::Mat& input);

private:
    cv::Mat toGray(const cv::Mat& input);
    cv::Mat applyCLAHE(const cv::Mat& gray);
};

#endif // FRAME_PREPROCESSOR_HPP
