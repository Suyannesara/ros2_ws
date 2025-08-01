#ifndef VIDEO_ANNOTATOR_HPP
#define VIDEO_ANNOTATOR_HPP

#include <opencv2/opencv.hpp>
#include <string>

class VideoAnnotator {
public:
    VideoAnnotator(int width, int height, double fps, const std::string& outputPath);
    ~VideoAnnotator();

    void drawCurrentZone(cv::Mat& frame, const cv::Rect& zone, const cv::Scalar& color = cv::Scalar(0, 255, 0));
    void writeFrame(const cv::Mat& frame);
    void close();

private:
    cv::VideoWriter writer;
    bool isOpen = false;
};

#endif // VIDEO_ANNOTATOR_HPP
