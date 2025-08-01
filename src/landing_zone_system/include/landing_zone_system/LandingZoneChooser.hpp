#pragma once
#include <opencv2/opencv.hpp>
#include <limits>

struct LandingZoneCandidate {
    cv::Rect zone;
    double certainty;
    double intensityVar;
    int frameIndex;
};

class LandingZoneChooser {
public:
    LandingZoneChooser(int rows = 4, int cols = 4);

    LandingZoneCandidate getBestZone(const std::vector<cv::KeyPoint>& keypoints, const cv::Mat& grayImage);
    LandingZoneCandidate evaluateMultipleFrames(
        const std::vector<std::vector<cv::KeyPoint>>& keypointsVec,
        const std::vector<cv::Mat>& grayFrames);

private:
    int gridRows;
    int gridCols;

    cv::Mat makeCountMatrix(const std::vector<cv::KeyPoint>& keypoints, const cv::Size& imageSize);
    cv::Mat computeCertainty(const cv::Mat& counts);
    cv::Mat computeIntensityVariance(const cv::Mat& grayImage, int gridRows, int gridCols);
    bool isBetterCandidate(const LandingZoneCandidate& a, const LandingZoneCandidate& b);
};
