#ifndef DETECTORS_H
#define DETECTORS_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <chrono>

using namespace cv;
using namespace std;

class Detector {  
    public:
        static constexpr int TARGET_KP = 800;

        virtual std::pair<std::vector<cv::KeyPoint>, double> run(const Mat &image) = 0;
        static std::vector<cv::KeyPoint> topN(const std::vector<cv::KeyPoint>& kps);
        virtual ~Detector() = default;
};

class ShiTomassi : public Detector {
    public:
        std::pair<std::vector<cv::KeyPoint>, double> run(const Mat &image) override;
        ~ShiTomassi(){};
};

#endif // DETECTORS_H
