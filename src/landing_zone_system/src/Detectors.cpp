#include "landing_zone_system/Detectors.hpp"

std::vector<cv::KeyPoint> Detector::topN(const std::vector<cv::KeyPoint>& kps) {
    if (static_cast<int>(kps.size()) <= TARGET_KP)
        return kps;

    std::vector<cv::KeyPoint> result = kps;
    // Ordena por response de forma estável (preserva a ordem original em empates)
    std::stable_sort(result.begin(), result.end(), [](const cv::KeyPoint& a, const cv::KeyPoint& b) {
        return a.response > b.response;
    });

    result.resize(TARGET_KP);
    return result;
}



std::pair<std::vector<cv::KeyPoint>, double> ShiTomassi::run(const cv::Mat& img)
{
    // detecta cantos como você já tinha
    double qualityLevel = 0.2;
    double minDistance  = 20;
    int    maxCorners   = 20;
    std::vector<cv::Point2f> corners;
    auto t0 = std::chrono::high_resolution_clock::now();
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance);
    auto t1 = std::chrono::high_resolution_clock::now();
    double elapsed = std::chrono::duration<double, std::milli>(t1 - t0).count();

    // calcula o mapa de qualidade (cornerMinEigenVal)
    cv::Mat eig;
    cv::cornerMinEigenVal(img, eig, /*blockSize=*/15, /*ksize=*/5);

    // converte cada canto em KeyPoint, usando eig como response
    std::vector<cv::KeyPoint> kps;
    kps.reserve(corners.size());
    for (auto &pt : corners) {
        float quality = eig.at<float>(int(pt.y), int(pt.x));
        // x, y, size=3, angle=-1, response=quality
        kps.emplace_back(pt.x, pt.y, 3.0f, -1.0f, quality);
    }

    // Top‑N pelo response (igual ao Python)
    auto best = topN(kps);
    return { best, elapsed };
}


