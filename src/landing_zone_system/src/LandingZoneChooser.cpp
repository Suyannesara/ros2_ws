#include "landing_zone_system/LandingZoneChooser.hpp"

constexpr double CONF_THRESHOLD = 0.8;
constexpr double DENSITY_THRESHOLD = 1;

LandingZoneChooser::LandingZoneChooser(int rows, int cols)
    : gridRows(rows), gridCols(cols) {}


// Retorna matriz P com coeficientes de certeza por quadrante normalizados
cv::Mat LandingZoneChooser::computeCertainty(const cv::Mat& counts) {
    int R = counts.rows, C = counts.cols;
    cv::Mat P = cv::Mat::zeros(R, C, CV_64F);
    for (int r = 0; r < R; ++r) {
        double S = (R > 1) ? double(r) / (R - 1) : 0;
        for (int c = 0; c < C; ++c)
            if (counts.at<int>(r, c) <= DENSITY_THRESHOLD)
                P.at<double>(r, c) = S;
    }
    return P;
}

// Calcula a variância de intensidade por quadrante
cv::Mat LandingZoneChooser::computeIntensityVariance(const cv::Mat& grayImage, int gridRows, int gridCols) {
    int cellH = grayImage.rows / gridRows;
    int cellW = grayImage.cols / gridCols;
    cv::Mat varMat = cv::Mat::zeros(gridRows, gridCols, CV_64F);

    for (int r = 0; r < gridRows; ++r) {
        for (int c = 0; c < gridCols; ++c) {
            int x = c * cellW;
            int y = r * cellH;
            cv::Rect roi(x, y, cellW, cellH);
            cv::Mat cell = grayImage(roi);

            cv::Scalar mean, stddev;
            cv::meanStdDev(cell, mean, stddev);
            varMat.at<double>(r, c) = stddev[0] * stddev[0];
        }
    }

    return varMat;
}

// Compara dois candidatos com base no coeficiente e variância
bool LandingZoneChooser::isBetterCandidate(const LandingZoneCandidate& a, const LandingZoneCandidate& b) {
    if (a.certainty > b.certainty) return true;
    if (a.certainty == b.certainty && a.intensityVar < b.intensityVar) return true;
    return false;
}

// Função principal que seleciona a melhor zona
LandingZoneCandidate LandingZoneChooser::getBestZone(const std::vector<cv::KeyPoint>& keypoints, const cv::Mat& grayImage) {
    cv::Mat counts = makeCountMatrix(keypoints, grayImage.size());
    int R = counts.rows, C = counts.cols;
    double blockW = grayImage.cols / C;
    double blockH = grayImage.rows / R;

    cv::Mat certaintyMap = computeCertainty(counts);
    cv::Mat intensityVar = computeIntensityVariance(grayImage, R, C);

    LandingZoneCandidate bestCandidate;
    bestCandidate.certainty = -1;
    bestCandidate.intensityVar = std::numeric_limits<double>::max();

    for (int r = 0; r < R; ++r) {
        for (int c = 0; c < C; ++c) {
            double cert = certaintyMap.at<double>(r, c);
            if (cert < CONF_THRESHOLD) continue;

            double var = intensityVar.at<double>(r, c);
            if (cert > bestCandidate.certainty || (cert == bestCandidate.certainty && var < bestCandidate.intensityVar)) {
                bestCandidate.zone = cv::Rect(c * blockW, r * blockH, blockW, blockH);
                bestCandidate.certainty = cert;
                bestCandidate.intensityVar = var;
            }
        }
    }

    return bestCandidate;
}



LandingZoneCandidate LandingZoneChooser::evaluateMultipleFrames(
    const std::vector<std::vector<cv::KeyPoint>>& keypointsVec,
    const std::vector<cv::Mat>& grayFrames
) {
    LandingZoneCandidate bestOverall;
    bestOverall.certainty = -1;
    bestOverall.intensityVar = std::numeric_limits<double>::max();

    for (size_t i = 0; i < keypointsVec.size(); ++i) {
        LandingZoneCandidate candidate = getBestZone(keypointsVec[i], grayFrames[i]);
        candidate.frameIndex = static_cast<int>(i);

        if (isBetterCandidate(candidate, bestOverall)) {
            bestOverall = candidate;
        }
    }

    return bestOverall;
}


cv::Mat LandingZoneChooser::makeCountMatrix(const std::vector<cv::KeyPoint>& keypoints, const cv::Size& imageSize) {
    cv::Mat countMatrix = cv::Mat::zeros(gridRows, gridCols, CV_32S);

    int cellWidth = imageSize.width / gridCols;
    int cellHeight = imageSize.height / gridRows;

    for (const auto& kp : keypoints) {
        int col = std::min(static_cast<int>(kp.pt.x / cellWidth), gridCols - 1);
        int row = std::min(static_cast<int>(kp.pt.y / cellHeight), gridRows - 1);
        countMatrix.at<int>(row, col)++;
    }

    return countMatrix;
}

// cv::Rect LandingZoneChooser::run(const std::vector<cv::KeyPoint>& keypoints, const cv::Mat& grayImage) {
//     // 1. Gera matriz de contagem automaticamente
//     cv::Mat countMatrix = makeCountMatrix(keypoints, grayImage.size());

//     // 2. Usa countMatrix e grayImage para encontrar a melhor zona
//     return getBestZone(countMatrix, grayImage);
// }
