#include "landing_zone_system/LandingZoneTracker.hpp"

CSRTLandingZoneTracker::CSRTLandingZoneTracker() {
    tracker = cv::TrackerCSRT::create();
}

void CSRTLandingZoneTracker::initialize(const cv::Mat& frame, const LandingZoneCandidate& zone) {
    tracker->init(frame, zone.zone);
}

bool CSRTLandingZoneTracker::track(const cv::Mat& frame, LandingZoneCandidate& zoneAtualizada) {
    cv::Rect trackedBox;
    bool success = tracker->update(frame, trackedBox);

    if (success) {
        zoneAtualizada.zone = trackedBox;
    }

    return success;
}


void TemplateMatchingLandingZoneTracker::initialize(const cv::Mat& frame, const LandingZoneCandidate& zone) {
    std::cout << "INICIEI O TRACKER" << std::endl;
    lastMatch = zone.zone;
    templateSize = zone.zone.size();

    // Converte para cinza
    cv::Mat grayFrame;
    if (frame.channels() == 3)
        cv::cvtColor(frame, grayFrame, cv::COLOR_BGR2GRAY);
    else
        grayFrame = frame.clone();

    // Verifica se a zona está completamente dentro da imagem
    if (zone.zone.x < 0 || zone.zone.y < 0 ||
        zone.zone.x + zone.zone.width > grayFrame.cols ||
        zone.zone.y + zone.zone.height > grayFrame.rows) {
        std::cerr << "Erro: zona fora dos limites da imagem no initialize()." << std::endl;
        return;
    }

    templateImage = grayFrame(zone.zone).clone();
}


bool TemplateMatchingLandingZoneTracker::track(const cv::Mat& frame, LandingZoneCandidate& zoneAtualizada) {
    if (frame.empty() || templateImage.empty()) return false;

    // Converte para cinza
    cv::Mat grayFrame;
    if (frame.channels() == 3)
        cv::cvtColor(frame, grayFrame, cv::COLOR_BGR2GRAY);
    else
        grayFrame = frame.clone();

    // Define área de busca ao redor da última localização
    const int searchMargin = 30;
    int x = std::max(0, static_cast<int>(lastMatch.x) - searchMargin);
    int y = std::max(0, static_cast<int>(lastMatch.y) - searchMargin);
    int w = std::min(grayFrame.cols - x, templateSize.width + 2 * searchMargin);
    int h = std::min(grayFrame.rows - y, templateSize.height + 2 * searchMargin);

    // Garante que esteja dentro dos limites
    if (w < templateSize.width || h < templateSize.height)
        return false;

    cv::Rect searchWindow(x, y, w, h);
    cv::Mat searchROI = grayFrame(searchWindow);

    // Faz o match
    cv::Mat result;
    cv::matchTemplate(searchROI, templateImage, result, cv::TM_CCOEFF_NORMED);

    double maxVal;
    cv::Point maxLoc;
    cv::minMaxLoc(result, nullptr, &maxVal, nullptr, &maxLoc);

    // Ajusta para coordenadas globais
    cv::Point matchTopLeft = maxLoc + searchWindow.tl();
    cv::Rect newMatch(matchTopLeft.x, matchTopLeft.y, templateSize.width, templateSize.height);

    // Verifica se newMatch está dentro dos limites da imagem
    if (newMatch.x < 0 || newMatch.y < 0 ||
        newMatch.x + newMatch.width > grayFrame.cols ||
        newMatch.y + newMatch.height > grayFrame.rows)
        return false;

    lastMatch = newMatch;

    zoneAtualizada.zone = newMatch;
    zoneAtualizada.certainty = maxVal;
    zoneAtualizada.intensityVar = 0.0;
    zoneAtualizada.frameIndex = -1;

    return maxVal > 0.5;  // Threshold suavizado
}


void LucasKanadeLandingZoneTracker::initialize(const cv::Mat& frame, const LandingZoneCandidate& zone) {
    cv::Mat gray;
    if (frame.channels() == 3)
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    else
        gray = frame.clone();

    lastZone = zone.zone;
    prevGray = gray.clone();

    // Recorta a ROI da zona de pouso
    cv::Mat roi = gray(zone.zone);

    // Usa o seu detector ShiTomassi
    auto [keypoints, score] = detector.run(roi);

    prevPoints.clear();
    for (const auto& kp : keypoints) {
        // Converte de ROI para coordenadas da imagem
        prevPoints.emplace_back(kp.pt.x + zone.zone.x, kp.pt.y + zone.zone.y);
    }
}

bool LucasKanadeLandingZoneTracker::track(const cv::Mat& frame, LandingZoneCandidate& zoneAtualizada) {
    if (prevPoints.empty() || prevGray.empty()) return false;

    cv::Mat gray;
    if (frame.channels() == 3)
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    else
        gray = frame.clone();

    std::vector<cv::Point2f> nextPoints;
    std::vector<uchar> status;
    std::vector<float> err;

    // Optical Flow
    cv::calcOpticalFlowPyrLK(prevGray, gray, prevPoints, nextPoints, status, err);

    std::vector<cv::Point2f> validPoints;
    for (size_t i = 0; i < status.size(); i++) {
        if (status[i])
            validPoints.push_back(nextPoints[i]);
    }

    if (validPoints.empty()) return false;

    cv::Rect2f newZone = cv::boundingRect(validPoints);

    // Atualiza estados
    prevPoints = validPoints;
    prevGray = gray.clone();
    lastZone = newZone;

    zoneAtualizada.zone = newZone;
    zoneAtualizada.certainty = 1.0;
    zoneAtualizada.intensityVar = 0.0;
    zoneAtualizada.frameIndex = -1;

    return true;
}
