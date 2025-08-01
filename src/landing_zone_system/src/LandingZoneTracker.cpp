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

// void TemplateMatchingLandingZoneTracker::initialize(const cv::Mat& frame, const LandingZoneCandidate& zone) {
//     std::cout << "INICIEI O TRACKER" << std::endl;
//     lastMatch = zone.zone;
//     templateSize = zone.zone.size();
//     templateImage = frame(zone.zone).clone();
// }

// bool TemplateMatchingLandingZoneTracker::track(const cv::Mat& frame, LandingZoneCandidate& zoneAtualizada) {
//     if (frame.empty() || templateImage.empty()) return false;

//     cv::Mat result;
//     cv::matchTemplate(frame, templateImage, result, cv::TM_CCOEFF_NORMED);

//     double maxVal;
//     cv::Point maxLoc;
//     cv::minMaxLoc(result, nullptr, &maxVal, nullptr, &maxLoc);

//     lastMatch = cv::Rect2d(maxLoc.x, maxLoc.y, templateSize.width, templateSize.height);

//     zoneAtualizada.zone = lastMatch;
//     zoneAtualizada.certainty = maxVal;
//     zoneAtualizada.intensityVar = 0.0; // Ou compute se quiser
//     zoneAtualizada.frameIndex = -1; // Atualize conforme necessário

//     return maxVal > 0.7; // Threshold que você escolher
// }


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


