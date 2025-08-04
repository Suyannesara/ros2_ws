#include "landing_zone_system/LandingZoneTracker.hpp"

CSRTLandingZoneTracker::CSRTLandingZoneTracker()
{
    tracker = cv::TrackerCSRT::create();
}

void CSRTLandingZoneTracker::initialize(const cv::Mat &frame, const LandingZoneCandidate &zone)
{
    tracker->init(frame, zone.zone);
}

bool CSRTLandingZoneTracker::track(const cv::Mat &frame, LandingZoneCandidate &zoneAtualizada)
{
    cv::Rect trackedBox;
    bool success = tracker->update(frame, trackedBox);

    if (success)
    {
        zoneAtualizada.zone = trackedBox;
    }

    return success;
}

void TemplateMatchingLandingZoneTracker::initialize(const cv::Mat &frame, const LandingZoneCandidate &zone)
{
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
        zone.zone.y + zone.zone.height > grayFrame.rows)
    {
        std::cerr << "Erro: zona fora dos limites da imagem no initialize()." << std::endl;
        return;
    }

    templateImage = grayFrame(zone.zone).clone();
}

bool TemplateMatchingLandingZoneTracker::track(const cv::Mat &frame, LandingZoneCandidate &zoneAtualizada)
{
    if (frame.empty() || templateImage.empty())
        return false;

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

    return maxVal > 0.5; // Threshold suavizado
}

void LucasKanadeLandingZoneTracker::initialize(const cv::Mat &frame, const LandingZoneCandidate &zone)
{
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
    for (const auto &kp : keypoints)
    {
        // Converte de ROI para coordenadas da imagem
        prevPoints.emplace_back(kp.pt.x + zone.zone.x, kp.pt.y + zone.zone.y);
    }
}

bool LucasKanadeLandingZoneTracker::track(const cv::Mat &frame, LandingZoneCandidate &zoneAtualizada)
{
    if (prevPoints.empty() || prevGray.empty())
        return false;

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
    for (size_t i = 0; i < status.size(); i++)
    {
        if (status[i])
            validPoints.push_back(nextPoints[i]);
    }

    if (validPoints.empty())
        return false;

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

bool FeatureBasedTracker::track(const cv::Mat& frame,
                                LandingZoneCandidate& z)
{
    // 1) Checa se temos um template válido
    if (templDesc_.empty()) {
        RCLCPP_WARN(
            rclcpp::get_logger("FeatureBasedTracker"),
            "Template vazio em track(), abortando."
        );
        return false;
    }

    // 2) Converte para cinza
    cv::Mat gray;
    if (frame.channels() == 3) {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = frame;
    }

    // 3) Calcula janela de busca em torno da última caixa
    const int M = 50;  // margem em pixels
    int x = std::max(0, lastMatch_.x - M);
    int y = std::max(0, lastMatch_.y - M);
    int w = std::min(gray.cols - x, lastMatch_.width  + 2*M);
    int h = std::min(gray.rows - y, lastMatch_.height + 2*M);

    // 4) Checa se a janela é válida
    if (w  <= 0 || h  <= 0 ||
        x >= gray.cols || y >= gray.rows)
    {
        RCLCPP_WARN(
            rclcpp::get_logger("FeatureBasedTracker"),
            "ROI inválido em track(): x=%d y=%d w=%d h=%d",
            x, y, w, h
        );
        return false;
    }

    cv::Rect searchWindow(x, y, w, h);
    cv::Mat searchROI = gray(searchWindow);

    // 5) Detecta keypoints na ROI usando Shi–Tomasi
    auto [kps2, score2] = shi_.run(searchROI);
    // ajusta para coordenadas globais
    for (auto &kp : kps2) {
        kp.pt += cv::Point2f((float)x, (float)y);
    }

    // 6) Verifica se detectou algo
    if (kps2.empty()) {
        RCLCPP_WARN(
            rclcpp::get_logger("FeatureBasedTracker"),
            "Nenhum keypoint detectado na ROI"
        );
        return false;
    }

    // 7) Extrai descritores para esses keypoints
    cv::Mat desc2;
    orb_->compute(gray, kps2, desc2);

    // 8) Garante que temos descritores compatíveis
    if (desc2.empty() || desc2.cols != templDesc_.cols) {
        RCLCPP_WARN(
            rclcpp::get_logger("FeatureBasedTracker"),
            "Descritores inválidos: templDesc_.cols=%d, desc2.cols=%d",
            templDesc_.cols, desc2.cols
        );
        return false;
    }

    // 9) Faz o matching com Lowe's ratio test
    std::vector<std::vector<cv::DMatch>> knn;
    matcher_.knnMatch(templDesc_, desc2, knn, 2);
    std::vector<cv::DMatch> good;
    for (auto &m : knn) {
        if (m.size() >= 2 && m[0].distance < 0.75f * m[1].distance) {
            good.push_back(m[0]);
        }
    }
    if (good.size() < 4) {
        // não há matches suficientes para homografia
        return false;
    }

    // 10) Estima homografia entre template e frame
    std::vector<cv::Point2f> pts1, pts2;
    pts1.reserve(good.size());
    pts2.reserve(good.size());
    for (auto &m : good) {
        pts1.push_back(templKp_[m.queryIdx].pt);
        pts2.push_back(kps2   [m.trainIdx].pt);
    }
    cv::Mat H = cv::findHomography(pts1, pts2, cv::RANSAC);
    if (H.empty()) {
        return false;
    }

    // 11) Projeta os cantos do retângulo original
    std::vector<cv::Point2f> corners = {
        {0, 0},
        {static_cast<float>(lastMatch_.width), 0},
        {static_cast<float>(lastMatch_.width), static_cast<float>(lastMatch_.height)},
        {0, static_cast<float>(lastMatch_.height)}
    }, out;
    cv::perspectiveTransform(corners, out, H);
    cv::Rect newZone = cv::boundingRect(out);

    // 12) Trunca ao interior da imagem
    newZone &= cv::Rect(0, 0, gray.cols, gray.rows);

    // 13) Atualiza estado e candidato
    lastMatch_       = newZone;
    z.zone           = newZone;
    z.certainty      = static_cast<float>(good.size()) / static_cast<float>(templKp_.size());
    z.intensityVar   = 0.0f;
    z.frameIndex     = -1;

    return true;
}
