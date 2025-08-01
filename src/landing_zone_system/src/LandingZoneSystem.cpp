#include "landing_zone_system/LandingZoneSystem.hpp"
#include "landing_zone_system/VideoSourceFactory.hpp"
#include "landing_zone_system/VideoAnnotator.hpp"
#include "landing_zone_system/FramePreProcessor.hpp"
#include "landing_zone_system/Detectors.hpp"
#include "landing_zone_system/LandingZoneTracker.hpp"

#include <opencv2/opencv.hpp>
#include <chrono>
#include <iostream>

LandingZoneCandidate LandingZoneSystem::findLandingZone(
    const SearchConfig &cfg,
    std::function<void(cv::Mat &,
                       const LandingZoneCandidate &,
                       bool)>
        visualize)
{
    // 1) Abra a fonte de vídeo (Gazebo ou arquivo)
    auto cap = VideoSourceFactory::create(cfg);
    if (!cap->isOpened())
    {
        std::cerr << "Não abriu a fonte de vídeo!" << std::endl;
        return {};
    }

    FramePreProcessor pre;
    auto detector = std::make_unique<ShiTomassi>();
    LandingZoneChooser chooser(cfg.gridRows, cfg.gridCols);
    std::vector<std::vector<cv::KeyPoint>> kpFrames;
    std::vector<cv::Mat> grayFrames;

    auto start = std::chrono::steady_clock::now();
    cv::Mat frame;

    // 2) Loop até estourar o tempo
    while (cap->read(frame))
    {
        double elapsed = std::chrono::duration<double>(
                             std::chrono::steady_clock::now() - start)
                             .count();
        if (elapsed > cfg.tempoBuscaSegundos)
            break;

        // redimensiona para garantir o tamanho
        if (frame.size() != cv::Size(cfg.width, cfg.height))
        {
            cv::resize(frame, frame, cv::Size(cfg.width, cfg.height));
        }

        // pré-processamento e detecção
        auto proc = pre.run(frame);
        auto [kps, score] = detector->run(proc);

        kpFrames.push_back(kps);
        grayFrames.push_back(proc);

        // avalia o melhor bloco até agora e desenha-o
        // auto current = chooser.evaluateMultipleFrames(kpFrames, grayFrames);
        // visualize(frame, current, /*isCurrent=*/true);
        auto current = chooser.evaluateMultipleFrames(kpFrames, grayFrames);
        RCLCPP_INFO(rclcpp::get_logger("LandingZoneSystem"),
                        "Frame %zu: current.zone = [x=%d,y=%d,w=%d,h=%d], keypoints=%zu",
                        kpFrames.size(),
                        current.zone.x, current.zone.y,
                        current.zone.width, current.zone.height,
                        kps.size());
        visualize(frame, current, true);
    }

    

    // 3) resultado final após o loop
    auto result = chooser.evaluateMultipleFrames(kpFrames, grayFrames);
    visualize(frame, result, /*isCurrent=*/false);

    return result;
}

LandingZoneSystem::LandingZoneSystem() {}

LandingZoneCandidate LandingZoneSystem::findLandingZone(const SearchConfig &cfg)
{
    auto cap = VideoSourceFactory::create(cfg);
    if (!cap->isOpened())
    {
        std::cerr << "Não abriu a fonte de vídeo!" << std::endl;
        return {};
    }

    FramePreProcessor pre;
    auto detector = std::make_unique<ShiTomassi>();
    LandingZoneChooser chooser(cfg.gridRows, cfg.gridCols);

    std::vector<std::vector<cv::KeyPoint>> kpFrames;
    std::vector<cv::Mat> grayFrames;
    std::unique_ptr<VideoAnnotator> annot;

    if (cfg.salvarVideo)
        annot = std::make_unique<VideoAnnotator>(
            cfg.width, cfg.height, cfg.fps, cfg.saidaVideoPath);

    auto start = std::chrono::steady_clock::now();
    cv::Mat frame;

    while (cap->read(frame))
    {
        double elapsed = std::chrono::duration<double>(
                             std::chrono::steady_clock::now() - start)
                             .count();
        if (elapsed > cfg.tempoBuscaSegundos)
            break;

        if (frame.size() != cv::Size(cfg.width, cfg.height))
            cv::resize(frame, frame, {cfg.width, cfg.height});

        auto proc = pre.run(frame);
        auto [kps, score] = detector->run(proc);

        kpFrames.push_back(kps);
        grayFrames.push_back(proc);

        auto current = chooser.evaluateMultipleFrames(kpFrames, grayFrames);
        if (annot)
        {
            annot->drawCurrentZone(frame, current.zone);
            annot->writeFrame(frame);
        }
    }

    if (annot)
        annot->close();
    return chooser.evaluateMultipleFrames(kpFrames, grayFrames);
}

// bool LandingZoneSystem::trackZone(const LandingZoneCandidate& zonaInicial, const SearchConfig& config) {
//     VideoSource videoSource;
//     cv::VideoCapture cap;

//     if (config.origem == "video") {
//         cap = videoSource.getVideoCapture(config.caminhoVideo);
//     } else if (config.origem == "camera") {
//         cap = videoSource.getCameraCapture();
//     } else {
//         std::cerr << "Origem '" << config.origem << "' não suportada para tracking." << std::endl;
//         return false;
//     }

//     if (!cap.isOpened()) {
//         std::cerr << "Erro ao abrir fonte de vídeo para tracking." << std::endl;
//         return false;
//     }

//     cv::Mat frame;
//     cap >> frame;
//     if (frame.empty()) return false;

//     CSRTLandingZoneTracker tracker;
//     tracker.initialize(frame, zonaInicial);

//     while (true) {
//         cap >> frame;
//         if (frame.empty()) break;

//         LandingZoneCandidate zonaAtual;
//         bool sucesso = tracker.track(frame, zonaAtual);

//         if (sucesso) {
//             cv::rectangle(frame, zonaAtual.zone, cv::Scalar(0, 255, 0), 2);
//         } else {
//             cv::putText(frame, "Tracking perdido", {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {0, 0, 255}, 2);
//         }

//         cv::imshow("Tracking Zona de Pouso", frame);
//         if (cv::waitKey(30) == 27) break; // ESC para sair
//     }

//     cap.release();
//     cv::destroyAllWindows();
//     return true;
// }

// bool LandingZoneSystem::findAndTrack(const SearchConfig& config, LandingZoneTracker* tracker) {
//     VideoSource videoSource;
//     cv::VideoCapture cap;

//     if (config.origem == "video") {
//         cap = videoSource.getVideoCapture(config.caminhoVideo);
//     } else if (config.origem == "camera") {
//         cap = videoSource.getCameraCapture();
//     } else {
//         std::cerr << "Origem '" << config.origem << "' não suportada." << std::endl;
//         return false;
//     }

//     if (!cap.isOpened()) {
//         std::cerr << "Erro ao abrir fonte de vídeo." << std::endl;
//         return false;
//     }

//     double fps = cap.get(cv::CAP_PROP_FPS);
//     if (fps <= 0.0 || fps > 120.0) {
//         fps = 30.0;  // fallback
//     }

//     FramePreProcessor preProcessor;
//     Detector* detector = new ShiTomassi();
//     LandingZoneChooser landingZoneChooser(6, 6);

//     VideoAnnotator* annotator = nullptr;
//     if (config.salvarVideo && !config.saidaVideoPath.empty()) {
//         annotator = new VideoAnnotator(config.width, config.height, static_cast<int>(fps), config.saidaVideoPath);
//     }

//     std::vector<std::vector<cv::KeyPoint>> keypointsFrames;
//     std::vector<cv::Mat> grayFrames;

//     auto start = std::chrono::steady_clock::now();
//     LandingZoneCandidate finalZone;
//     bool encontrou = false;
//     cv::Mat frame;

//     // ---------- Fase de busca ----------
//     while (true) {
//         cap >> frame;
//         if (frame.empty()) break;

//         auto now = std::chrono::steady_clock::now();
//         double elapsed = std::chrono::duration<double>(now - start).count();
//         if (elapsed > config.tempoBuscaSegundos) {
//             encontrou = true;
//             break;
//         }

//         if (frame.cols != config.width || frame.rows != config.height)
//             cv::resize(frame, frame, cv::Size(config.width, config.height));

//         cv::Mat processed = preProcessor.run(frame);
//         auto [keypoints, _] = detector->run(processed);

//         keypointsFrames.push_back(keypoints);
//         grayFrames.push_back(processed);

//         LandingZoneCandidate current = landingZoneChooser.evaluateMultipleFrames(keypointsFrames, grayFrames);
//         if (annotator) {
//             annotator->drawCurrentZone(frame, current.zone);
//             annotator->writeFrame(frame);
//         }
//     }

//     if (!encontrou) {
//         std::cerr << "Nenhuma zona de pouso encontrada." << std::endl;
//         if (annotator) {
//             annotator->close();
//             delete annotator;
//         }
//         delete detector;
//         return false;
//     }

//     finalZone = landingZoneChooser.evaluateMultipleFrames(keypointsFrames, grayFrames);

//     // Usa o último frame processado (cinza e do mesmo tamanho) para o initialize
//     cv::Mat initFrame;
//     if (!grayFrames.empty()) {
//         cv::cvtColor(grayFrames.back(), initFrame, cv::COLOR_GRAY2BGR);  // Converte para 3 canais se necessário
//         tracker->initialize(initFrame, finalZone);
//     }

//     if (annotator) {
//         annotator->drawCurrentZone(frame, finalZone.zone);
//         annotator->writeFrame(frame);
//     }

//     while (true) {
//         cap >> frame;
//         if (frame.empty()) break;

//         if (frame.cols != config.width || frame.rows != config.height)
//             cv::resize(frame, frame, cv::Size(config.width, config.height));

//         LandingZoneCandidate zonaAtual;
//         bool sucesso = tracker->track(frame, zonaAtual);

//         if (sucesso && annotator) {
//             annotator->drawCurrentZone(frame, zonaAtual.zone);
//         }

//         if (annotator) {
//             annotator->writeFrame(frame);
//         }
//     }

//     // ---------- Finalização ----------
//     if (annotator) {
//         annotator->close();
//         delete annotator;
//     }

//     delete detector;
//     cap.release();
//     cv::destroyAllWindows();
//     return true;
// }
