#include "landing_zone_system/VideoAnnotator.hpp"

VideoAnnotator::VideoAnnotator(int width, int height, double fps, const std::string& outputPath) {
    writer.open(outputPath, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, cv::Size(width, height));
    isOpen = writer.isOpened();

    if (!isOpen) {
        throw std::runtime_error("Erro ao abrir o arquivo de vídeo de saída: " + outputPath);
    }
}

VideoAnnotator::~VideoAnnotator() {
    close();
}

void VideoAnnotator::drawCurrentZone(cv::Mat& frame, const cv::Rect& zone, const cv::Scalar& color) {
    if (zone.area() > 0) {
        cv::rectangle(frame, zone, color, 2);
    }
}

void VideoAnnotator::writeFrame(const cv::Mat& frame) {
    if (isOpen) {
        writer.write(frame);
    }
}

void VideoAnnotator::close() {
    if (isOpen) {
        writer.release();
        isOpen = false;
    }
}
