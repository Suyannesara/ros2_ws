#pragma once
#include <opencv2/opencv.hpp>
#include "LandingZoneChooser.hpp"
#include "Detectors.hpp"
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
#include <rclcpp/rclcpp.hpp> 


class LandingZoneTracker {
public:
    virtual ~LandingZoneTracker() = default;
    virtual void initialize(const cv::Mat& frame, const LandingZoneCandidate& zone) = 0;
    virtual bool track(const cv::Mat& frame, LandingZoneCandidate& zoneAtualizada) = 0;
};

class CSRTLandingZoneTracker : public LandingZoneTracker {
private:
    cv::Ptr<cv::Tracker> tracker;

public:
    CSRTLandingZoneTracker();
    void initialize(const cv::Mat& frame, const LandingZoneCandidate& zone) override;
    bool track(const cv::Mat& frame, LandingZoneCandidate& zoneAtualizada) override;
};

class TemplateMatchingLandingZoneTracker : public LandingZoneTracker {
private:
    cv::Mat templateImage;
    cv::Size templateSize;
    cv::Rect2d lastMatch;

public:
    void initialize(const cv::Mat& frame, const LandingZoneCandidate& zone) override;
    bool track(const cv::Mat& frame, LandingZoneCandidate& zoneAtualizada) override;
};

class LucasKanadeLandingZoneTracker : public LandingZoneTracker {
private:
    std::vector<cv::Point2f> prevPoints;
    cv::Rect2d lastZone;
    cv::Mat prevGray;
    ShiTomassi detector;

public:
    void initialize(const cv::Mat& frame, const LandingZoneCandidate& zone) override;
    bool track(const cv::Mat& frame, LandingZoneCandidate& zoneAtualizada) override;
};


class FeatureBasedTracker : public LandingZoneTracker {
  ShiTomassi shi_;                    // seu detector
  cv::Ptr<cv::ORB>   orb_    = cv::ORB::create(500);
  cv::BFMatcher       matcher_{cv::NORM_HAMMING};
  std::vector<cv::KeyPoint> templKp_;
  cv::Mat                  templDesc_;
  cv::Rect                 lastMatch_;

public:
  void initialize(const cv::Mat& frame,
                  const LandingZoneCandidate& zone) override
  {
    // 1) detecta corners no template
    cv::Mat gray; cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    auto [kps,_] = shi_.run(gray(zone.zone));
    templKp_ = kps;

    // 2) calc descriptors
    orb_->compute(gray(zone.zone), templKp_, templDesc_);

    // 3) guarda posição inicial
    lastMatch_ = zone.zone;
  }
  bool track(const cv::Mat& frame,LandingZoneCandidate& z);

  // bool track(const cv::Mat& frame,
  //            LandingZoneCandidate& z) override
  // {
  //   if (templDesc_.empty()) return false;

  //   // preparar ROI de busca
  //   cv::Mat gray; cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
  //   int M = 50; // margem
  //   cv::Rect win(
  //     std::max(0, lastMatch_.x - M),
  //     std::max(0, lastMatch_.y - M),
  //     std::min((int)gray.cols, lastMatch_.width + 2*M),
  //     std::min((int)gray.rows, lastMatch_.height + 2*M)
  //   );
  //   cv::Mat roi = gray(win);

  //   // detecta Shi-Tomasi na ROI
  //   auto [kps2, _score] = shi_.run(roi);
  //   // traduz keypoints para coords globais
  //   for (auto &kp : kps2)
  //     kp.pt += cv::Point2f(win.x, win.y);

  //   // calcula descritores
  //   cv::Mat desc2;
  //   orb_->compute(gray, kps2, desc2);

  //   // faz knnMatch + Lowe
  //   std::vector<std::vector<cv::DMatch>> knn;
  //   matcher_.knnMatch(templDesc_, desc2, knn, 2);
  //   std::vector<cv::DMatch> good;
  //   for (auto &m : knn) {
  //     if (m[0].distance < 0.75f * m[1].distance)
  //       good.push_back(m[0]);
  //   }
  //   if (good.size() < 4) return false;

  //   // monta pontos para homografia
  //   std::vector<cv::Point2f> pts1, pts2;
  //   for (auto &m : good) {
  //     pts1.push_back(templKp_[m.queryIdx].pt);
  //     pts2.push_back(kps2   [m.trainIdx].pt);
  //   }
  //   cv::Mat H = cv::findHomography(pts1, pts2, cv::RANSAC);
  //   if (H.empty()) return false;

  //   // projeta canto do retângulo inicial
  //   std::vector<cv::Point2f> corners = {
  //     {0,0},
  //     {(float)lastMatch_.width,0},
  //     {(float)lastMatch_.width,(float)lastMatch_.height},
  //     {0,(float)lastMatch_.height}
  //   }, out;
  //   cv::perspectiveTransform(corners, out, H);
  //   cv::Rect newZone = cv::boundingRect(out);

  //   lastMatch_ = newZone;
  //   z.zone     = newZone;
  //   z.certainty = (float)good.size() / templKp_.size();  // exemplo de certeza
  //   return true;
  // }
};
