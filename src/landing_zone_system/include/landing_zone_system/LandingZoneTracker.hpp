#pragma once
#include <opencv2/opencv.hpp>
#include "LandingZoneChooser.hpp"
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>


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

