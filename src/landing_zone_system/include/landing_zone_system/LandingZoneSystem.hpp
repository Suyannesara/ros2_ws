#ifndef LANDING_ZONE_SYSTEM_HPP
#define LANDING_ZONE_SYSTEM_HPP

#include <string>
#include "LandingZoneChooser.hpp"
#include "LandingZoneTracker.hpp"
#include "SearchConfig.hpp"

class LandingZoneSystem
{
public:
    LandingZoneSystem();
    LandingZoneCandidate findLandingZone(
        const SearchConfig &cfg,
        std::function<void(cv::Mat &frame,
                           const LandingZoneCandidate &candidate,
                           bool isCurrent)>
            visualize);

    LandingZoneCandidate findLandingZone(const SearchConfig &config);
    bool trackZone(const LandingZoneCandidate &zonaInicial, const SearchConfig &config);
    bool findAndTrack(const SearchConfig &config, LandingZoneTracker *tracker);
};

#endif
