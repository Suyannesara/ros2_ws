// landZONE_ing_node.hpp

#ifndef LANDING_ZONE_NODE_HPP
#define LANDING_ZONE_NODE_HPP

#include <memory>                           // PARA std::unique_ptr
#include <opencv2/core.hpp>                 // PARA cv::Mat
#include "landing_zone_system/ICapture.hpp" // DECLARAÇÃO DE IVideoCapture

#include "landing_zone_system/FramePreProcessor.hpp"
#include "landing_zone_system/Detectors.hpp"
#include "landing_zone_system/LandingZoneChooser.hpp"
#include "landing_zone_system/VideoSourceFactory.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>

#include <std_msgs/msg/string.hpp>
#include "landing_zone_system/LandingZoneTracker.hpp"

class LandingZoneNode : public rclcpp::Node
{
public:
    LandingZoneNode();
    void run_once(); // captura, processa e publica um frame

    enum class RunMode{ FIND, TRACK };
    RunMode mode_ = RunMode::FIND;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;

    
    bool tracker_initialized_ = false; // se usar TRACK

private:
    // setup methods
    void declare_parameters_();
    void load_parameters_();
    void init_publishers_();
    void init_tracker_();
    void init_subscriptions_();
    void handle_cmd_(const std::string &cmd);
    void set_mode_(RunMode m);
    void init_video_source_();

    
    // processing stages
    bool capture_frame_(cv::Mat &frame);
    LandingZoneCandidate find_best_zone_(cv::Mat &frame);
    void annotate_frame_(cv::Mat &frame,
                         const LandingZoneCandidate &best);
    void publish_zone_(const LandingZoneCandidate &best);
    void publish_image_(cv::Mat &frame);

    // members
    std::string camera_topic_, tracker_type_;
    int width_, height_, grid_rows_, grid_cols_;

    // ROS publishers
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr zone_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

    // processing objects
    LandingZoneCandidate best_zone_;
    std::unique_ptr<LandingZoneTracker> tracker_;
    std::unique_ptr<ShiTomassi> detector_;
    std::unique_ptr<ICapture> cap_;
    FramePreProcessor pre_;
    LandingZoneChooser chooser_;
    cv::Mat last_frame_;

};

#endif // LANDING_ZONE_NODE_HPP
