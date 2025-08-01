// landZONE_ing_node.hpp

#ifndef LANDING_ZONE_NODE_HPP
#define LANDING_ZONE_NODE_HPP


#include <memory>               // PARA std::unique_ptr
#include <opencv2/core.hpp>     // PARA cv::Mat
#include "landing_zone_system/ICapture.hpp"  // DECLARAÇÃO DE IVideoCapture

#include "landing_zone_system/FramePreProcessor.hpp"
#include "landing_zone_system/Detectors.hpp"
#include "landing_zone_system/LandingZoneChooser.hpp"
#include "landing_zone_system/VideoSourceFactory.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>


class LandingZoneNode : public rclcpp::Node
{
public:
    LandingZoneNode();
    void run_once(); // captura, processa e publica um frame

private:
    // setup methods
    void declare_parameters_();
    void load_parameters_();
    void init_publishers_();
    void init_video_source_();

    // processing stages
    bool capture_frame_(cv::Mat &frame);
    LandingZoneCandidate find_best_zone_(cv::Mat &frame);
    void annotate_frame_(cv::Mat &frame,
                         const LandingZoneCandidate &best);
    void publish_zone_(const LandingZoneCandidate &best);
    void publish_image_(cv::Mat &frame);

    // members
    std::string camera_topic_;
    int width_, height_, grid_rows_, grid_cols_;

    // ROS publishers
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr zone_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

    // processing objects
    std::unique_ptr<ShiTomassi> detector_;
    std::unique_ptr<ICapture> cap_;
    FramePreProcessor pre_;
    LandingZoneChooser chooser_;
};

#endif // LANDING_ZONE_NODE_HPP
