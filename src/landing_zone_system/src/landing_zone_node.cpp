#include "landing_zone_node.hpp"
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <algorithm>
using namespace std::chrono_literals;

LandingZoneNode::LandingZoneNode()
    : Node("landing_zone_node"),
      detector_(std::make_unique<ShiTomassi>()),
      chooser_(6, 6),
      mode_(RunMode::IDLE) 
{
  declare_parameters_();
  load_parameters_();
  init_publishers_();
  init_video_source_();
  init_subscriptions_();
  init_tracker_();

  cv::namedWindow("Safe Landing View", cv::WINDOW_NORMAL);
  cv::resizeWindow("Safe Landing View", width_, height_);
}

void LandingZoneNode::declare_parameters_()
{
  declare_parameter("camera_topic", "/camera/image_raw");
  declare_parameter("width", 1280);
  declare_parameter("height", 960);
  declare_parameter("gridRows", 6);
  declare_parameter("gridCols", 6);
  declare_parameter("tracker_type", "lk");
}

void LandingZoneNode::load_parameters_()
{
  get_parameter("camera_topic", camera_topic_);
  get_parameter("width", width_);
  get_parameter("height", height_);
  get_parameter("gridRows", grid_rows_);
  get_parameter("gridCols", grid_cols_);
  get_parameter("tracker_type", tracker_type_);
  chooser_ = LandingZoneChooser(grid_rows_, grid_cols_);
}

void LandingZoneNode::init_publishers_()
{
  zone_pub_ = create_publisher<geometry_msgs::msg::Point>("zoneTopic", 10);
  image_pub_ = create_publisher<sensor_msgs::msg::Image>("annotated_image", 10);
}

void LandingZoneNode::init_video_source_()
{
  SearchConfig cfg;
  cfg.origem = "ros";
  cfg.cameraTopic = camera_topic_;
  cfg.width = width_;
  cfg.height = height_;
  cfg.gridRows = grid_rows_;
  cfg.gridCols = grid_cols_;
  cap_ = VideoSourceFactory::create(cfg);
}

void LandingZoneNode::init_subscriptions_()
{
  cmd_sub_ = create_subscription<std_msgs::msg::String>(
      "landing_zone_cmd", 10,
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        std::string s = msg->data;
        std::transform(s.begin(), s.end(), s.begin(), ::tolower);
        handle_cmd_(s);
      });
}

void LandingZoneNode::init_tracker_()
{
  if (tracker_type_ == "template") {
    tracker_ = std::make_unique<TemplateMatchingLandingZoneTracker>();
    RCLCPP_INFO(get_logger(), "Tracker selecionado: Template Matching");
  }
  else if (tracker_type_ == "feature") {
    tracker_ = std::make_unique<FeatureBasedTracker>();
    RCLCPP_INFO(get_logger(), "Tracker selecionado: Feature-Based (ShiTomasi+ORB)");
  }
  else if (tracker_type_ == "lk") {
    tracker_ = std::make_unique<LucasKanadeLandingZoneTracker>();
    RCLCPP_INFO(get_logger(), "Tracker selecionado: Lucas-Kanade");
  }
  else {
    tracker_ = std::make_unique<CSRTLandingZoneTracker>();
    RCLCPP_INFO(get_logger(), "Tracker selecionado: CSRT");
  }
  tracker_initialized_ = false;
}


void LandingZoneNode::run_once()
{
  cv::Mat frame;
  while (rclcpp::ok() && !capture_frame_(frame)) {}

  switch (mode_) {
    case RunMode::IDLE:
      process_idle_(frame);
      break;
    case RunMode::FIND:
      process_find_(frame);
      break;
    case RunMode::TRACK:
      process_track_(frame);
      break;
  }
}

void LandingZoneNode::process_idle_(cv::Mat &frame)
{
  display_status_(frame, "IDLE: esperando comando");
  publish_image_(frame);
}

void LandingZoneNode::process_find_(cv::Mat &frame)
{
  best_zone_ = find_best_zone_(frame);
  last_frame_ = frame.clone();
  annotate_frame_(frame, best_zone_);
  display_status_(frame, "FIND: procurando zona de pouso");
  publish_zone_(best_zone_);
  publish_image_(frame);
}

void LandingZoneNode::process_track_(cv::Mat &frame)
{
  if (!tracker_) init_tracker_();

  if (!tracker_initialized_ && best_zone_.zone.width > 0) {
    tracker_->initialize(last_frame_, best_zone_);
    tracker_initialized_ = true;
    int cx = best_zone_.zone.x + best_zone_.zone.width / 2;
    int cy = best_zone_.zone.y + best_zone_.zone.height / 2;
    RCLCPP_INFO(get_logger(), "Tracker inicializado a partir do FIND.");
  }

  std::string status_msg;
  if (tracker_initialized_) {
    LandingZoneCandidate updated = best_zone_;
    bool ok = tracker_->track(frame, updated);
    int cx = best_zone_.zone.x + best_zone_.zone.width / 2;
    int cy = best_zone_.zone.y + best_zone_.zone.height / 2;
    if (ok) {
      best_zone_ = updated;
      annotate_frame_(frame, best_zone_);
      status_msg = "TRACK: seguindo ponto em (" + std::to_string(cx) + ", " + std::to_string(cy) + ")";
    } else {
      status_msg = "TRACK: tracking perdido, ative o modo FIND";
      RCLCPP_WARN(get_logger(), "Tracking perdido. Continuando tentativa no próximo frame.");
    }
    display_status_(frame, status_msg);
    publish_zone_(best_zone_);
    publish_image_(frame);
  } else {
    display_status_(frame, "TRACK: aguardando zona válida para inicializar");
    publish_image_(frame);
  }
}

bool LandingZoneNode::capture_frame_(cv::Mat &frame)
{
  if (cap_->read(frame) && !frame.empty()) {
    if (frame.size() != cv::Size(width_, height_))
      cv::resize(frame, frame, {width_, height_});
    return true;
  }
  rclcpp::sleep_for(10ms);
  return false;
}

LandingZoneCandidate LandingZoneNode::find_best_zone_(cv::Mat &frame)
{
  cv::Mat gray = pre_.run(frame);
  auto [kps, score] = detector_->run(gray);
  return chooser_.getBestZone(kps, gray);
}

void LandingZoneNode::annotate_frame_(cv::Mat &frame, const LandingZoneCandidate &best)
{
  if (best.zone.width > 0 && best.zone.height > 0)
  {
    // Centro da zona
    cv::Point center(
      best.zone.x + best.zone.width  / 2,
      best.zone.y + best.zone.height / 2
    );

    // Tamanho fixo da mira (em pixels)
    const int length = 40;
    const int gap = 10;        // espaço no meio da cruz
    const int thickness = 2;
    cv::Scalar color(0, 255, 0);  // verde

    // Linha horizontal esquerda
    cv::line(
      frame,
      cv::Point(center.x - length, center.y),
      cv::Point(center.x - gap,    center.y),
      color, thickness
    );

    // Linha horizontal direita
    cv::line(
      frame,
      cv::Point(center.x + gap,    center.y),
      cv::Point(center.x + length, center.y),
      color, thickness
    );

    // Linha vertical cima
    cv::line(
      frame,
      cv::Point(center.x, center.y - length),
      cv::Point(center.x, center.y - gap),
      color, thickness
    );

    // Linha vertical baixo
    cv::line(
      frame,
      cv::Point(center.x, center.y + gap),
      cv::Point(center.x, center.y + length),
      color, thickness
    );

    // Círculo central
    cv::circle(
      frame,
      center,
      gap,
      color,
      thickness
    );
  }
}


void LandingZoneNode::publish_zone_(const LandingZoneCandidate &best)
{
  if (best.zone.width <= 0) return;
  geometry_msgs::msg::Point p;
  p.x = best.zone.x + best.zone.width / 2.0;
  p.y = best.zone.y + best.zone.height / 2.0;
  zone_pub_->publish(p);
}

void LandingZoneNode::publish_image_(cv::Mat &frame)
{
  std_msgs::msg::Header h;
  h.stamp = now();
  cv::Mat rgb;
  cv::cvtColor(frame, rgb, cv::COLOR_BGR2RGB);
  auto img_msg = cv_bridge::CvImage(h, "rgb8", rgb).toImageMsg();
  image_pub_->publish(*img_msg);
  RCLCPP_INFO(this->get_logger(), "Exibindo frame %dx%d", frame.cols, frame.rows);
  cv::imshow("Safe Landing View", frame);
  cv::waitKey(1);
}

void LandingZoneNode::display_status_(cv::Mat &frame, const std::string &msg)
{
  int font = cv::FONT_HERSHEY_SIMPLEX;
  double scale = 0.9;
  int thickness = 2;
  int baseline = 0;
  cv::Size textSize = cv::getTextSize(msg, font, scale, thickness, &baseline);
  cv::Point origin(10, textSize.height + 10);
  cv::rectangle(frame,
                origin + cv::Point(0, baseline),
                origin + cv::Point(textSize.width, -textSize.height),
                cv::Scalar(0, 0, 0), cv::FILLED);
  cv::putText(frame, msg, origin, font, scale,
              cv::Scalar(0, 255, 255), thickness);
}

void LandingZoneNode::handle_cmd_(const std::string &cmd)
{
  if (cmd == "find") {
    set_mode_(RunMode::FIND);
  } else if (cmd == "track") {
    set_mode_(RunMode::TRACK);
  } else if (cmd == "exit") {
    RCLCPP_INFO(get_logger(), "Encerrando por comando de tópico...");
    rclcpp::shutdown();
  } else {
    RCLCPP_WARN(get_logger(), "Comando inválido: %s", cmd.c_str());
  }
}

void LandingZoneNode::set_mode_(RunMode m)
{
  mode_ = m;
  if (m == RunMode::TRACK) {
    tracker_initialized_ = false;
  }
  RCLCPP_INFO(get_logger(), "Modo agora: %s",
              m == RunMode::FIND ? "FIND" : m == RunMode::TRACK ? "TRACK" : "IDLE");
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LandingZoneNode>();
  rclcpp::Rate rate(10);
  while (rclcpp::ok()) {
    node->run_once();
    rclcpp::spin_some(node);
    rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
