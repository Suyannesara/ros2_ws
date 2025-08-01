#include "landing_zone_node.hpp"

using namespace std::chrono_literals;

LandingZoneNode::LandingZoneNode()
: Node("landing_zone_node"),
  detector_(std::make_unique<ShiTomassi>()),
  chooser_(/*rows=*/6, /*cols=*/6)  // temporário, será sobrescrito em load_parameters_
{
  declare_parameters_();
  load_parameters_();
  init_publishers_();
  init_video_source_();

  // opcional: criar janela
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
}

void LandingZoneNode::load_parameters_()
{
  get_parameter("camera_topic", camera_topic_);
  get_parameter("width", width_);
  get_parameter("height", height_);
  get_parameter("gridRows", grid_rows_);
  get_parameter("gridCols", grid_cols_);

  // reconfigura chooser com parâmetros reais
  chooser_ = LandingZoneChooser(grid_rows_, grid_cols_);
}

void LandingZoneNode::init_publishers_()
{
  zone_pub_  = create_publisher<geometry_msgs::msg::Point>("zoneTopic",      10);
  image_pub_ = create_publisher<sensor_msgs::msg::Image>("annotated_image", 10);
}

void LandingZoneNode::init_video_source_()
{
  SearchConfig cfg;
  cfg.origem      = "ros";
  cfg.cameraTopic = camera_topic_;
  cfg.width       = width_;
  cfg.height      = height_;
  cfg.gridRows    = grid_rows_;
  cfg.gridCols    = grid_cols_;
  cap_ = VideoSourceFactory::create(cfg);
}

bool LandingZoneNode::capture_frame_(cv::Mat &frame)
{
  // tenta ler um frame, espera brevemente se falhar
  if (cap_->read(frame) && !frame.empty()) {
    if (frame.size() != cv::Size(width_, height_))
      cv::resize(frame, frame, {width_, height_});
    return true;
  }
  rclcpp::sleep_for(10ms);
  return false;
}

LandingZoneCandidate
LandingZoneNode::find_best_zone_(cv::Mat &frame)
{
  // pré-processamento e ShiTomassi
  cv::Mat gray = pre_.run(frame);
  auto [kps, score] = detector_->run(gray);

  // aplica o chooser no único frame
  auto best = chooser_.getBestZone(kps, gray);
  return best;
}

void LandingZoneNode::annotate_frame_(
    cv::Mat &frame,
    const LandingZoneCandidate &best)
{
  cv::rectangle(frame, best.zone, cv::Scalar(0,255,0), 3);
}

void LandingZoneNode::publish_zone_(
    const LandingZoneCandidate &best)
{
  geometry_msgs::msg::Point p;
  p.x = best.zone.x + best.zone.width  / 2.0;
  p.y = best.zone.y + best.zone.height / 2.0;
  zone_pub_->publish(p);
}

void LandingZoneNode::publish_image_(cv::Mat &frame)
{
  std_msgs::msg::Header h; h.stamp = now();
  cv::Mat rgb;
  cv::cvtColor(frame, rgb, cv::COLOR_BGR2RGB);
  auto img_msg = cv_bridge::CvImage(h, "rgb8", rgb).toImageMsg();
  image_pub_->publish(*img_msg);

  // opcional: exibe localmente
  RCLCPP_INFO(this->get_logger(), "Mostrando imagem com tamanho %d x %d", frame.cols, frame.rows);

  cv::imshow("Safe Landing View", frame);
  cv::waitKey(1);
}

void LandingZoneNode::run_once()
{
  cv::Mat frame;
  // 1) captura um frame
  while (rclcpp::ok() && !capture_frame_(frame)) {}

  // 2) encontra melhor zona
  auto best = find_best_zone_(frame);

  // 3) anota
  annotate_frame_(frame, best);

  // 4) publica coordenadas e imagem
  publish_zone_(best);
  publish_image_(frame);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LandingZoneNode>();

  rclcpp::Rate rate(10);  // 10 Hz
  while (rclcpp::ok()) {
    node->run_once();
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}

