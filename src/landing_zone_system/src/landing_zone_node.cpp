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
      chooser_(/*rows=*/6, /*cols=*/6) // temporário, será sobrescrito em load_parameters_
{
  declare_parameters_();
  load_parameters_();
  init_publishers_();
  init_video_source_();
  init_subscriptions_();
  init_tracker_(); // preparar tracker para o modo TRACK

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

  // opcional: permitir selecionar o tracker por parâmetro
  declare_parameter("tracker_type", "lk"); // "csrt" ou "template"
}

void LandingZoneNode::load_parameters_()
{
  get_parameter("camera_topic", camera_topic_);
  get_parameter("width", width_);
  get_parameter("height", height_);
  get_parameter("gridRows", grid_rows_);
  get_parameter("gridCols", grid_cols_);
  get_parameter("tracker_type", tracker_type_);

  // reconfigura chooser com parâmetros reais
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

bool LandingZoneNode::capture_frame_(cv::Mat &frame)
{
  // tenta ler um frame, espera brevemente se falhar
  if (cap_->read(frame) && !frame.empty())
  {
    if (frame.size() != cv::Size(width_, height_))
      cv::resize(frame, frame, {width_, height_});
    return true;
  }
  rclcpp::sleep_for(10ms);
  return false;
}

LandingZoneCandidate LandingZoneNode::find_best_zone_(cv::Mat &frame)
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
  if (best.zone.width > 0 && best.zone.height > 0)
  {
    cv::rectangle(frame, best.zone, cv::Scalar(0, 255, 0), 3);
  }
}

void LandingZoneNode::publish_zone_(
    const LandingZoneCandidate &best)
{
  if (best.zone.width <= 0 || best.zone.height <= 0)
    return;

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

  // opcional: exibe localmente
  RCLCPP_INFO(this->get_logger(), "Mostrando imagem com tamanho %d x %d", frame.cols, frame.rows);

  cv::imshow("Safe Landing View", frame);
  cv::waitKey(1);
}

/** Inicializa o tracker concreto conforme parâmetro "tracker_type".
 *  Chamada no construtor; pode ser chamada novamente se quiser trocar em runtime.
 */
void LandingZoneNode::init_tracker_()
{
  std::string tracker_type = "csrt";
  (void)get_parameter("tracker_type", tracker_type);

  if (tracker_type == "template") {
    tracker_ = std::make_unique<TemplateMatchingLandingZoneTracker>();
    RCLCPP_INFO(get_logger(), "Tracker selecionado: Template Matching");
  } else if (tracker_type == "lk") {
    tracker_ = std::make_unique<LucasKanadeLandingZoneTracker>();
    RCLCPP_INFO(get_logger(), "Tracker selecionado: Lucas-Kanade");
  } else {
    tracker_ = std::make_unique<CSRTLandingZoneTracker>();
    RCLCPP_INFO(get_logger(), "Tracker selecionado: CSRT");
  }

  tracker_initialized_ = false;
}


/** Loop de uma iteração respeitando o modo FIND/TRACK. */
void LandingZoneNode::run_once()
{
  cv::Mat frame;
  // 1) captura um frame
  while (rclcpp::ok() && !capture_frame_(frame))
  {
  }

  if (mode_ == RunMode::FIND)
  {
    // 2) encontra melhor zona
    best_zone_ = find_best_zone_(frame);
    last_frame_ = frame.clone(); // salva para TRACK

    // 3) anota
    annotate_frame_(frame, best_zone_);

    // 4) publica coordenadas e imagem
    publish_zone_(best_zone_);
    publish_image_(frame);
  }
  else
  { // TRACK
    if (!tracker_)
    {
      init_tracker_();
    }

    // Se ainda não inicializamos o tracker, inicialize agora usando o último frame detectado no FIND
    if (!tracker_initialized_ && best_zone_.zone.width > 0 && best_zone_.zone.height > 0)
    {
      tracker_->initialize(last_frame_, best_zone_);
      tracker_initialized_ = true;
      RCLCPP_INFO(this->get_logger(), "Tracker inicializado a partir do FIND.");
    }

    if (tracker_initialized_)
    {
      LandingZoneCandidate updated = best_zone_;
      bool ok = tracker_->track(frame, updated);

      if (ok)
      {
        best_zone_ = updated;
        annotate_frame_(frame, best_zone_);
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Tracking perdido. Ative novamente o modo FIND.");
        best_zone_.zone = cv::Rect();
        cv::putText(frame,
                    "Tracking perdido - use modo FIND",
                    cv::Point(50, 50),
                    cv::FONT_HERSHEY_SIMPLEX,
                    1.0,
                    cv::Scalar(0, 0, 255),
                    2);
      }

      publish_zone_(best_zone_);
      publish_image_(frame);
    }
    else
    {
      // Sem tracker válido, apenas publique a imagem
      publish_image_(frame);
    }
  }
}

void LandingZoneNode::init_subscriptions_()
{
  cmd_sub_ = create_subscription<std_msgs::msg::String>(
      "landing_zone_cmd", 10,
      [this](std_msgs::msg::String::ConstSharedPtr msg)
      {
        std::string s = msg->data;
        std::transform(s.begin(), s.end(), s.begin(), ::tolower);
        handle_cmd_(s);
      });
}

void LandingZoneNode::set_mode_(RunMode m)
{
  mode_ = m;
  if (mode_ == RunMode::TRACK)
  {
    tracker_initialized_ = false; // força reinit do tracker no próximo run_once()
  }
  RCLCPP_INFO(get_logger(), "Modo agora: %s", mode_ == RunMode::FIND ? "FIND" : "TRACK");
}

void LandingZoneNode::handle_cmd_(const std::string &cmd)
{
  if (cmd == "find")
  {
    set_mode_(RunMode::FIND);
  }
  else if (cmd == "track")
  {
    set_mode_(RunMode::TRACK);
  }
  else if (cmd == "exit")
  {
    RCLCPP_INFO(get_logger(), "Encerrando por comando de tópico...");
    rclcpp::shutdown();
  }
  else
  {
    RCLCPP_WARN(get_logger(), "Comando inválido: %s (use: find | track | exit)", cmd.c_str());
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LandingZoneNode>();

  rclcpp::Rate rate(10); // 10 Hz
  while (rclcpp::ok())
  {
    node->run_once();
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
