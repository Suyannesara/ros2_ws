// // safe_landing_node.cpp

// #include "landing_zone_system/VideoSourceFactory.hpp"
// #include "landing_zone_system/FramePreProcessor.hpp"
// #include "landing_zone_system/Detectors.hpp"
// #include "landing_zone_system/LandingZoneChooser.hpp"
// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/point.hpp"
// #include "sensor_msgs/msg/image.hpp"
// #include <cv_bridge/cv_bridge.h>
// #include <opencv2/imgproc.hpp>
// #include <opencv2/highgui.hpp>

// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = rclcpp::Node::make_shared("safe_landing_node");

//   // parâmetros fixos (podem vir de ROS params, se quiser)
//   const std::string camera_topic = "/camera/image_raw";
//   const int width    = 1280, height = 960;
//   const int gridRows = 6,    gridCols = 6;

//   // publishers
//   auto zone_pub  = node->create_publisher<geometry_msgs::msg::Point>("zoneTopic",      10);
//   auto image_pub = node->create_publisher<sensor_msgs::msg::Image>("annotated_image", 10);

//   // configura a captura ROS/Gazebo
//   SearchConfig cfg;
//   cfg.origem            = "ros";
//   cfg.cameraTopic       = camera_topic;
//   cfg.tempoBuscaSegundos= 0;                // não usado aqui
//   cfg.width             = width;
//   cfg.height            = height;
//   cfg.gridRows          = gridRows;
//   cfg.gridCols          = gridCols;
//   auto cap = VideoSourceFactory::create(cfg);

//   FramePreProcessor pre;
//   auto detector = std::make_unique<ShiTomassi>();
//   LandingZoneChooser chooser(gridRows, gridCols);

//   // opcional: mostrar janela localmente
//   cv::namedWindow("Safe Landing View", cv::WINDOW_NORMAL);
//   cv::resizeWindow("Safe Landing View", width, height);

//   cv::Mat frame;
//   // 1) lê só **um** frame
//   while (rclcpp::ok()) {
//     rclcpp::spin_some(node);
//     if (cap->read(frame) && !frame.empty()) {
//       break;
//     }
//     std::this_thread::sleep_for(std::chrono::milliseconds(10));
//   }

//   // 2) garante tamanho
//   if (frame.size() != cv::Size(width, height)) {
//     cv::resize(frame, frame, {width, height});
//   }

//   // 3) pré-processa e detecta keypoints
//   cv::Mat gray = pre.run(frame);
//   auto [kps, score] = detector->run(gray);

//   // 4) escolhe o melhor bloco **usando seu LandingZoneChooser**
//   LandingZoneCandidate best = chooser.getBestZone(kps, gray);

//   // 5) desenha o retângulo grosso em volta do bloco
//   cv::rectangle(frame, best.zone, cv::Scalar(0,255,0), 3);

//   // 6) calcula o centro e publica as coordenadas em zoneTopic
//   geometry_msgs::msg::Point p;
//   p.x = best.zone.x + best.zone.width  / 2.0;
//   p.y = best.zone.y + best.zone.height / 2.0;
//   zone_pub->publish(p);

//   // 7) exibe localmente (opcional)
//   cv::imshow("Safe Landing View", frame);
//   cv::waitKey(1);

//   // 8) converte para RGB e publica a imagem anotada
//   std_msgs::msg::Header h; h.stamp = node->now();
//   cv::Mat rgb;
//   cv::cvtColor(frame, rgb, cv::COLOR_BGR2RGB);
//   auto img_msg = cv_bridge::CvImage(h, "rgb8", rgb).toImageMsg();
//   image_pub->publish(*img_msg);

//   // 9) finaliza
//   cv::destroyWindow("Safe Landing View");
//   rclcpp::shutdown();
//   return 0;
// }
