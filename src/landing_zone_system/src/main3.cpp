// // landing_zone_node.cpp
// #include "landing_zone_system/LandingZoneSystem.hpp"
// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/empty.hpp"
// #include "geometry_msgs/msg/point.hpp"
// #include "sensor_msgs/msg/image.hpp"
// #include <cv_bridge/cv_bridge.h>
// #include <opencv2/imgproc.hpp>

// int main(int argc, char **argv)
// {
//   // Inicializa ROS2
//   rclcpp::init(argc, argv);
//   auto node = rclcpp::Node::make_shared("landing_zone_node");

//   // Declara e recupera parâmetros
//   SearchConfig cfg;
//   node->declare_parameter<std::string>("origem", "ros");
//   node->declare_parameter<std::string>("cameraTopic", "/camera/image_raw");
//   node->declare_parameter<int>("tempoBuscaSegundos", 10);
//   node->declare_parameter<int>("width", 640);
//   node->declare_parameter<int>("height", 480);
//   node->declare_parameter<double>("fps", 30.0);
//   node->declare_parameter<int>("gridRows", 6);
//   node->declare_parameter<int>("gridCols", 6);

//   node->get_parameter("origem", cfg.origem);
//   node->get_parameter("cameraTopic", cfg.cameraTopic);
//   node->get_parameter("tempoBuscaSegundos", cfg.tempoBuscaSegundos);
//   node->get_parameter("width", cfg.width);
//   node->get_parameter("height", cfg.height);
//   node->get_parameter("fps", cfg.fps);
//   node->get_parameter("gridRows", cfg.gridRows);
//   node->get_parameter("gridCols", cfg.gridCols);

//   // Publishers para resultado e imagem anotada
//   auto zone_pub = node->create_publisher<geometry_msgs::msg::Point>("zoneTopic", 10);
//   auto image_pub = node->create_publisher<sensor_msgs::msg::Image>("/annotated_image", 10);

//   // Instancia o sistema de busca
//   LandingZoneSystem lzs;

//   // Subscriber para comando de busca

//   auto sub = node->create_subscription<std_msgs::msg::Empty>(
//       "find", 10,
//       [node, &lzs, &cfg, zone_pub, image_pub](std_msgs::msg::Empty::SharedPtr)
//       {
//         RCLCPP_INFO(node->get_logger(), "Find command received. Starting search...");
//         // Executa busca com callback de visualização
//         auto result = lzs.findLandingZone(
//             cfg,
//             [node, zone_pub, image_pub](cv::Mat &frame,
//                                         const LandingZoneCandidate &cand,
//                                         bool isCurrent)
//             {
//               // 1) Se não há frame, ignora
//               if (frame.empty())
//               {
//                 RCLCPP_WARN(node->get_logger(), "visualize: received empty frame, skipping");
//                 return;
//               }

//               // 2) Desenha o retângulo fino para os estados intermediários
//               //    ou grosso no estado final
//               cv::Scalar color(0, 255, 0);
//               int thickness = isCurrent ? 1 : 3;
//               cv::rectangle(frame, cand.zone, color, thickness);

//               // 3) Se for o estado final, publica também as coordenadas
//               if (!isCurrent)
//               {
//                 geometry_msgs::msg::Point p;
//                 p.x = cand.zone.x + cand.zone.width / 2;
//                 p.y = cand.zone.y + cand.zone.height / 2;
//                 zone_pub->publish(p);
//               }

//               // 4) Converte para RGB e publica o frame anotado em cada iteração
//               std_msgs::msg::Header h;
//               h.stamp = node->now();
//               cv::Mat rgb;
//               cv::cvtColor(frame, rgb, cv::COLOR_BGR2RGB);
//               auto img_msg = cv_bridge::CvImage(h, "rgb8", rgb).toImageMsg();
//               image_pub->publish(*img_msg);
//             });

//         RCLCPP_INFO(node->get_logger(), "Search completed at zone [x=%d, y=%d]",
//                     result.zone.x + result.zone.width / 2,
//                     result.zone.y + result.zone.height / 2);
//       });

//   RCLCPP_INFO(node->get_logger(), "LandingZoneNode ready. Awaiting 'find' message.");
//   // cv::destroyWindow("Keypoints Debug");
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// };


//   // === [INÍCIO DO TRECHO DE DEBUG DE KEYPOINTS] ===

//   // Parâmetros do Shi–Tomasi (ajuste se quiser)
//   // int maxCorners = 20;       // no máximo 20 pontos
//   // double qualityLevel = 0.2; // só pegam cantos com pelo menos 20% do melhor canto
//   // double minDistance = 20.0;
//   // int blockSize = 15;

//   // // Cria a janela de debug
//   // cv::namedWindow("Keypoints Debug", cv::WINDOW_NORMAL);
//   // cv::resizeWindow("Keypoints Debug", cfg.width, cfg.height);

//   // // Subscreve direto em camera/image_raw
//   // auto kp_sub = node->create_subscription<sensor_msgs::msg::Image>(
//   //     cfg.cameraTopic,
//   //     rclcpp::SensorDataQoS(),
//   //     [=](sensor_msgs::msg::Image::ConstSharedPtr msg)
//   //     {
//   //       // 1) Converter pra Mat BGR
//   //       cv::Mat bgr = cv_bridge::toCvShare(msg, "bgr8")->image;
//   //       // 2) Para cinza e detectar cantos
//   //       cv::Mat gray;
//   //       cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
//   //       std::vector<cv::Point2f> corners;
//   //       cv::goodFeaturesToTrack(
//   //           gray, corners,
//   //           maxCorners, qualityLevel, minDistance,
//   //           cv::Mat(), blockSize);
//   //       // 3) Desenha cada keypoint em azul
//   //       for (auto &pt : corners)
//   //       {
//   //         cv::circle(bgr, pt, 4, cv::Scalar(255, 0, 0), 2);
//   //       }
//   //       // 4) Mostra a janela
//   //       cv::imshow("Keypoints Debug", bgr);
//   //       cv::waitKey(1);
//   //     });