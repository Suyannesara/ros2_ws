// #include "landing_zone_system/LandingZoneSystem.hpp"
// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/empty.hpp"
// #include "geometry_msgs/msg/point.hpp"
// #include "sensor_msgs/msg/image.hpp"
// #include <cv_bridge/cv_bridge.h>
// #include <opencv2/imgproc.hpp>

// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = rclcpp::Node::make_shared("landing_zone_node");

//   // parâmetros
//   SearchConfig cfg;
//   node->declare_parameter<std::string>("origem", "ros");
//   node->declare_parameter<std::string>("cameraTopic", "/camera/image_raw");
//   node->declare_parameter<int>("tempoBuscaSegundos", 10);
//   node->declare_parameter<int>("width", 640);
//   node->declare_parameter<int>("height", 480);
//   node->declare_parameter<double>("fps", 30.0);
//   node->declare_parameter<int>("gridRows", 6);
//   node->declare_parameter<int>("gridCols", 6);

//   node->get_parameter("origem",            cfg.origem);
//   node->get_parameter("cameraTopic",       cfg.cameraTopic);
//   node->get_parameter("tempoBuscaSegundos",cfg.tempoBuscaSegundos);
//   node->get_parameter("width",             cfg.width);
//   node->get_parameter("height",            cfg.height);
//   node->get_parameter("fps",               cfg.fps);
//   node->get_parameter("gridRows",          cfg.gridRows);
//   node->get_parameter("gridCols",          cfg.gridCols);

//   // publishers
//   auto zone_pub  = node->create_publisher<geometry_msgs::msg::Point>("zoneTopic",      10);
//   auto image_pub = node->create_publisher<sensor_msgs::msg::Image>("annotated_image", 10);

//   LandingZoneSystem lzs;

//   auto sub = node->create_subscription<std_msgs::msg::Empty>(
//       "find", 10,
//       [node, &lzs, &cfg, zone_pub, image_pub](std_msgs::msg::Empty::SharedPtr)
//       {
//         RCLCPP_INFO(node->get_logger(), "Find command received. Starting search...");

//         auto result = lzs.findLandingZone(
//           cfg,
//           [node, zone_pub, image_pub](cv::Mat &frame,
//                                       const LandingZoneCandidate &cand,
//                                       bool isCurrent)
//           {
//             // só publica o final
//             if (!isCurrent || frame.empty())
//               return;

//             // desenha em verde grosso
//             cv::rectangle(frame, cand.zone, cv::Scalar(0,255,0), 3);

//             // publica coords
//             geometry_msgs::msg::Point p;
//             p.x = cand.zone.x + cand.zone.width  / 2;
//             p.y = cand.zone.y + cand.zone.height / 2;
//             zone_pub->publish(p);

//             // publica imagem anotada
//             std_msgs::msg::Header h; h.stamp = node->now();
//             cv::Mat rgb; cv::cvtColor(frame, rgb, cv::COLOR_BGR2RGB);
//             auto msg = cv_bridge::CvImage(h,"rgb8",rgb).toImageMsg();
//             image_pub->publish(*msg);

//             RCLCPP_INFO(node->get_logger(),
//                         "Visualize(final) called; published annotated_image");
//           }
//         );

//         RCLCPP_INFO(node->get_logger(),
//                     "Search completed at zone [x=%d, y=%d]",
//                     result.zone.x + result.zone.width/2,
//                     result.zone.y + result.zone.height/2);
//       }
//   );

//   RCLCPP_INFO(node->get_logger(), "LandingZoneNode ready. Awaiting 'find' message.");
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }
