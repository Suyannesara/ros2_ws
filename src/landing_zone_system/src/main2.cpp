// #include "landing_zone_system/LandingZoneSystem.hpp"
// #include "landing_zone_system/LandingZoneTracker.hpp"
// #include "rclcpp/rclcpp.hpp"
// #include <iostream>

// int main(int argc, char** argv) {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<rclcpp::Node>("landing_zone_node");

//   // Declare parâmetros
//   node->declare_parameter<std::string>("origem", "ros");
//   node->declare_parameter<std::string>("cameraTopic", "/camera/image_raw");
//   node->declare_parameter<int>("tempoBuscaSegundos", 10);
//   node->declare_parameter<int>("width", 640);
//   node->declare_parameter<int>("height", 480);
//   node->declare_parameter<double>("fps", 30.0);
//   node->declare_parameter<int>("gridRows", 6);
//   node->declare_parameter<int>("gridCols", 6);
//   node->declare_parameter<bool>("salvarVideo", false);
//   node->declare_parameter<std::string>("saidaVideoPath", "");

//   // Preencher SearchConfig a partir dos parâmetros
//   SearchConfig cfg;
//   node->get_parameter("origem",            cfg.origem);
//   node->get_parameter("cameraTopic",          cfg.cameraTopic);
//   node->get_parameter("tempoBuscaSegundos",cfg.tempoBuscaSegundos);
//   node->get_parameter("width",             cfg.width);
//   node->get_parameter("height",            cfg.height);
//   node->get_parameter("fps",               cfg.fps);
//   node->get_parameter("gridRows",          cfg.gridRows);
//   node->get_parameter("gridCols",          cfg.gridCols);
//   node->get_parameter("salvarVideo",       cfg.salvarVideo);
//   node->get_parameter("saidaVideoPath",    cfg.saidaVideoPath);
//   // (não esqueça de get_parameter para caminhoVideo se usar "video")

//   LandingZoneSystem lzs;
//   auto result = lzs.findLandingZone(cfg);

//   // ... faça algo com result (log, salvar, etc.)

//   rclcpp::shutdown();
//   return 0;
// }
