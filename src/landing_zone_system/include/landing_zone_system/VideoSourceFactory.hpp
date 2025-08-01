// VideoSourceFactory.hpp
#ifndef VIDEOSOURCEFACTORY_HPP
#define VIDEOSOURCEFACTORY_HPP

// #include "OpenCvCapture.hpp"
#include "RosCapture.hpp"
#include "SearchConfig.hpp"
#include <memory>
#include <stdexcept>


#include "ICapture.hpp"

class VideoSourceFactory
{
public:
    static std::unique_ptr<ICapture> create(const SearchConfig &cfg)
    {
        if (cfg.origem == "ros") {
            RCLCPP_INFO(rclcpp::get_logger("VideoSourceFactory"),
                        "Criando RosCapture para topo '%s'", cfg.cameraTopic.c_str());
            return std::make_unique<RosCapture>(cfg.cameraTopic);
        }

        throw std::runtime_error("Origem não suportada: " + cfg.origem);
    }
};


#endif // VIDEOSOURCEFACTORY_HPP


// VideoSourceFactory.hpp
// #ifndef VIDEOSOURCEFACTORY_HPP
// #define VIDEOSOURCEFACTORY_HPP

// #include "IVideoCapture.hpp"
// // #include "OpenCvCapture.hpp"
// #include "RosCapture.hpp"
// #include "SearchConfig.hpp"
// #include <memory>
// #include <stdexcept>


// class VideoSourceFactory
// {
// public:
//     // static std::unique_ptr<IVideoCapture>
//     IVideoCapture* create(const SearchConfig &cfg)
//     {
//         // if (cfg.origem == "video" || cfg.origem == "camera")
//         // {
//         //     std::string arg = (cfg.origem == "camera" ? "0" : cfg.caminhoVideo);
//         //     return std::make_unique<OpenCvCapture>(arg);
//         // }
//         if (cfg.origem == "ros")
//         {
//             RCLCPP_INFO(rclcpp::get_logger("VideoSourceFactory"),
//                         "Criando RosCapture para topo '%s'", cfg.cameraTopic.c_str());
//             return std::make_unique<RosCapture>(cfg.cameraTopic);
//         }

//         throw std::runtime_error("Origem não suportada: " + cfg.origem);
//     }
// };

// #endif // VIDEOSOURCEFACTORY_HPP

