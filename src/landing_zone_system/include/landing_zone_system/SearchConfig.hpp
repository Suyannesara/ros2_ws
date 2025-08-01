#ifndef SEARCH_CONFIG_H
#define SEARCH_CONFIG_H

#include <string>

struct SearchConfig
{
    std::string origem = "video";  // "video", "camera", "simulador"
    std::string caminhoVideo = ""; // Usado se origem == "video"
    int tempoBuscaSegundos = 20;
    int width = 640;
    int height = 480;
    double fps = 30.0;
    bool salvarVideo = false;
    std::string saidaVideoPath = ""; // usado se salvarVideo == true
    int gridRows;
    int gridCols;

    // **novo campo** para o tópico da câmera ROS
    std::string cameraTopic = "/camera/image_raw";
};

#endif // SEARCH_CONFIG_H