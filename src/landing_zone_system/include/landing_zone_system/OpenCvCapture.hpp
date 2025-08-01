// // OpenCvCapture.hpp
// #ifndef OPENCVCAPTURE_HPP
// #define OPENCVCAPTURE_HPP

// #include "IVideoCapture.hpp"
// #include <opencv2/opencv.hpp>
// #include <string>

// class OpenCvCapture : public ::IVideoCapture {
// public:
//     explicit OpenCvCapture(const std::string& path_or_index) {
//         if (!path_or_index.empty() && std::isdigit(path_or_index[0]))
//             cap_.open(std::stoi(path_or_index));
//         else
//             cap_.open(path_or_index);
//     }

//     bool read(cv::Mat& frame) override {
//         cap_ >> frame;
//         return !frame.empty();
//     }

//     bool isOpened() const override {
//         return cap_.isOpened();
//     }

//     void release() override {
//         cap_.release();
//     }

// private:
//     cv::VideoCapture cap_;
// };

// #endif // OPENCVCAPTURE_HPP
