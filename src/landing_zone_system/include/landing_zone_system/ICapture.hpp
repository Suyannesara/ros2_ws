#ifndef ICAPTURE_HPP
#define ICAPTURE_HPP

#include <opencv2/core.hpp>

class ICapture {
public:
    virtual ~ICapture() = default;
    virtual bool read(cv::Mat &frame) = 0;
    virtual bool isOpened() const = 0;
    virtual void release() = 0;
};

#endif // ICAPTURE_HPP
