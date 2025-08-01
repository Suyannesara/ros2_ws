#ifndef ROS_CAPTURE_HPP
#define ROS_CAPTURE_HPP

#include "ICapture.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <condition_variable>
#include <memory>



class RosCapture : public ICapture
{
public:
    explicit RosCapture(const std::string &topic = "/camera/image_raw")
        : node_(std::make_shared<rclcpp::Node>("ros_capture_node")),
          got_frame_(false)
    {
        subscription_ = node_->create_subscription<sensor_msgs::msg::Image>(
            topic,
            rclcpp::SensorDataQoS(),
            std::bind(&RosCapture::imageCallback, this, std::placeholders::_1));

        // prepara executor numa thread
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(node_);
        exec_thread_ = std::thread([this]()
                                   { executor_->spin(); });
    }

    ~RosCapture() override
    {
        release();
    }

    bool read(cv::Mat &frame) override
    {
        std::unique_lock<std::mutex> lk(mtx_);
        if (!cv_.wait_for(lk, std::chrono::milliseconds(200),
                          [this]
                          { return got_frame_; }))
            return false;
        frame = latest_frame_.clone();
        got_frame_ = false;
        return true;
    }

    bool isOpened() const override
    {
        // true enquanto o nó ROS e o executor estiverem saudáveis
        return (node_ && rclcpp::ok());
    }

    void release() override
    {
        // cancela e finaliza o executor
        if (executor_)
        {
            executor_->cancel();
            if (exec_thread_.joinable())
                exec_thread_.join();
        }
        // opcional: reset pointers para liberar tudo
        subscription_.reset();
        executor_.reset();
        node_.reset();
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // RCLCPP_INFO(node_->get_logger(),
        //             "RosCapture: got image callback (h=%d w=%d)",
        //             msg->height, msg->width);
        auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        {
            std::lock_guard<std::mutex> lk(mtx_);
            latest_frame_ = cv_ptr->image;
            got_frame_ = true;
        }
        cv_.notify_one();
    }

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread exec_thread_;
    cv::Mat latest_frame_;
    std::mutex mtx_;
    std::condition_variable cv_;
    bool got_frame_;
};

#endif // ROS_CAPTURE_HPP
