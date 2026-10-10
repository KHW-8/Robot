#ifndef CAMERA_H
#define CAMERA_H

//////////* Headers *//////////

/* Deptrum */
#include <deptrum/device.h>
#include <deptrum/stream.h>

/* OpenCV */
#include <opencv2/core.hpp>

/* ROS */
#include <rclcpp/node.hpp>

///////////////////////////////

//////////* Classes *//////////

class Camera : public rclcpp::Node {
public:
    Camera();

public:
    //
    void start();
    void stop();
private:
    // Initialization
    auto initialize() -> void;

    // Frame
    static auto depth_frame(const std::shared_ptr<deptrum::stream::StreamFrame>& frame) -> cv::Mat;
    static auto ir_frame(const std::shared_ptr<deptrum::stream::StreamFrame>& frame) -> cv::Mat;
    static auto rgb_frame(const std::shared_ptr<deptrum::stream::StreamFrame>& frame) -> cv::Mat;

private:
    std::shared_ptr<deptrum::stream::Device> device = nullptr;
    deptrum::stream::Stream *stream = nullptr;
};

///////////////////////////////

#endif