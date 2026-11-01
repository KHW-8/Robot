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
#include <rclcpp/publisher.hpp>
#include <rclcpp/service.hpp>
#include "deptrum/common_types.h"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <std_srvs/srv/trigger.hpp>

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
    // Callback
    auto initializetion_complete([[maybe_unused]]const std::shared_ptr<std_srvs::srv::Trigger::Request> request, const std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> void;

    // Conversion
    auto instric_param_to_camera_info(const deptrum::Intrinsic& param) -> sensor_msgs::msg::CameraInfo;

    // Initialization
    auto initialize() -> void;
    auto initialize_camera() -> void;

    // Publish
    auto publish_images() -> void;
    auto publish_camera_info() -> void;

private:
    // Depth Camera
    std::shared_ptr<deptrum::stream::Device> device = nullptr;
    deptrum::stream::Stream *stream = nullptr;

    // Publisher
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_bgr_image;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth_image;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_infrared_image;

    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_camera_info_infrared;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_camera_info_rgb;

    // Service
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_ini;
};

///////////////////////////////

#endif