#ifndef VISION_H
#define VISION_H

//////////* Headers *//////////

/* Core */
#include "kinematics_msgs/msg/transformation.hpp"

/* Eigen */
#include <eigen3/Eigen/Dense>

/* ROS */
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_srvs/srv/trigger.hpp"

/* OpenCV */
#include <opencv2/core.hpp>

/* YAML-CPP*/
#include <yaml-cpp/node/node.h>

/* STD */
#include <string>

///////////////////////////////

//////////* Enumerates *//////////


//////////////////////////////////

//////////* Classes *//////////
class Color {
public:
    enum Type {
        NONE,
        BLUE,
        BLACK,
        RED,
    };

public:
    Color(const Type& type) { this->type = type; }
    Color(const std::string& type);

public:
    static auto to_type(const std::string& type) -> Type;

    auto to_string() -> std::string;

private:
    Type type;
};

class Vision : public rclcpp::Node {
public:
    Vision();

private:
    // Callback
    auto initialization_complete([[maybe_unused]]const std::shared_ptr<std_srvs::srv::Trigger::Request> request, const std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> void;

    auto receive_bgr_frame(const sensor_msgs::msg::Image& msg) -> void;
    auto receive_depth_frame(const sensor_msgs::msg::Image& msg) -> void;
    auto receive_infrared_frame(const sensor_msgs::msg::Image& msg) -> void;

    auto receive_camera_info_rgb(const sensor_msgs::msg::CameraInfo& msg) -> void;

    // Convert
    auto pixels_to_world_coordinate(const Eigen::Vector2d& pos) -> Eigen::Vector3d;

    // Initialization
    auto initialize() -> void;

    // Load
    auto load_config(const std::string& path) -> YAML::Node;

    // Parse
    auto parse_config() -> void;

    // Recognition
    auto recognize_color(const sensor_msgs::msg::Image& msg, Color color) -> void;
    auto find_center_point(const sensor_msgs::msg::Image& msg) -> void;

private:
    // Config
    YAML::Node color;
    YAML::Node extrinsic;

    Eigen::Matrix3d K;
    Eigen::Matrix4d T;

    // Client
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_camera;

    // Publisher
    rclcpp::Publisher<kinematics_msgs::msg::Transformation>::SharedPtr pub_kinematics_calculator;

    // Subscription
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_bgr_frame;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_frame;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_infrared_frame;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_camera_info_rgb;

    // Service
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_init;
};

///////////////////////////////

#endif