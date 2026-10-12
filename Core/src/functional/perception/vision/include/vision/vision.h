#ifndef VISION_H
#define VISION_H

//////////* Headers *//////////

/* ROS */
#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>

/* YAML-CPP*/
#include <string>
#include <yaml-cpp/node/node.h>

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
    auto receive_bgr_frame(const sensor_msgs::msg::Image& msg) -> void;
    auto receive_depth_frame(const sensor_msgs::msg::Image& msg) -> void;
    auto receive_infrared_frame(const sensor_msgs::msg::Image& msg) -> void;

    // Initialization
    auto initialize() -> void;

    // Load
    auto load_config() -> bool;

    // Recognition
    auto recognize_color(const sensor_msgs::msg::Image& msg, Color color) -> void;

private:
    // Config
    YAML::Node config;

    // Client
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_camera;

    // Subscription
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_bgr_frame;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_frame;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_infrared_frame;
};

///////////////////////////////

#endif