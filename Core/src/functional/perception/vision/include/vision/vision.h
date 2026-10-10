#ifndef VISION_H
#define VISION_H

//////////* Headers *//////////

/* ROS */
#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>

///////////////////////////////

//////////* Classes *//////////

class Vision : public rclcpp::Node {
public:
    Vision();

private:
    // Callback
    void receive_bgr_frame(const sensor_msgs::msg::Image& msg);
    void receive_depth_frame(const sensor_msgs::msg::Image& msg);
    void receive_infrared_frame(const sensor_msgs::msg::Image& msg);

    // Initialization
    auto initialize() -> void;

private:
    // Client
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_camera;

    // Subscription
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_bgr_frame;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_frame;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_infrared_frame;
};

///////////////////////////////

#endif