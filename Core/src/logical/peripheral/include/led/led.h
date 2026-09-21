#ifndef LED_H
#define LED_H

// ROS2
#include <rclcpp/node.hpp>
#include <std_srvs/srv/trigger.hpp>
// Robot Controller Message
#include "peripheral_msg/msg/led_request.hpp"

class LED : public rclcpp::Node {
public: 
    LED();

public:
    void set_state(decltype(peripheral_msg::msg::LED::id) led_id, 
                   decltype(peripheral_msg::msg::LED::on_duration) on_duration, 
                   decltype(peripheral_msg::msg::LED::off_duration) off_duration, 
                   decltype(peripheral_msg::msg::LED::repeat_count) repeat_count);

private:
    rclcpp::Publisher<peripheral_msg::msg::LEDRequest>::SharedPtr pub;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_robot_controller;
};

#endif