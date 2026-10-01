#ifndef LED_H
#define LED_H

//////////* Headers *//////////

/* Core */
#include "board_msg/msg/packet.hpp"
#include "peripheral_msg/msg/led_request.hpp"
/* ROS2 */
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <std_srvs/srv/trigger.hpp>

///////////////////////////////

class LED : public rclcpp::Node {
public: 
    LED();

public:
    // Callback
    auto set(const peripheral_msg::msg::LEDRequest& request) -> void;

    
    private:
    auto initialize() -> void;
    
    // Callback
    auto initialization_complete([[maybe_unused]]const std::shared_ptr<std_srvs::srv::Trigger::Request> request, const std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> void;
    
private:
    // Client
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_board_controller;

    // Publisher
    rclcpp::Publisher<board_msg::msg::Packet>::SharedPtr pub_board_controller;

    // Subscription
    rclcpp::Subscription<peripheral_msg::msg::LEDRequest>::SharedPtr sub_robot_controller;
    
    // Service
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_ini;
};

#endif