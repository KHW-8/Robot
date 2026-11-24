#ifndef LED_H
#define LED_H

//////////* Headers *//////////

/* Core */
#include "board_msgs/srv/packet.hpp"
#include "peripheral_msgs/msg/led.hpp"
#include "peripheral_msgs/srv/led_request.hpp"
/* ROS2 */
#include <rclcpp/callback_group.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <std_srvs/srv/trigger.hpp>

/* STD */
#include <vector>

///////////////////////////////

//////////* Classes *//////////

class LED : public rclcpp::Node {
public: 
    LED();

public:
    auto set(peripheral_msgs::msg::LED::_id_type led_id, 
             peripheral_msgs::msg::LED::_on_duration_type on_duration, 
             peripheral_msgs::msg::LED::_off_duration_type off_duration, 
             peripheral_msgs::msg::LED::_repeat_count_type repeat_count) -> void;
    auto set(const std::vector<peripheral_msgs::msg::LED>& leds) -> void;

private:
    auto initialize() -> void;
    
    // Callback
    auto initialization_complete([[maybe_unused]]const std::shared_ptr<std_srvs::srv::Trigger::Request> request, const std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> void;

    auto handle_request(const peripheral_msgs::srv::LEDRequest::Request::SharedPtr request, const peripheral_msgs::srv::LEDRequest::Response::SharedPtr response) -> void;
    
private:
    // Callback Group
    rclcpp::CallbackGroup::SharedPtr cli_board_controller_cb_group;

    // Client
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_board_controller_ready;
    rclcpp::Client<board_msgs::srv::Packet>::SharedPtr cli_board_controller;

    
    // Service
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_ini;
    rclcpp::Service<peripheral_msgs::srv::LEDRequest>::SharedPtr srv_request;
};

///////////////////////////////

#endif