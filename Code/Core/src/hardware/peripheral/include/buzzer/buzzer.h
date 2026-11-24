#ifndef BUZZER_H
#define BUZZER_H

//////////* Headers *//////////

/* Core */
#include "board_msgs/srv/packet.hpp"
#include "peripheral_msgs/msg/buzzer_request.hpp"
#include "peripheral_msgs/srv/buzzer_request.hpp"

/* ROS2 */
#include <rclcpp/callback_group.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <vector>

///////////////////////////////

//////////* Classes *//////////

class Buzzer : public rclcpp::Node {
public: 
    Buzzer();

public:
    auto set(peripheral_msgs::msg::BuzzerRequest::_frequency_type freq, 
             peripheral_msgs::msg::BuzzerRequest::_on_duration_type on_duration, 
             peripheral_msgs::msg::BuzzerRequest::_off_duration_type off_duration, 
             peripheral_msgs::msg::BuzzerRequest::_repeat_count_type repeat_count) -> void;
    auto set(const std::vector<peripheral_msgs::msg::Buzzer>& buzzers) -> void;

private:
    // Callback
    auto initialization_complete([[maybe_unused]]const std::shared_ptr<std_srvs::srv::Trigger::Request> request, const std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> void;

    auto handle_request(const peripheral_msgs::srv::BuzzerRequest::Request::SharedPtr request, const peripheral_msgs::srv::BuzzerRequest::Response::SharedPtr response) -> void;


    // Initialization
    auto initialize() -> void;

private:
    // Callback group
    rclcpp::CallbackGroup::SharedPtr cli_board_controller_cb_group;

    // Client
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_board_controller_ready;
    rclcpp::Client<board_msgs::srv::Packet>::SharedPtr cli_board_controller;

    // Service
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_ini;
    rclcpp::Service<peripheral_msgs::srv::BuzzerRequest>::SharedPtr srv_request;
};

///////////////////////////////

#endif