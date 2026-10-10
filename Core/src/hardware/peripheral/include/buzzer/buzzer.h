// ROS2
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <std_srvs/srv/trigger.hpp>
// Core
#include "board_msg/msg/packet.hpp"
#include "peripheral_msg/msg/buzzer_request.hpp"

class Buzzer : public rclcpp::Node {
public: 
    Buzzer();

public:
    auto set(const peripheral_msg::msg::BuzzerRequest& request) -> void;

private:
    // Callback
    auto initialization_complete([[maybe_unused]]const std::shared_ptr<std_srvs::srv::Trigger::Request> request, const std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> void;

    // Initialization
    auto initialize() -> void;

private:
    // Client
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_board_controller;

    // Publisher
    rclcpp::Publisher<board_msg::msg::Packet>::SharedPtr pub_board_controller;

    // Subscription
    rclcpp::Subscription<peripheral_msg::msg::BuzzerRequest>::SharedPtr sub_robot_controller;

    // Service
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_ini;
};