#ifndef ROBOT_CONTROLLER_NODE_H
#define ROBOT_CONTROLLER_NODE_H

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
// Robot Controller Node
#include "robot_controller_msg/msg/buzzer.hpp"
#include "robot_controller_msg/msg/le_ds.hpp"
#include "robot_controller_msg/msg/servos.hpp"

#include "board_controller_msg/msg/packet.hpp"

class RobotController : public rclcpp::Node {
public:
    RobotController();
private:
    // Callback
    auto set_bus_servo(const robot_controller_msg::msg::Servos::UniquePtr& msg) -> void;
    auto set_buzzer(const robot_controller_msg::msg::Buzzer& msg) -> void;
    auto set_led(const robot_controller_msg::msg::LEDs& msg) -> void;
    auto initialization_complete([[maybe_unused]]const std::shared_ptr<std_srvs::srv::Trigger::Request> request, const std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> void;

    // Initialization
    auto initialize() -> void;

private:
    // Client
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_board_controller;

    // Publisher
    rclcpp::Publisher<board_controller_msg::msg::Packet>::SharedPtr pub_packet;

    // Subscription
    rclcpp::Subscription<robot_controller_msg::msg::Buzzer>::SharedPtr sub_buzzer;
    rclcpp::Subscription<robot_controller_msg::msg::LEDs>::SharedPtr sub_led;
    rclcpp::Subscription<robot_controller_msg::msg::Servos>::SharedPtr sub_bus_servo;
    
    // Service
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_ini;
};


#endif