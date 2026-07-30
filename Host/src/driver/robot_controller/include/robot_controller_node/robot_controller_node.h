#ifndef ROBOT_CONTROLLER_NODE_H
#define ROBOT_CONTROLLER_NODE_H

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
// Host
#include "robot_controller_msg/msg/buzzer.hpp"
#include "robot_controller_msg/msg/le_ds.hpp"
#include "robot_controller_msg/msg/servos.hpp"

class RobotController : public rclcpp::Node {
public:
    RobotController();
private:
    // Callback
    auto set_bus_servo(robot_controller_msg::msg::Servos::UniquePtr msg) -> void;
    auto set_buzzer(robot_controller_msg::msg::Buzzer msg) -> void;
    auto set_led(robot_controller_msg::msg::LEDs msg) -> void;
    auto initialization_complete([[maybe_unused]]const std::shared_ptr<std_srvs::srv::Trigger::Request> request, const std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> void;

    // Initialization
    auto initialize() -> void;

private:
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr board_controller_client;

    rclcpp::Subscription<robot_controller_msg::msg::Buzzer>::SharedPtr buzzer_sub;
    rclcpp::Subscription<robot_controller_msg::msg::LEDs>::SharedPtr led_sub;
    rclcpp::Subscription<robot_controller_msg::msg::Servos>::SharedPtr bus_servo_sub;
    
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ini_srv;
};


#endif