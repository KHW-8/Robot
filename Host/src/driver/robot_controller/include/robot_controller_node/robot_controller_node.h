#ifndef ROBOT_CONTROLLER_NODE_H
#define ROBOT_CONTROLLER_NODE_H

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
// Host
#include "robot_controller_msg/msg/buzzer.hpp"
#include "robot_controller_msg/msg/led.hpp"
#include "robot_controller_msg/msg/servo.hpp"
#include "robot_controller_msg/msg/servos.hpp"

class RobotController : public rclcpp::Node {
public:
    RobotController() 
        :Node("robot_controller")
    {
        // Bus servo node subscription
        this->bus_servo_sub = this->create_subscription<robot_controller_msg::msg::Servos>(
            "/robot_controller/bus_servo/set_position", 
            10, 
            std::bind(&RobotController::set_bus_servo_position, this, std::placeholders::_1)
        );

        // Complete initialization and notify all other nodes
        this->inition_service = this->create_service<std_srvs::srv::Trigger>(
            "~/initialization_complete", 
            std::bind(&RobotController::initialization_callback, this, std::placeholders::_1, std::placeholders::_2)
        );
    }
private:
    void initialization_callback([[maybe_unused]]const std::shared_ptr<std_srvs::srv::Trigger::Request> request, const std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void set_bus_servo_position(robot_controller_msg::msg::Servos::UniquePtr msg);

private:
    rclcpp::Subscription<robot_controller_msg::msg::Buzzer>::SharedPtr buzzer_sub;
    rclcpp::Subscription<robot_controller_msg::msg::LED>::SharedPtr led_sub;
    rclcpp::Subscription<robot_controller_msg::msg::Servos>::SharedPtr bus_servo_sub;
    
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr inition_service;
};


#endif