#ifndef BOARD_CONTROLLER_H
#define BOARD_CONTROLLER_H

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
// Serial
#include "serial.h"
// Host
#include "robot_controller_msg/msg/buzzer.hpp"
#include "robot_controller_msg/msg/led.hpp"
#include "robot_controller_msg/msg/servo.hpp"
#include "robot_controller_msg/msg/servos.hpp"

class BoardController : public rclcpp::Node {
public:
    BoardController() 
        :Node("board_controller")
    {
        // Waiting for robot controller node to start
        this->client = this->create_client<std_srvs::srv::Trigger>("/robot_controller/initialization_complete");
        this->client->wait_for_service();
    }

public:
    void list_ports();
    void transmit();

private:
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;
    rclcpp::Subscription<robot_controller_msg::msg::Buzzer>::SharedPtr buzzer_sub;
    rclcpp::Subscription<robot_controller_msg::msg::LED>::SharedPtr led_sub;
    rclcpp::Subscription<robot_controller_msg::msg::Servos>::SharedPtr servo_sub;
};

void BoardController::list_ports() {
    auto devices = serial::list_ports();
    for (const auto& device : devices) {
        RCLCPP_INFO(
            rclcpp::get_logger(""), 
            "Port: %s, Description: %s, Hardware ID: %s", 
            device.port.c_str(),
            device.description.c_str(),
            device.hardware_id.c_str()
        );
    }
}

void BoardController::transmit() {
    serial::Serial serial = serial::Serial("", 115200, serial::Timeout::simpleTimeout(1000));

    try {
        serial.setPort("COM4");
        serial.open();
    } catch (const serial::IOException& e) {
        RCLCPP_ERROR(rclcpp::get_logger(""), "%s", e.what());
        return;
    }

    if (!serial.isOpen())
        std::cout << "Failed to open." << std::endl;
}

#endif