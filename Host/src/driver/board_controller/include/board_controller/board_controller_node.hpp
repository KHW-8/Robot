#ifndef BOARD_CONTROLLER_H
#define BOARD_CONTROLLER_H

// STD
// ROS2
#include "rclcpp/rclcpp.hpp"
// Serial
#include "serial.h"

class BoardController : public rclcpp::Node {
public:
    BoardController() 
        :Node("board_controller")
    {
    }

public:
    void list_ports();
    void transmit();
};

void BoardController::list_ports() {
    auto devices = serial::list_ports();
    for (const auto& device : devices) {
        std::string str;
        str.append( "Port: " + device.port + " ");
        str.append( "Description: " + device.description + " ");
        str.append( "Hardware ID: " + device.hardware_id + " ");

        RCLCPP_INFO(rclcpp::get_logger(""), "%s", str.c_str());
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