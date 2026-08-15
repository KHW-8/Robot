#ifndef BOARD_CONTROLLER_H
#define BOARD_CONTROLLER_H

// Serial
#include "serial/serial.h"
// ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "board_controller_msg/msg/packet.hpp"

class BoardController : public rclcpp::Node {
public:
    BoardController();
    ~BoardController();

public:
    // Port
    static auto list_ports() -> void;

    // Connection
    auto connect(const std::string& port) -> void;
    auto close() -> void;

    // Transmit / Receive
    auto transmit(const std::vector<uint8_t>& vector_data) -> void;
    auto receive() -> void;
    auto receive_packet(const board_controller_msg::msg::Packet& msg) -> void;

    // Callback
    auto initialization_complete([[maybe_unused]]const std::shared_ptr<std_srvs::srv::Trigger::Request> request, const std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> void;

    // Initialization
    auto initialize() -> void;

    // Thread
    auto listen() -> void;

private:
    // Client
    rclcpp::Subscription<board_controller_msg::msg::Packet>::SharedPtr sub_packet;

    // Service
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_ini;

    // Serial
    serial::Serial serial;

    // Thread
    std::thread thread_receive;
};

#endif