#ifndef board_controller_H
#define board_controller_H

//////////* Headers *//////////

// Boost
#include <boost/asio/serial_port.hpp>
// ROS2
#include <rclcpp/node.hpp>
#include <std_srvs/srv/trigger.hpp>
// Board BoardController Message
#include "board_msg/msg/packet.hpp"

///////////////////////////////

//////////* Classes *//////////

class BoardController : public rclcpp::Node {
public:
    BoardController();
    ~BoardController();

public:
    // Connection
    auto connect(const std::string& port) -> bool;
    auto close() -> void;

    // Transmit / Receive
    auto transmit(const std::vector<uint8_t>& vector_data) -> void;
    auto receive() -> void;
    auto receive_packet(const board_msg::msg::Packet& msg) -> void;

    // Callback
    auto initialization_complete([[maybe_unused]]const std::shared_ptr<std_srvs::srv::Trigger::Request> request, const std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> void;

    // Initialization
    auto initialize() -> void;

    // Thread
    auto listen() -> void;

private:
    // Subscription
    rclcpp::Subscription<board_msg::msg::Packet>::SharedPtr sub_bus_servo;
    rclcpp::Subscription<board_msg::msg::Packet>::SharedPtr sub_buzzer;
    rclcpp::Subscription<board_msg::msg::Packet>::SharedPtr sub_led;


    // Service
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_ini;

    // Serial port
    boost::asio::io_context io;

    boost::asio::serial_port serial;

    // Thread
    std::thread thread_receive;
};

///////////////////////////////

#endif