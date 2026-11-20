#ifndef board_controller_H
#define board_controller_H

//////////* Headers *//////////

// Boost
#include <boost/asio/serial_port.hpp>
// ROS2
#include <cstdint>
#include <rclcpp/node.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <vector>
// Board BoardController Message
#include "board_msgs/msg/packet.hpp"

///////////////////////////////

///////////* Macros *///////////

#define PACKET_HEADER 0x55

////////////////////////////////

//////////* Classes *//////////

class BoardController : public rclcpp::Node {
public:
    BoardController();
    ~BoardController();

private:
    // Connection
    auto connect(const std::string& port) -> bool;
    auto close() -> void;
    
    // Callback
    auto initialization_complete([[maybe_unused]]const std::shared_ptr<std_srvs::srv::Trigger::Request> request, const std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> void;

    // Check
    auto check_packet(const std::vector<uint8_t>& packet) -> std::pair<bool, bool>; // <isReceiving, haveError>

    // Checksum
    auto generate_checksum(const std::vector<uint8_t>& vector) -> uint8_t;

    // Initialization
    auto initialize() -> void;

    // Handle
    auto handle_packet(const std::vector<uint8_t>& packet) -> void;

    // Transmit / Receive
    auto transmit(const std::vector<uint8_t>& vector_data) -> void;
    auto receive() -> void;
    auto receive_packet(const board_msgs::msg::Packet& msg) -> void;
    auto receive_packet_from_board() -> void;
    auto receive_msg_from_board() -> void;

private:
    // Subscription
    rclcpp::Subscription<board_msgs::msg::Packet>::SharedPtr sub_bus_servo;
    rclcpp::Subscription<board_msgs::msg::Packet>::SharedPtr sub_buzzer;
    rclcpp::Subscription<board_msgs::msg::Packet>::SharedPtr sub_led;


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