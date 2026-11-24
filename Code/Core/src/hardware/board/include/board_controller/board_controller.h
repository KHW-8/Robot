#ifndef board_controller_H
#define board_controller_H

//////////* Headers *//////////

/* Boost */
#include <boost/asio/serial_port.hpp>

/* Core */
#include "board_msgs/msg/packet.hpp"
#include "board_msgs/srv/packet.hpp"

/* ROS2 */
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <std_srvs/srv/trigger.hpp>

/* STD */
#include <cstdint>
#include <memory>
#include <vector>

///////////////////////////////

///////////* Macros *///////////

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

    // Generate
    auto generate_request_id(const std::vector<uint8_t>& vector) -> uint16_t;
    auto generate_checksum(const std::vector<uint8_t>& vector) -> uint8_t;

    // Initialization
    auto initialize() -> void;

    // Handle
    auto handle_request(const std::shared_ptr<board_msgs::srv::Packet::Request> request, const std::shared_ptr<board_msgs::srv::Packet::Response> response) -> void;
    auto handle_packet(const board_msgs::msg::Packet& msg) -> void;

    // Transmit / Receive
    auto transmit(const std::vector<uint8_t>& data) -> void;
    auto receive_board_packet() -> void;
    auto receive_msg() -> void;

    // Store
    auto store_board_packet(const std::vector<uint8_t>& packet) -> void;

private:
    // Request queue
    std::map<uint16_t, std::vector<uint8_t> > packet_map; // <Request ID, Packet>

    // Subscription
    rclcpp::Subscription<board_msgs::msg::Packet>::SharedPtr sub_buzzer;
    rclcpp::Subscription<board_msgs::msg::Packet>::SharedPtr sub_led;

    // Service
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_ini;
    rclcpp::Service<board_msgs::srv::Packet>::SharedPtr srv_packet;

    // Serial port
    boost::asio::io_context io;

    boost::asio::serial_port serial;

    // Thread
    std::thread thread_receive;
};

///////////////////////////////

#endif