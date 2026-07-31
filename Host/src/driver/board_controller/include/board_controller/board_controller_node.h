#ifndef BOARD_CONTROLLER_H
#define BOARD_CONTROLLER_H

// Serial
#include "serial/serial.h"
// ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

class BoardController : public rclcpp::Node {
public:
    BoardController();
    ~BoardController();

public:
    static auto list_ports() -> void;

    // Connection
    auto connect(const std::string& port) -> void;
    auto close() -> void;

    auto transmit() -> void;

    // Callback
    auto initialization_complete([[maybe_unused]]const std::shared_ptr<std_srvs::srv::Trigger::Request> request, const std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> void;

private:
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ini_srv;

    serial::Serial serial;
};

#endif