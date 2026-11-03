#ifndef CONTROL_PANEL
#define CONTROL_PANEL


//////////* Headers *//////////

/* Core */
#include "gui_msgs/msg/command.hpp"

/* ROS2 */
#include <rclcpp/node.hpp>

/* STD */
#include <thread>

///////////////////////////////

class ControlPanel : public rclcpp::Node {
public:
    ControlPanel();
    ~ControlPanel();

public:
    auto receive_cmd() -> void;

private:
    auto initialize() -> void;

private:
    // Publisher
    rclcpp::Publisher<gui_msgs::msg::Command>::SharedPtr pub_robot_controller;

    std::thread thread_input;
};

#endif