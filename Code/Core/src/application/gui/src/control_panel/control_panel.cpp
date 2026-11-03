#include "control_panel.h"
#include "gui_msgs/msg/command.hpp"


//////////* Headers *//////////

/* Core */

/* ROS2 */
#include <iostream>
#include <rclcpp/executors.hpp>
#include <rclcpp/utilities.hpp>

/* STD */
#include <memory>
#include <thread>

///////////////////////////////


ControlPanel::ControlPanel() 
    : Node("control_panel")
{
    initialize();
}

ControlPanel::~ControlPanel() {
    if (this->thread_input.joinable())
        this->thread_input.join();
}


void ControlPanel::receive_cmd() {
    int servo_id = 0;
    int angle = 0;
    
    gui_msgs::msg::Command msg;

    while (rclcpp::ok()) {
        RCLCPP_INFO(rclcpp::get_logger(""), "Please input servo's Id and angle:");

        if (std::cin >> servo_id >> angle) {
            msg.servo_id = servo_id;
            msg.angle = angle;

            this->pub_robot_controller->publish(msg);
        } else {
            std::cin.clear();

            RCLCPP_ERROR(rclcpp::get_logger(""), "Error!");
            continue;
        }
    }
}

auto ControlPanel::initialize() -> void {
    this->pub_robot_controller = this->create_publisher<gui_msgs::msg::Command>("~/command", 1);

    this->thread_input = std::thread(&ControlPanel::receive_cmd, this);
}


auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlPanel>());
    rclcpp::shutdown();

    return 0;
}