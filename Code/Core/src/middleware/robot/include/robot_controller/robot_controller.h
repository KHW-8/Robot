#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

//////////* Headers *//////////

/* Core */
//// Application
#include "gui_msgs/msg/command.hpp"
//// Functional
#include "kinematics_msgs/srv/pose.hpp"
//// Hardware
#include "peripheral_msgs/msg/bus_servo.hpp"
#include "peripheral_msgs/msg/buzzer_request.hpp"
#include "peripheral_msgs/msg/led_request.hpp"
#include "board_msgs/msg/packet.hpp"

/* ROS2 */
#include <rclcpp/callback_group.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include "peripheral_msgs/srv/bus_servo_request.hpp"
#include "std_srvs/srv/trigger.hpp"

///////////////////////////////

//////////* Classes *//////////

class RobotController : public rclcpp::Node {
public:
    RobotController();

public:
    //
    auto read_servo_pos(const std::vector<peripheral_msgs::msg::BusServo::_id_type>& ids) -> void;
    auto set_servo_pos(const peripheral_msgs::msg::BusServo::_id_type& id, 
                       const peripheral_msgs::msg::BusServo::_angle_type& angle, 
                       const peripheral_msgs::msg::BusServo::_duration_type& duration) -> void;
    auto set_servo_pos(std::vector<peripheral_msgs::msg::BusServo> servos_pos) -> void;

    auto set_buzzer(peripheral_msgs::msg::BuzzerRequest::_frequency_type freq, 
                peripheral_msgs::msg::BuzzerRequest::_on_duration_type on_duration, 
                peripheral_msgs::msg::BuzzerRequest::_off_duration_type off_duration, 
                peripheral_msgs::msg::BuzzerRequest::_repeat_count_type repeat_count) -> void;
    auto set_led(peripheral_msgs::msg::LED::_id_type led_id, 
                peripheral_msgs::msg::LED::_on_duration_type on_duration, 
                peripheral_msgs::msg::LED::_off_duration_type off_duration, 
                peripheral_msgs::msg::LED::_repeat_count_type repeat_count) -> void;
        
    auto test() -> void;

private:
    // Callback
    auto receive_control_panel_msg(const gui_msgs::msg::Command& msg) -> void;

    auto initialization_complete([[maybe_unused]]const std::shared_ptr<std_srvs::srv::Trigger::Request> request, const std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> void;

    /* Initialization */
    auto initialize() -> void;
    // Hardware
    auto initialize_board_controller() -> void;
    auto initialize_buzzer() -> void;
    auto initialize_led() -> void;
    auto initialize_servos() -> void;
    // Functional
    auto initialize_kinematics_calculator() -> void;
    auto initialize_vision() -> void;
    // Application
    auto initialize_control_panel() -> void;

private:
    // Callback group
    rclcpp::CallbackGroup::SharedPtr cli_servo_cb_group;

    // Client
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_board_controller_ready;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_buzzer_ready;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_led_ready;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_servo_ready;
    rclcpp::Client<peripheral_msgs::srv::BusServoRequest>::SharedPtr cli_servo;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_kinematics_calculator_ready;
    rclcpp::Client<kinematics_msgs::srv::Pose>::SharedPtr cli_kinematics_calculator;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_vision_ready;

    // Publisher
    rclcpp::Publisher<board_msgs::msg::Packet>::SharedPtr pub_packet;
    rclcpp::Publisher<peripheral_msgs::msg::LEDRequest>::SharedPtr pub_led;
    rclcpp::Publisher<peripheral_msgs::msg::BuzzerRequest>::SharedPtr pub_buzzer;
    
    // Service
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_ini;

    // Subscription
    rclcpp::Subscription<gui_msgs::msg::Command>::SharedPtr sub_gui;
};


///////////////////////////////

#endif