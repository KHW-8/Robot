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
#include "peripheral_msgs/msg/bus_servo_request.hpp"
#include "board_msgs/msg/packet.hpp"

/* ROS2 */
#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include "std_srvs/srv/trigger.hpp"

///////////////////////////////

//////////* Classes *//////////

class RobotController : public rclcpp::Node {
public:
    RobotController();

public:
    //
    void read_servo_pos(const std::vector<decltype(peripheral_msgs::msg::BusServo::id)>& ids);
    auto set_servo_pos(const decltype(peripheral_msgs::msg::BusServo::id)& id, 
                       const decltype(peripheral_msgs::msg::BusServo::angle)& angle, 
                       const decltype(peripheral_msgs::msg::BusServo::duration)& duration) -> void;
    auto set_servo_pos(std::vector<peripheral_msgs::msg::BusServo> servos_pos) -> void;

    auto set_buzzer(decltype(peripheral_msgs::msg::BuzzerRequest::frequency) freq, 
                decltype(peripheral_msgs::msg::BuzzerRequest::on_duration) on_duration, 
                decltype(peripheral_msgs::msg::BuzzerRequest::off_duration) off_duration, 
                decltype(peripheral_msgs::msg::BuzzerRequest::repeat_count) repeat_count) -> void;
    auto set_led(decltype(peripheral_msgs::msg::LED::id) led_id, 
                decltype(peripheral_msgs::msg::LED::on_duration) on_duration, 
                decltype(peripheral_msgs::msg::LED::off_duration) off_duration, 
                decltype(peripheral_msgs::msg::LED::repeat_count) repeat_count) -> void;
        
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
    // Client
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_board_controller_ready;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_buzzer_ready;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_led_ready;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_sevo_ready;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_kinematics_calculator_ready;
    rclcpp::Client<kinematics_msgs::srv::Pose>::SharedPtr cli_kinematics_calculator;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_vision_ready;

    // Publisher
    rclcpp::Publisher<board_msgs::msg::Packet>::SharedPtr pub_packet;
    rclcpp::Publisher<peripheral_msgs::msg::LEDRequest>::SharedPtr pub_led;
    rclcpp::Publisher<peripheral_msgs::msg::BusServoRequest>::SharedPtr pub_servo;
    rclcpp::Publisher<peripheral_msgs::msg::BuzzerRequest>::SharedPtr pub_buzzer;
    
    // Service
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_ini;

    // Subscription
    rclcpp::Subscription<gui_msgs::msg::Command>::SharedPtr sub_gui;
};


///////////////////////////////

#endif