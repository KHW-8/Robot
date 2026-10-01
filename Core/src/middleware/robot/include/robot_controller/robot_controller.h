#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

//////////* Headers *//////////


/* ROS2 */
#include <rclcpp/node.hpp>
#include <std_srvs/srv/trigger.hpp>
/* Message*/
// Peripheral Message
#include "peripheral_msg/msg/buzzer_request.hpp"
#include "peripheral_msg/msg/led_request.hpp"
#include "peripheral_msg/msg/bus_servo_request.hpp"
// Board Message
#include "board_msg/msg/packet.hpp"

///////////////////////////////

//////////* Classes *//////////

class RobotController : public rclcpp::Node {
public:
    RobotController();

public:
    //
    void read_bus_servo_pos(const std::vector<decltype(peripheral_msg::msg::BusServo::id)>& ids);

    auto set_bus_servo_pos(const decltype(peripheral_msg::msg::BusServo::id)& id, 
                const decltype(peripheral_msg::msg::BusServo::angle)& angle, 
                const decltype(peripheral_msg::msg::BusServo::duration)& duration) -> void;

    auto set_buzzer(decltype(peripheral_msg::msg::BuzzerRequest::frequency) freq, 
                decltype(peripheral_msg::msg::BuzzerRequest::on_duration) on_duration, 
                decltype(peripheral_msg::msg::BuzzerRequest::off_duration) off_duration, 
                decltype(peripheral_msg::msg::BuzzerRequest::repeat_count) repeat_count) -> void;
    auto set_led(decltype(peripheral_msg::msg::LED::id) led_id, 
                decltype(peripheral_msg::msg::LED::on_duration) on_duration, 
                decltype(peripheral_msg::msg::LED::off_duration) off_duration, 
                decltype(peripheral_msg::msg::LED::repeat_count) repeat_count) -> void;

private:
    // Callback
    auto initialization_complete([[maybe_unused]]const std::shared_ptr<std_srvs::srv::Trigger::Request> request, const std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> void;


    // Initialization
    auto initialize() -> void;

private:
    // Client
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_board_controller;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_bus_servo;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_buzzer;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_led;


    // Publisher
    rclcpp::Publisher<board_msg::msg::Packet>::SharedPtr pub_packet;
    rclcpp::Publisher<peripheral_msg::msg::LEDRequest>::SharedPtr pub_led;
    rclcpp::Publisher<peripheral_msg::msg::BusServoRequest>::SharedPtr pub_bus_servo;
    rclcpp::Publisher<peripheral_msg::msg::BuzzerRequest>::SharedPtr pub_buzzer;
    
    // Service
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_ini;
};


///////////////////////////////

#endif