#ifndef BUS_SERVO_H
#define BUS_SERVO_H

///////////////* Headers *///////////////

/* Core */
//// Hardware
/// Interfaces
#include "peripheral_msgs/msg/bus_servo.hpp"
#include "peripheral_msgs/srv/bus_servo_request.hpp"
#include "board_msgs/srv/packet.hpp"

/* ROS2 */
#include <cstdint>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <vector>

/////////////////////////////////////////

////////////////////* Enumeration *////////////////////

enum class BusServoCMD {
    NONE = 0,
    SET_ROTAION_ANGLE_AND_DURATION = 1,
    READ_ROTAION_ANGLE_AND_DURATION = 2,
    SET_ROTAION_ANGLE_AND_DURATION_WITH_WAITING_STATE  = 7,
    READ_ROTAION_ANGLE_AND_DURATION_WITH_WAITING_STATE = 8,
    START_ROTATION = 11,
    STOP_ROTATION = 12,
    SET_ID = 13,
    READ_ID = 14,
    SET_ANGLE_DEVIATION = 17,
    SET_ANGLE_DEVIATION_DURING_POWER_OFF = 18,
    READ_ANGLE_DEVIATION = 19,
    SET_ANGLE_LIMIT = 20,
    READ_ANGLE_LIMIT = 21,
    SET_MAXIMUM_VIN = 22,
    READ_MAXIMUM_VIN = 23,
    SET_MAXIMUM_TEMPRATURE = 24,
    READ_MAXIMUM_TEMPRATURE = 25,
    READ_TEMPRATURE = 26,
    READ_VIN = 27,
    READ_ANGLE = 28,
    /*
     * Mode
     * 0: Servo
     * 1: Motor
     */
    SET_MODE = 29,
    READ_MODE = 30,
    /*
     * Power State
     * 0: OFF
     * 1: ON
     */
    SET_POWER_STATE = 31,
    READ_POWER_STATE = 32,
    /*
     * LED State
     * 0: OFF
     * 1: ON
     */
    SET_LED_STATE = 33,
    READ_LED_STATE = 34,

    /*
     * Error
     * 0: 
     * 1: 
     */
    SET_ERROR = 35,
    READ_ERROR = 36,
    READ_DISTANCE = 48
};

enum RequestResult {
    OK,
    ERROR
};

///////////////////////////////////////////////////////

////////////////////* Classes *////////////////////

class BusServo : public rclcpp::Node {
public:
    BusServo();
    
public:
    auto read_pos(const std::vector<peripheral_msgs::msg::BusServo::_id_type>& servo_ids) -> std::vector<peripheral_msgs::msg::BusServo>;
    auto set_pos(const std::vector<peripheral_msgs::msg::BusServo>& servos) -> void;

private:
    // Callback
    auto initialization_complete([[maybe_unused]]const std_srvs::srv::Trigger::Request::SharedPtr& request, const std_srvs::srv::Trigger::Response::SharedPtr& response) -> void;
    auto handle_request(const peripheral_msgs::srv::BusServoRequest::Request::SharedPtr& request, const peripheral_msgs::srv::BusServoRequest::Response::SharedPtr& response) -> void;
    
    // Handle
    auto handle_packet(const std::vector<uint8_t>& packet) -> std::vector<peripheral_msgs::msg::BusServo>;

    // Initialization
    void initialize();

private:
    // Callback group
    rclcpp::CallbackGroup::SharedPtr cli_board_controller_cb_group;

    // Client
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_board_controller_ready;
    rclcpp::Client<board_msgs::srv::Packet>::SharedPtr cli_board_controller;

    // Service
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_ini;
    rclcpp::Service<peripheral_msgs::srv::BusServoRequest>::SharedPtr srv_request;
};

/////////////////////////////////////////////////////

#endif