//////////* Headers *//////////

#include "robot_controller.h"

/* Core */
//// Hardware
#include "board_msg/msg/packet.hpp"
#include "bus_servo.h"
#include "gui_msgs/msg/command.hpp"

/* ROS2 */
#include <functional>
#include <rclcpp/executors.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

/* STD */
#include <cstdint>
#include <chrono>
#include <map>
#include <thread>
#include <vector>

///////////////////////////////

//////////* Functions *//////////

RobotController::RobotController() 
    :Node("robot_controller")
{
    initialize();
}

auto RobotController::read_servo_pos(const std::vector<decltype(peripheral_msg::msg::BusServo::id)>& ids) -> void {
   // Create request
    auto request = peripheral_msg::msg::BusServoRequest();
    request.cmd = static_cast<uint8_t>(BusServoCMD::READ_ANGLE);
    request.query_only = true;

    for (const auto& id : ids) {
        // Generate servo message
        auto servo = peripheral_msg::msg::BusServo();
        servo.id = id;

        request.servos.emplace_back(servo);
    }

    this->pub_servo->publish(request);

    // Print log
    for (const auto& servo : request.servos) 
        RCLCPP_INFO(rclcpp::get_logger(""), "Servo Id: %d", servo.id);
}


auto RobotController::set_servo_pos(const decltype(peripheral_msg::msg::BusServo::id)& id, 
                                    const decltype(peripheral_msg::msg::BusServo::angle)& angle, 
                                    const decltype(peripheral_msg::msg::BusServo::duration)& duration) -> void
{
    // Generate servo message
    auto servo = peripheral_msg::msg::BusServo();
    servo.id = id;
    servo.angle = angle;
    servo.duration = duration;

    auto request = peripheral_msg::msg::BusServoRequest();
    request.cmd = static_cast<uint8_t>(BusServoCMD::SET_ROTAION_ANGLE_AND_DURATION);
    request.query_only = false;
    request.servos.emplace_back(servo);

    // Send message
    this->pub_servo->publish(request);

    // Print log
    for (const auto& servo : request.servos) 
        RCLCPP_INFO(rclcpp::get_logger(""), "Servo Id: %d, Angle: %d°, Duration: %.2fs", servo.id, servo.angle, servo.duration);
}


auto RobotController::set_buzzer(decltype(peripheral_msg::msg::BuzzerRequest::frequency) freq, 
                                decltype(peripheral_msg::msg::BuzzerRequest::on_duration) on_duration, 
                                decltype(peripheral_msg::msg::BuzzerRequest::off_duration) off_duration, 
                                decltype(peripheral_msg::msg::BuzzerRequest::repeat_count) repeat_count) -> void 
{
    auto msg = peripheral_msg::msg::BuzzerRequest();
    msg.frequency = freq;
    msg.on_duration = on_duration;
    msg.off_duration = off_duration;
    msg.repeat_count= repeat_count;

    // Send message
    this->pub_buzzer->publish(msg);
    RCLCPP_INFO(
        rclcpp::get_logger(""), 
        "Published Buzzer State: freq=%d, on_duration=%.2f, off_duration=%.2f, repeat=%d",
        msg.frequency,
        msg.on_duration,
        msg.off_duration,
        msg.repeat_count
    );
}

auto RobotController::set_led(decltype(peripheral_msg::msg::LED::id) led_id, 
                 decltype(peripheral_msg::msg::LED::on_duration) on_duration, 
                 decltype(peripheral_msg::msg::LED::off_duration) off_duration, 
                 decltype(peripheral_msg::msg::LED::repeat_count) repeat_count) -> void
{
    auto led = peripheral_msg::msg::LED();
    led.id = led_id;
    led.on_duration = on_duration;
    led.off_duration = off_duration;
    led.repeat_count = repeat_count;

    auto msg = peripheral_msg::msg::LEDRequest();
    msg.leds.emplace_back(led);
    
    this->pub_led->publish(msg);

    RCLCPP_INFO(
        rclcpp::get_logger(""), 
        "LED ID: %d, On Duration: %.2fs, Off Duration: %.2fs, Repeat Count: %d", 
        led.id,
        led.on_duration,
        led.off_duration,
        led.repeat_count
    );
}

auto RobotController::receive_control_panel_msg(const gui_msgs::msg::Command& msg) -> void {
    set_servo_pos(msg.servo_id, msg.angle, 1);
}


auto RobotController::initialization_complete([[maybe_unused]]const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                              const std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> void 
{
    response->success = true;
}

auto RobotController::initialize() -> void {
    initialize_board_controller();
    initialize_buzzer();
    initialize_led();
    initialize_servos();
    initialize_vision();
    initialize_control_panel();

    // Complete initialization and notify all other nodes
    this->srv_ini = this->create_service<std_srvs::srv::Trigger>(
        "~/initialization_complete", 
        std::bind(&RobotController::initialization_complete, this, std::placeholders::_1, std::placeholders::_2)
    );
}

auto RobotController::initialize_board_controller() -> void {
    // Wait for board controller
    this->cli_board_controller = this->create_client<std_srvs::srv::Trigger>("/board_controller/initialization_complete");
    this->cli_board_controller->wait_for_service();

    // Create a publisher for board controller
    this->pub_packet = this->create_publisher<board_msg::msg::Packet>("/board_controller/packet", 10);
}

auto RobotController::initialize_buzzer() -> void {
    // Wait for board buzzer
    this->cli_buzzer = this->create_client<std_srvs::srv::Trigger>("/buzzer/initialization_complete");
    this->cli_buzzer->wait_for_service();

    // Create a publisher for buzzer
    this->pub_buzzer = this->create_publisher<peripheral_msg::msg::BuzzerRequest>("/buzzer", 10);
}

auto RobotController::initialize_led() -> void {
    // Wait for LED
    this->cli_led = this->create_client<std_srvs::srv::Trigger>("/led/initialization_complete");
    this->cli_led->wait_for_service();

    // Create a publisher for LED
    this->pub_led = this->create_publisher<peripheral_msg::msg::LEDRequest>("/led",10);
}

auto RobotController::initialize_servos() -> void {
    // Wait for servo
    this->cli_servo = this->create_client<std_srvs::srv::Trigger>("/bus_servo/initialization_complete");
    this->cli_servo->wait_for_service();

    // Create a publisher for servo 
    this->pub_servo = this->create_publisher<peripheral_msg::msg::BusServoRequest>("/bus_servo", 10);

    // Set servos' angle
    std::map<uint8_t, uint8_t> map = {
        {1, 0},
        {2, 120},
        {3, 30},
        {4, 195},
        {5, 140},
        {6, 120},
    };

    for (auto pair : map) {
        set_servo_pos(pair.first, pair.second, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

auto RobotController::initialize_vision() -> void {
    // Wait for board controller
    this->cli_vision = this->create_client<std_srvs::srv::Trigger>("/vision/initialization_complete");
    this->cli_vision->wait_for_service();
}

auto RobotController::initialize_control_panel() -> void {
    this->sub_gui = this->create_subscription<gui_msgs::msg::Command>(
        "/control_panel/command",
        1,
        std::bind(&RobotController::receive_control_panel_msg, this, std::placeholders::_1)
    );
}

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotController>());
    rclcpp::shutdown();

    return 0;
}

/////////////////////////////////