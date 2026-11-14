//////////* Headers *//////////

#include "robot_controller.h"

/* Core */
//// Hardware
#include "board_msgs/msg/packet.hpp"
#include "bus_servo.h"
#include "gui_msgs/msg/command.hpp"
#include "kinematics_msgs/srv/pose.hpp"

/* ROS2 */
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/future_return_code.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

/* STD */
#include <cstdint>
#include <chrono>
#include <functional>
#include <map>
#include <thread>
#include <vector>

///////////////////////////////

//////////* Functions *//////////

RobotController::RobotController() 
    :Node("robot_controller")
{
    initialize();
    test();
}

auto RobotController::read_servo_pos(const std::vector<decltype(peripheral_msgs::msg::BusServo::id)>& ids) -> void {
   // Create request
    auto request = peripheral_msgs::msg::BusServoRequest();
    request.cmd = static_cast<uint8_t>(BusServoCMD::READ_ANGLE);
    request.query_only = true;

    for (const auto& id : ids) {
        // Generate servo message
        auto servo = peripheral_msgs::msg::BusServo();
        servo.id = id;

        request.servos.emplace_back(servo);
    }

    this->pub_servo->publish(request);

    // Print log
    for (const auto& servo : request.servos) 
        RCLCPP_INFO(rclcpp::get_logger(""), "Servo Id: %d", servo.id);
}


auto RobotController::set_servo_pos(const decltype(peripheral_msgs::msg::BusServo::id)& id, 
                                    const decltype(peripheral_msgs::msg::BusServo::angle)& angle, 
                                    const decltype(peripheral_msgs::msg::BusServo::duration)& duration) -> void
{
    // Generate servo message
    auto servo = peripheral_msgs::msg::BusServo();
    servo.id = id;
    servo.angle = angle;
    servo.duration = duration;

    auto request = peripheral_msgs::msg::BusServoRequest();
    request.cmd = static_cast<uint8_t>(BusServoCMD::SET_ROTAION_ANGLE_AND_DURATION);
    request.query_only = false;
    request.servos.emplace_back(servo);

    // Send message
    this->pub_servo->publish(request);

    // Print log
    for (const auto& servo : request.servos) 
        RCLCPP_INFO(rclcpp::get_logger(""), "Servo Id: %d, Angle: %d°, Duration: %.2fs", servo.id, servo.angle, servo.duration);
}


auto RobotController::set_buzzer(decltype(peripheral_msgs::msg::BuzzerRequest::frequency) freq, 
                                decltype(peripheral_msgs::msg::BuzzerRequest::on_duration) on_duration, 
                                decltype(peripheral_msgs::msg::BuzzerRequest::off_duration) off_duration, 
                                decltype(peripheral_msgs::msg::BuzzerRequest::repeat_count) repeat_count) -> void 
{
    auto msg = peripheral_msgs::msg::BuzzerRequest();
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

auto RobotController::set_led(decltype(peripheral_msgs::msg::LED::id) led_id, 
                 decltype(peripheral_msgs::msg::LED::on_duration) on_duration, 
                 decltype(peripheral_msgs::msg::LED::off_duration) off_duration, 
                 decltype(peripheral_msgs::msg::LED::repeat_count) repeat_count) -> void
{
    auto led = peripheral_msgs::msg::LED();
    led.id = led_id;
    led.on_duration = on_duration;
    led.off_duration = off_duration;
    led.repeat_count = repeat_count;

    auto msg = peripheral_msgs::msg::LEDRequest();
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

auto RobotController::test() -> void {
    auto request = std::make_shared<kinematics_msgs::srv::Pose::Request>();
    request->pose.position.x = 0.2;
    request->pose.position.y = -0.1;
    request->pose.position.z = 0.05;
    request->pitch = 90;

    auto future = this->cli_kinematics_calculator->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) != rclcpp::FutureReturnCode::SUCCESS) {
         RCLCPP_ERROR(
            rclcpp::get_logger(""), 
            "File: %s, Function: %s, Line: %d, Error: Service call failed.", 
            __FILE__, 
            __FUNCTION__,
            __LINE__
        );

        return;
    }

    auto res = future.get();
    if (!res->success) {
        RCLCPP_INFO(rclcpp::get_logger(""), "No solutions.");
        return;
    }

    RCLCPP_INFO(rclcpp::get_logger(""), "Get the result.");

    for (const auto& e : res->servo_angles) 
        RCLCPP_INFO(rclcpp::get_logger(""), "Angle: %f", e);
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
    // Hardware
    // initialize_board_controller();
    // initialize_buzzer();
    // initialize_led();
    // initialize_servos();
    // Functional
    initialize_kinematics_calculator();
    // initialize_vision();
    // Application
    initialize_control_panel();

    // Complete initialization and notify all other nodes
    this->srv_ini = this->create_service<std_srvs::srv::Trigger>(
        "~/initialization_complete", 
        std::bind(&RobotController::initialization_complete, this, std::placeholders::_1, std::placeholders::_2)
    );
}

auto RobotController::initialize_board_controller() -> void {
    // Wait for board controller
    this->cli_board_controller_ready = this->create_client<std_srvs::srv::Trigger>("/board_controller/initialization_complete");
    this->cli_board_controller_ready->wait_for_service();

    // Create a publisher for board controller
    this->pub_packet = this->create_publisher<board_msgs::msg::Packet>("/board_controller/packet", 10);
}

auto RobotController::initialize_buzzer() -> void {
    // Wait for board buzzer
    this->cli_buzzer_ready = this->create_client<std_srvs::srv::Trigger>("/buzzer/initialization_complete");
    this->cli_buzzer_ready->wait_for_service();

    // Create a publisher for buzzer
    this->pub_buzzer = this->create_publisher<peripheral_msgs::msg::BuzzerRequest>("/buzzer", 10);
}

auto RobotController::initialize_led() -> void {
    // Wait for LED
    this->cli_led_ready = this->create_client<std_srvs::srv::Trigger>("/led/initialization_complete");
    this->cli_led_ready->wait_for_service();

    // Create a publisher for LED
    this->pub_led = this->create_publisher<peripheral_msgs::msg::LEDRequest>("/led",10);
}

auto RobotController::initialize_servos() -> void {
    // Wait for servo
    this->cli_sevo_ready = this->create_client<std_srvs::srv::Trigger>("/bus_servo/initialization_complete");
    this->cli_sevo_ready->wait_for_service();

    // Create a publisher for servo 
    this->pub_servo = this->create_publisher<peripheral_msgs::msg::BusServoRequest>("/bus_servo", 10);

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

auto RobotController::initialize_kinematics_calculator() -> void {
    // Wait for kinematics calculator
    this->cli_kinematics_calculator_ready = this->create_client<std_srvs::srv::Trigger>("/kinematics_calculator/initialization_complete");
    this->cli_kinematics_calculator_ready->wait_for_service();

    this->cli_kinematics_calculator = this->create_client<kinematics_msgs::srv::Pose>("/kinematics_calculator/pose");
}


auto RobotController::initialize_vision() -> void {
    // Wait for vision
    this->cli_vision_ready = this->create_client<std_srvs::srv::Trigger>("/vision/initialization_complete");
    this->cli_vision_ready->wait_for_service();
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