#include "robot_controller_node.h"

// C STD
#include <cstdint>
// C++ STD
// ROS2
#include "rclcpp/executors.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
// Robot Controller
#include "board_controller_msg/msg/packet.hpp"
#include "peripheral.h"
#include "utility.hpp"

RobotController::RobotController() 
    :Node("robot_controller")
{
    initialize();
}

auto RobotController::set_bus_servo(const robot_controller_msg::msg::BusServoRequest::UniquePtr& request) -> void {
    // Print request info 
    for (const auto& servo : request->servos) {
        RCLCPP_INFO(
            rclcpp::get_logger(""), 
            "Servo Id: %d, angle: %d°, Duration: %.2fs", 
            servo.id,
            servo.angle,
            servo.duration
        );
    }

    // Create a board controller packet
    auto packet = board_controller_msg::msg::Packet();

    // Peripheral ID
    packet.peripheral_id = static_cast<uint8_t>(Peripheral::BUS_SERVO);
    // Command
    packet.array_data.emplace_back(request->cmd);
    // Bus servo count
    packet.array_data.emplace_back(request->servos.size());
    // Bus servo information
    if (request->query_only) {
        for (const auto& servo : request->servos) 
            // Servo ID
            packet.array_data.emplace_back(servo.id);
    } else {
        for (const auto& servo : request->servos) {
            const auto& duration = to_byte_vector<decltype(servo.duration)>(servo.duration); // Convert second to millisecond
            
            // Servo ID
            packet.array_data.emplace_back(servo.id);
            // Angle
            packet.array_data.emplace_back(servo.angle); 
            // Duration
            packet.array_data.insert(packet.array_data.end(), duration.begin(), duration.end()); 

        }
    }

    this->pub_packet->publish(packet);
}

auto RobotController::set_buzzer(const robot_controller_msg::msg::BuzzerRequest& request) -> void {
    // Print request info
    RCLCPP_INFO(
        rclcpp::get_logger(""), 
        "Frequency: %d, On Duration: %.2fs, Off Duration: %.2fs, Repeat Count: %d", 
        request.frequency,
        request.on_duration,
        request.off_duration,
        request.repeat_count
    );

    // Convert uint16_t and uint32_t numbers to byte vector. (Size equals to 2 or 4)
    const auto& freq = to_byte_vector<decltype(request.frequency)>(request.frequency); 
    const auto& on_duration = to_byte_vector<decltype(request.on_duration)>(request.on_duration); 
    const auto& off_duration = to_byte_vector<decltype(request.off_duration)>(request.off_duration);
    const auto& repeat_count = to_byte_vector<decltype(request.repeat_count)>(request.repeat_count);

    // Create a board controller packet
    auto packet = board_controller_msg::msg::Packet();

    // Peripheral ID
    packet.peripheral_id = static_cast<uint8_t>(Peripheral::BUZZER);
    // Frequency
    packet.array_data.insert(packet.array_data.end(), freq.begin(), freq.end()); 
    // On duration
    packet.array_data.insert(packet.array_data.end(), on_duration.begin(), on_duration.end()); 
    // Off duration
    packet.array_data.insert(packet.array_data.end(), off_duration.begin(), off_duration.end()); 
    // Repeat count
    packet.array_data.insert(packet.array_data.end(), repeat_count.begin(), repeat_count.end()); 

    this->pub_packet->publish(packet);
}

auto RobotController::set_led(const robot_controller_msg::msg::LEDRequest& request) -> void {
    // Create a board controller packet
    auto packet = board_controller_msg::msg::Packet();

    // Peripheral ID
    packet.peripheral_id = static_cast<uint8_t>(Peripheral::LED);
    // LED count
    packet.array_data.emplace_back(request.leds.size());
    // LED information
    for (const auto& led : request.leds) {
        RCLCPP_INFO(
            rclcpp::get_logger(""), 
            "ID: %d, On Duration: %.2fs°, Off Duration: %.2fs, Repeat Count: %d", 
            led.id,
            led.on_duration,
            led.off_duration,
            led.repeat_count
        );
            
        // Convert uint16_t and uint32_t numbers to byte vector. (Size equals to 2 or 4)
        const auto& on_duration = to_byte_vector<decltype(led.on_duration)>(led.on_duration); 
        const auto& off_duration = to_byte_vector<decltype(led.off_duration)>(led.off_duration); 
        const auto& repeat_count = to_byte_vector<decltype(led.repeat_count)>(led.repeat_count);

        packet.array_data.emplace_back(led.id);
        packet.array_data.insert(packet.array_data.end(), on_duration.begin(), on_duration.end()); 
        packet.array_data.insert(packet.array_data.end(), off_duration.begin(), off_duration.end()); 
        packet.array_data.insert(packet.array_data.end(), repeat_count.begin(), repeat_count.end()); 
    }

    this->pub_packet->publish(packet);
}

auto RobotController::initialization_complete([[maybe_unused]]const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                              const std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> void 
{
    response->success = true;
}

auto RobotController::initialize() -> void {
    // Create the subscription of bus servo node 
    this->sub_bus_servo = this->create_subscription<robot_controller_msg::msg::BusServoRequest>(
        "~/bus_servo", 
        10, 
        std::bind(&RobotController::set_bus_servo, this, std::placeholders::_1)
    );

    // Create the subscription of buzzer
    this->sub_buzzer = this->create_subscription<robot_controller_msg::msg::BuzzerRequest>(
        "~/buzzer",
        10,
        std::bind(&RobotController::set_buzzer, this, std::placeholders::_1)
    );

    // Create the subscription of LED
    this->sub_led = this->create_subscription<robot_controller_msg::msg::LEDRequest>(
        "~/led",
        10,
        std::bind(&RobotController::set_led, this, std::placeholders::_1)
    );

    // Wait for board controller node
    this->cli_board_controller = this->create_client<std_srvs::srv::Trigger>("/board_controller/initialization_complete");
    this->cli_board_controller.get()->wait_for_service();

    // Create the publisher of host packet
    this->pub_packet = this->create_publisher<board_controller_msg::msg::Packet>("/board_controller/packet", 10);

    // Complete initialization and notify all other nodes
    this->srv_ini = this->create_service<std_srvs::srv::Trigger>(
        "~/initialization_complete", 
        std::bind(&RobotController::initialization_complete, this, std::placeholders::_1, std::placeholders::_2)
    );
}


auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotController>());
    rclcpp::shutdown();

    return 0;
}