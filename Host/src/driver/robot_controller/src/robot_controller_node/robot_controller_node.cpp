#include "robot_controller_node.h"

// Host
#include "peripheral.h"
#include "board_controller_msg/msg/host_packet.hpp"

RobotController::RobotController() 
    :Node("robot_controller")
{
    initialize();
}

auto RobotController::set_bus_servo(robot_controller_msg::msg::Servos::UniquePtr msg) -> void {
    for (auto& servo : msg->servos) {
        RCLCPP_INFO(
            rclcpp::get_logger(""), 
            "Servo Id: %d, Position: %d°, Duration: %.2fs", 
            servo.id,
            servo.position,
            servo.duration
        );
    }
}

auto RobotController::set_buzzer(robot_controller_msg::msg::Buzzer msg) -> void {
    RCLCPP_INFO(
        rclcpp::get_logger(""), 
        "Frequency: %d, On Duration: %.2fs°, Off Duration: %.2fs, Repeat Count: %d", 
        msg.freq,
        msg.on_duration,
        msg.off_duration,
        msg.repeat_count
    );
}

auto RobotController::set_led(robot_controller_msg::msg::LEDs msg) -> void {
    auto _msg = board_controller_msg::msg::HostPacket();
    // Header
    _msg.host_packet.emplace_back(0x55);
    _msg.host_packet.emplace_back(0x55);
    // Peripheral ID
    _msg.host_packet.emplace_back(static_cast<uint8_t>(Peripheral::LED));
    // LED count
    _msg.host_packet.emplace_back(1);
    // Data length
    _msg.host_packet.emplace_back(5);

    for (auto& led : msg.leds) {
        _msg.host_packet.emplace_back(led.id);
        _msg.host_packet.emplace_back();
        _msg.host_packet.emplace_back(led.repeat_count);
    }
}

auto RobotController::initialization_complete([[maybe_unused]]const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                              const std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> void 
{
    response->success = true;
}

auto RobotController::initialize() -> void {
    // Create subscription of bus servo node 
    this->bus_servo_sub = this->create_subscription<robot_controller_msg::msg::Servos>(
        "~/bus_servo", 
        10, 
        std::bind(&RobotController::set_bus_servo, this, std::placeholders::_1)
    );

    // Create subscription of LED
    this->buzzer_sub = this->create_subscription<robot_controller_msg::msg::Buzzer>(
        "~/buzzer",
        10,
        std::bind(&RobotController::set_buzzer, this, std::placeholders::_1)
    );

    // Create subscription of LED
    this->led_sub = this->create_subscription<robot_controller_msg::msg::LEDs>(
        "~/led",
        10,
        std::bind(&RobotController::set_led, this, std::placeholders::_1)
    );

    // Wait for board controller node
    this->board_controller_client = this->create_client<std_srvs::srv::Trigger>("/board_controller/initialization_complete");
    this->board_controller_client.get()->wait_for_service();

    // Complete initialization and notify all other nodes
    this->ini_srv = this->create_service<std_srvs::srv::Trigger>(
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