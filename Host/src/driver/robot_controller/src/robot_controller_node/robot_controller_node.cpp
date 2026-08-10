#include "robot_controller_node.h"

// Robot Controller
#include "peripheral.h"
#include "utility.hpp"

RobotController::RobotController() 
    :Node("robot_controller")
{
    initialize();
}

auto RobotController::set_bus_servo(const robot_controller_msg::msg::Servos::UniquePtr& msg) -> void {
    for (const auto& servo : msg->servos) {
        RCLCPP_INFO(
            rclcpp::get_logger(""), 
            "Servo Id: %d, Position: %d°, Duration: %.2fs", 
            servo.id,
            servo.position,
            servo.duration
        );
    }
}

auto RobotController::set_buzzer(const robot_controller_msg::msg::Buzzer& msg) -> void {
    RCLCPP_INFO(
        rclcpp::get_logger(""), 
        "Frequency: %d, On Duration: %.2fs°, Off Duration: %.2fs, Repeat Count: %d", 
        msg.freq,
        msg.on_duration,
        msg.off_duration,
        msg.repeat_count
    );
}

auto RobotController::set_led(const robot_controller_msg::msg::LEDs& msg) -> void {
    auto _msg = board_controller_msg::msg::Packet();

    // Peripheral ID
    _msg.peripheral_id = static_cast<uint8_t>(Peripheral::LED);
    // LED count
    _msg.array_data.emplace_back(msg.leds.size());
    // LED information
    for (const auto& led : msg.leds) {
        
        // Convert uint16_t and uint32_t numbers to byte vector. (Size equals to 2 or 4)
        const auto& on_duration = to_byte_vector<uint32_t>(static_cast<uint32_t>(led.on_duration * 1000)); // Convert second to millisecond
        const auto& off_duration = to_byte_vector<uint32_t>(static_cast<uint32_t>(led.off_duration * 1000)); // Convert second to millisecond
        const auto& repeat_count = to_byte_vector<uint16_t>(led.repeat_count);

        _msg.array_data.emplace_back(led.id);
        _msg.array_data.insert(_msg.array_data.end(), on_duration.begin(), on_duration.end()); 
        _msg.array_data.insert(_msg.array_data.end(), off_duration.begin(), off_duration.end()); 
        _msg.array_data.insert(_msg.array_data.end(), repeat_count.begin(), repeat_count.end()); 
    }

    this->pub_packet->publish(_msg);
}

auto RobotController::initialization_complete([[maybe_unused]]const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                              const std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> void 
{
    response->success = true;
}

auto RobotController::initialize() -> void {
    // Create the subscription of bus servo node 
    this->sub_bus_servo = this->create_subscription<robot_controller_msg::msg::Servos>(
        "~/bus_servo", 
        10, 
        std::bind(&RobotController::set_bus_servo, this, std::placeholders::_1)
    );

    // Create the subscription of buzzer
    this->sub_buzzer = this->create_subscription<robot_controller_msg::msg::Buzzer>(
        "~/buzzer",
        10,
        std::bind(&RobotController::set_buzzer, this, std::placeholders::_1)
    );

    // Create the subscription of LED
    this->sub_led = this->create_subscription<robot_controller_msg::msg::LEDs>(
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