#include "led.h"

///////////////* Headers *///////////////

// Core
#include "utility.hpp"
#include "peripheral_msg/msg/led.hpp"
#include "board_msg/msg/packet.hpp"
#include "peripheral.h"

/* ROS */
#include <rclcpp/executors.hpp>
#include <rclcpp/logger.hpp>

/* STD*/
#include <functional>

/////////////////////////////////////////

LED::LED() 
    : Node("led")
{
    initialize();
}

auto LED::set(const peripheral_msg::msg::LEDRequest& request) -> void {
    // Create a board RobotController packet
    auto packet = board_msg::msg::Packet();

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

    this->pub_board_controller->publish(packet);
}

auto LED::initialize() -> void {
    // Waiting for robot controller node to start
    this->cli_board_controller = this->create_client<std_srvs::srv::Trigger>("/board_controller/initialization_complete");
    this->cli_board_controller->wait_for_service();

    // Create a publisher of board controller
    this->pub_board_controller = this->create_publisher<board_msg::msg::Packet>("~/packet", 10);

    // Create a subscription of robot controller
    this->sub_robot_controller = this->create_subscription<peripheral_msg::msg::LEDRequest>(
        "~",
        10, 
        std::bind(&LED::set, this, std::placeholders::_1)
    );
    
    // Complete initialization and notify all other nodes
    this->srv_ini = this->create_service<std_srvs::srv::Trigger>(
        "~/initialization_complete", 
        std::bind(&LED::initialization_complete, this, std::placeholders::_1, std::placeholders::_2)
    );
}

auto LED::initialization_complete([[maybe_unused]]const std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
                             const std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> void
{
    response->success = true;
}



int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LED>());
    rclcpp::shutdown();

    return 0;
}