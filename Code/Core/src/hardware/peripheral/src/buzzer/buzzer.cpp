#include "buzzer.h"

/* STD*/
#include <functional>

/* ROS */
#include <rclcpp/executors.hpp>

/* Core */
#include "board_msg/msg/packet.hpp"
#include "peripheral_msg/msg/buzzer_request.hpp"
#include "utility.hpp"
#include "peripheral.h"

Buzzer::Buzzer() 
    : Node("buzzer")
{
    initialize();
}

auto Buzzer::set(const peripheral_msg::msg::BuzzerRequest& request) -> void {
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

    // Create a board RobotController packet
    auto packet = board_msg::msg::Packet();

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

    this->pub_board_controller->publish(packet);
}

auto Buzzer::initialization_complete([[maybe_unused]]const std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
                             const std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> void
{
    response->success = true;
}

auto Buzzer::initialize() -> void {
    // Waiting for robot controller node to start
    this->cli_board_controller = this->create_client<std_srvs::srv::Trigger>("/board_controller/initialization_complete");
    this->cli_board_controller->wait_for_service();

    // Create a publisher of board controller
    this->pub_board_controller = this->create_publisher<board_msg::msg::Packet>("~/packet", 1);

    // Create a subscription of robot controller
    this->sub_robot_controller = this->create_subscription<peripheral_msg::msg::BuzzerRequest>(
        "~", 
        10, 
        std::bind(&Buzzer::set, this, std::placeholders::_1)
    );

    // Complete initialization and notify all other nodes
    this->srv_ini = this->create_service<std_srvs::srv::Trigger>(
        "~/initialization_complete", 
        std::bind(&Buzzer::initialization_complete, this, std::placeholders::_1, std::placeholders::_2)
    );
}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Buzzer>());
    rclcpp::shutdown(); // Shutdown

    return 0;
}