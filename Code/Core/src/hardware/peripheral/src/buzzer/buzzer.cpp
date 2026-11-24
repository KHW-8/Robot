///////////////* Headers *///////////////

#include "buzzer.h"

/* STD*/
#include <functional>

/* ROS */
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

/* Core */
#include "board_msgs/srv/packet.hpp"
#include "peripheral_msgs/msg/buzzer_request.hpp"
#include "peripheral_msgs/srv/buzzer_request.hpp"
#include "utility.hpp"
#include "peripheral.h"

/////////////////////////////////////////

///////////////* Functions *///////////////

Buzzer::Buzzer() 
    : Node("buzzer")
{
    initialize();
}

auto Buzzer::set(peripheral_msgs::msg::BuzzerRequest::_frequency_type freq, 
         peripheral_msgs::msg::BuzzerRequest::_on_duration_type on_duration, 
         peripheral_msgs::msg::BuzzerRequest::_off_duration_type off_duration, 
         peripheral_msgs::msg::BuzzerRequest::_repeat_count_type repeat_count) -> void
{
    auto buzzer = peripheral_msgs::msg::Buzzer();
    buzzer.frequency = freq;
    buzzer.on_duration = on_duration;
    buzzer.off_duration = off_duration;
    buzzer.repeat_count= repeat_count;

    // Send message
    set({ buzzer });
}

auto Buzzer::set(const std::vector<peripheral_msgs::msg::Buzzer>& buzzers) -> void {
    // Print request info
    for (auto buzzer : buzzers) {
        RCLCPP_INFO(
            rclcpp::get_logger(""), 
            "Published Buzzer State: freq=%d, on_duration=%.2f, off_duration=%.2f, repeat=%d",
            buzzer.frequency,
            buzzer.on_duration,
            buzzer.off_duration,
            buzzer.repeat_count
        );
    }

    auto request = std::make_shared<board_msgs::srv::Packet::Request>();
    request->tx_only = true;

    for (auto buzzer : buzzers) {
        // Convert uint16_t and uint32_t numbers to byte vector. (Size equals to 2 or 4)
        const auto& freq = to_byte_vector<decltype(buzzer.frequency)>(buzzer.frequency); 
        const auto& on_duration = to_byte_vector<decltype(buzzer.on_duration)>(buzzer.on_duration); 
        const auto& off_duration = to_byte_vector<decltype(buzzer.off_duration)>(buzzer.off_duration);
        const auto& repeat_count = to_byte_vector<decltype(buzzer.repeat_count)>(buzzer.repeat_count);

        // Peripheral ID
        request->peripheral_id = static_cast<uint8_t>(Peripheral::BUZZER);
        // Frequency
        request->data.insert(request->data.end(), freq.begin(), freq.end()); 
        // On duration
        request->data.insert(request->data.end(), on_duration.begin(), on_duration.end()); 
        // Off duration
        request->data.insert(request->data.end(), off_duration.begin(), off_duration.end()); 
        // Repeat count
        request->data.insert(request->data.end(), repeat_count.begin(), repeat_count.end()); 
    }

    this->cli_board_controller->async_send_request(request);
}

auto Buzzer::initialization_complete([[maybe_unused]]const std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
                             const std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> void
{
    response->success = true;
}

auto Buzzer::handle_request(const peripheral_msgs::srv::BuzzerRequest::Request::SharedPtr request, 
                            const peripheral_msgs::srv::BuzzerRequest::Response::SharedPtr response) -> void 
{
    set(request->buzzers);

    response->success = true;
}

auto Buzzer::initialize() -> void {
    // Waiting for robot controller node to start
    this->cli_board_controller_ready = this->create_client<std_srvs::srv::Trigger>("/board_controller/initialization_complete");
    this->cli_board_controller_ready->wait_for_service();

   // Create callback groups
    this->cli_board_controller_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    
    // Create clients
    this->cli_board_controller = this->create_client<board_msgs::srv::Packet>(
        "/board_controller/packet",
        rclcpp::ServicesQoS(),
        cli_board_controller_cb_group
    );


    // Create services
    this->srv_request = this->create_service<peripheral_msgs::srv::BuzzerRequest>(
        "~/request",
        std::bind(&Buzzer::handle_request, this, std::placeholders::_1, std::placeholders::_2)
    );
  
    // Complete initialization and notify all other nodes
    this->srv_ini = this->create_service<std_srvs::srv::Trigger>(
        "~/initialization_complete", 
        std::bind(&Buzzer::initialization_complete, this, std::placeholders::_1, std::placeholders::_2)
    );
}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Buzzer>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown(); // Shutdown

    return 0;
}

///////////////////////////////////////////