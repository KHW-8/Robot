///////////////* Headers *///////////////
#include "led.h"

/* Core */
#include "board_msgs/srv/packet.hpp"
#include "peripheral_msgs/srv/led_request.hpp"
#include "utility.hpp"
#include "peripheral_msgs/msg/led.hpp"
#include "peripheral.h"

/* ROS */
#include <rclcpp/qos.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/logger.hpp>

/* STD*/
#include <functional>
#include <memory>

/////////////////////////////////////////

///////////////* Functions *///////////////

LED::LED() 
    : Node("led")
{
    initialize();
}

auto LED::set(peripheral_msgs::msg::LED::_id_type led_id, 
              peripheral_msgs::msg::LED::_on_duration_type on_duration, 
              peripheral_msgs::msg::LED::_off_duration_type off_duration, 
              peripheral_msgs::msg::LED::_repeat_count_type repeat_count) -> void 
{
    auto led = peripheral_msgs::msg::LED();
    led.id = led_id;
    led.on_duration = on_duration;
    led.off_duration = off_duration;
    led.repeat_count = repeat_count;
    
    set({ led });
}

auto LED::set(const std::vector<peripheral_msgs::msg::LED>& leds) -> void {
    // Print request
    for (const auto& led : leds) {
        RCLCPP_INFO(
            rclcpp::get_logger(""), 
            "ID: %d, On Duration: %.2fs°, Off Duration: %.2fs, Repeat Count: %d", 
            led.id,
            led.on_duration,
            led.off_duration,
            led.repeat_count
        );
    }

    // Create a board RobotController packet
    auto request = std::make_shared<board_msgs::srv::Packet::Request>();
    request->tx_only = true;

    // Peripheral ID
    request->peripheral_id = static_cast<uint8_t>(Peripheral::LED);
    // LED count
    request->data.emplace_back(leds.size());
    // LED information
    for (const auto& led : leds) {
        // Convert uint16_t and uint32_t numbers to byte vector. (Size equals to 2 or 4)
        const auto& on_duration = to_byte_vector<decltype(led.on_duration)>(led.on_duration); 
        const auto& off_duration = to_byte_vector<decltype(led.off_duration)>(led.off_duration); 
        const auto& repeat_count = to_byte_vector<decltype(led.repeat_count)>(led.repeat_count);

        request->data.emplace_back(led.id);
        request->data.insert(request->data.end(), on_duration.begin(), on_duration.end()); 
        request->data.insert(request->data.end(), off_duration.begin(), off_duration.end()); 
        request->data.insert(request->data.end(), repeat_count.begin(), repeat_count.end()); 
    }

    this->cli_board_controller->async_send_request(request);
}

auto LED::initialize() -> void {
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
    this->srv_request = this->create_service<peripheral_msgs::srv::LEDRequest>(
        "~/request",
        std::bind(&LED::handle_request, this, std::placeholders::_1, std::placeholders::_2)
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

auto LED::handle_request(const peripheral_msgs::srv::LEDRequest::Request::SharedPtr request, 
                         const peripheral_msgs::srv::LEDRequest::Response::SharedPtr response) -> void 
{
    set(request->leds);

    response->success = true;
}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<LED>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}

///////////////////////////////////////////