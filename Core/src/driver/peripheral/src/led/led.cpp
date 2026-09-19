#include "led.h"
#include <rclcpp/logger.hpp>

LED::LED() 
    : Node("led")
{
    this->pub = this->create_publisher<peripheral_msg::msg::LEDRequest>("~", 10);

    // Waiting for robot controller node to start
    this->cli_robot_controller = this->create_client<std_srvs::srv::Trigger>("/robot_controller/initialization_complete");
    this->cli_robot_controller->wait_for_service();
}

void LED::set_state(decltype(peripheral_msg::msg::LED::id) led_id, 
                decltype(peripheral_msg::msg::LED::on_duration) on_duration, 
                decltype(peripheral_msg::msg::LED::off_duration) off_duration, 
                decltype(peripheral_msg::msg::LED::repeat_count) repeat_count) 
{
    auto led = peripheral_msg::msg::LED();
    led.id = led_id;
    led.on_duration = on_duration;
    led.off_duration = off_duration;
    led.repeat_count = repeat_count;

    auto msg = peripheral_msg::msg::LEDRequest();
    msg.leds.emplace_back(led);
    
    this->pub->publish(msg);

    RCLCPP_INFO(
        rclcpp::get_logger(""), 
        "LED ID: %d, On Duration: %.2fs, Off Duration: %.2fs, Repeat Count: %d", 
        led.id,
        led.on_duration,
        led.off_duration,
        led.repeat_count
    );
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    const auto& node = std::make_shared<LED>();
    node->set_state(0, 1, 1, 5);

    rclcpp::shutdown();

    return 0;
}