// ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
// LED Node
#include "robot_controller_msg/msg/le_ds.hpp"

class LEDNode : public rclcpp::Node {
public: 
    LEDNode() 
        : Node("led_node")
    {
        this->pub = this->create_publisher<robot_controller_msg::msg::LEDs>("/robot_controller/led", 10);

        // Waiting for robot controller node to start
        this->client_robot_controller = this->create_client<std_srvs::srv::Trigger>("/robot_controller/initialization_complete");
        this->client_robot_controller->wait_for_service();
    }

public:
    void publish_led_state() {
        auto led = robot_controller_msg::msg::LED();
        led.id = 1;
        led.on_duration = 1;
        led.off_duration = 1;
        led.repeat_count = 5;

        auto msg = robot_controller_msg::msg::LEDs();
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

private:
    rclcpp::Publisher<robot_controller_msg::msg::LEDs>::SharedPtr pub;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_robot_controller;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    const auto& node = std::make_shared<LEDNode>();
    node->publish_led_state();

    rclcpp::shutdown();

    return 0;
}