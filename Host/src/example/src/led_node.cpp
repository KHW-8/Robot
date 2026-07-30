// ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
// Host
#include "robot_controller_msg/msg/le_ds.hpp"

class LEDNode : public rclcpp::Node {
public: 
    LEDNode() 
        : Node("led_node")
    {
        this->pub = this->create_publisher<robot_controller_msg::msg::LEDs>("/robot_controller/led", 10);

        // Waiting for robot controller node to start
        this->robot_controller_client = this->create_client<std_srvs::srv::Trigger>("/robot_controller/initialization_complete");
        this->robot_controller_client->wait_for_service();

        // Send message
        publish_led_state();
    }

public:
    void publish_led_state() {
        auto led = robot_controller_msg::msg::LED();
        led.id = 2;
        led.on_duration = 0.5;
        led.off_duration = 0.5;
        led.repeat_count = 10;

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
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr robot_controller_client;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin_some(std::make_shared<LEDNode>());
    rclcpp::shutdown();

    return 0;
}