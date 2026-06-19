// ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
// ArmPi
#include "robot_controller_msg/msg/led_state.hpp"

class LEDNode : public rclcpp::Node {
public: 
    LEDNode() 
        : Node("led_node")
    {
        this->pub = this->create_publisher<robot_controller_msg::msg::LEDState>("/robot_controller/led/set", 10);

        // Waiting for robot arm underlying control services to start
        this->client = this->create_client<std_srvs::srv::Trigger>("/robot_controller/init_complete");
        this->client->wait_for_service();

        // Send message
        publish_led_state();
    }

public:
    void publish_led_state() {
        auto msg = robot_controller_msg::msg::LEDState();
        msg.id = 2;
        msg.on_duration = 0.1;
        msg.off_duration = 0.5;
        msg.repeat_count = 10;
        
        this->pub->publish(msg);
        RCLCPP_INFO(
            rclcpp::get_logger(""), 
            "Published LED State: id=%d, on_duration=%.2f, off_duration=%.2f, repeat=%d",
            msg.id,
            msg.on_duration,
            msg.on_duration,
            msg.repeat_count
        );
    }

private:
    rclcpp::Publisher<robot_controller_msg::msg::LEDState>::SharedPtr pub;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin_some(std::make_shared<LEDNode>());
    rclcpp::shutdown();

    return 0;
}