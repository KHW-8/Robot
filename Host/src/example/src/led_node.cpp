// ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
// ArmPi
#include "ros_robot_controller_msgs/msg/led_state.hpp"

class LEDController : public rclcpp::Node {
public: 
    LEDController() 
        : Node("LEDController")
    {
        this->pub = this->create_publisher<ros_robot_controller_msgs::msg::LedState>("/ros_robot_controller/set_led", 10);

        // Waiting for robot arm underlying control services to start
        this->client = this->create_client<std_srvs::srv::Trigger>("/ros_robot_controller/init_finish");
        this->client->wait_for_service();

        // Send message
        publish_led_state();
    }

public:
    void publish_led_state() {
        auto msg = ros_robot_controller_msgs::msg::LedState();
        msg.id = 2;
        msg.on_time = 0.1;
        msg.off_time = 0.5;
        msg.repeat = 10;
        
        this->pub->publish(msg);
        RCLCPP_INFO(
            rclcpp::get_logger(""), 
            "Published LED State: id=%d, on_time=%.2f, off_time=%.2f, repeat=%d",
            msg.id,
            msg.on_time,
            msg.on_time,
            msg.repeat
        );
    }

private:
    rclcpp::Publisher<ros_robot_controller_msgs::msg::LedState>::SharedPtr pub;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin_some(std::make_shared<LEDController>());
    rclcpp::shutdown();

    return 0;
}