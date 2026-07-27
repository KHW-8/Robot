// STD
#include <chrono>
// ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
// Host
#include "robot_controller_msg/msg/buzzer.hpp"

using namespace std::chrono_literals;

class BuzzerNode : public rclcpp::Node {
public: 
    BuzzerNode() 
        : Node("buzzer_node")
    {
        this->pub = this->create_publisher<robot_controller_msg::msg::Buzzer>("/robot_controller/buzzer/set", 1);

        // Waiting for robot controller node to start
        this->client = this->create_client<std_srvs::srv::Trigger>("/robot_controller/initialization_complete");
        this->client->wait_for_service();
    }

public:
    void set_buzzer(double freq, double on_duration, double off_duration, size_t repeqat) {
        auto msg = robot_controller_msg::msg::Buzzer();
        msg.freq = freq;
        msg.on_duration = on_duration;
        msg.off_duration = off_duration;
        msg.repeat_count= repeqat;

        // Send message
        this->pub->publish(msg);
        RCLCPP_INFO(
            rclcpp::get_logger(""), 
            "Published Buzzer State: freq=%d, on_duration=%.2f, off_duration=%.2f, repeat=%d",
            msg.freq,
            msg.on_duration,
            msg.off_duration,
            msg.repeat_count
        );
    }

private:
    rclcpp::Publisher<robot_controller_msg::msg::Buzzer>::SharedPtr pub;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // Send buzzer state
    auto controller = std::make_shared<BuzzerNode>();
    std::this_thread::sleep_for(5s);
    controller->set_buzzer(1500, 0.1, 0.5, 10);

    rclcpp::shutdown(); // Shutdown

    return 0;
}