// STD
#include <chrono>
// ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
// ArmPi
#include "ros_robot_controller_msgs/msg/buzzer_state.hpp"

using namespace std::chrono_literals;

class BuzzerController : public rclcpp::Node {
public: 
    BuzzerController() 
        : Node("BuzzerController")
    {
        this->pub = this->create_publisher<ros_robot_controller_msgs::msg::BuzzerState>("/ros_robot_controller/set_buzzer", 1);

        // Waiting for robot arm underlying control services to start
        this->client = this->create_client<std_srvs::srv::Trigger>("/ros_robot_controller/init_finish");
        this->client->wait_for_service();
    }

public:
    void set_buzzer(double freq, double on_time, double off_time, size_t repeqat) {
        auto msg = ros_robot_controller_msgs::msg::BuzzerState();
        msg.freq = freq;
        msg.on_time = on_time;
        msg.off_time = off_time;
        msg.repeat = repeqat;

        // Send message
        this->pub->publish(msg);
        RCLCPP_INFO(
            rclcpp::get_logger(""), 
            "Published Buzzer State: freq=%d, on_time=%.2f, off_time=%.2f, repeat=%d",
            msg.freq,
            msg.on_time,
            msg.on_time,
            msg.repeat
        );
    }

private:
    rclcpp::Publisher<ros_robot_controller_msgs::msg::BuzzerState>::SharedPtr pub;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // Send buzzer state
    auto controller = std::make_shared<BuzzerController>();
    std::this_thread::sleep_for(5s);
    controller->set_buzzer(1500, 0.1, 0.5, 10);
    controller.reset(); // Clear node

    rclcpp::shutdown(); // Shutdown

    return 0;
}