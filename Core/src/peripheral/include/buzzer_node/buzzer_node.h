// STD
#include <chrono>
// ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
// Buzzer Node
#include "robot_controller_msg/msg/buzzer_request.hpp"

using namespace std::chrono_literals;

class BuzzerNode : public rclcpp::Node {
public: 
    BuzzerNode();

public:
    void set_state(decltype(robot_controller_msg::msg::BuzzerRequest::frequency) freq, 
                   decltype(robot_controller_msg::msg::BuzzerRequest::on_duration) on_duration, 
                   decltype(robot_controller_msg::msg::BuzzerRequest::off_duration) off_duration, 
                   decltype(robot_controller_msg::msg::BuzzerRequest::repeat_count) repeat_count);

private:
    rclcpp::Publisher<robot_controller_msg::msg::BuzzerRequest>::SharedPtr pub;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;
};