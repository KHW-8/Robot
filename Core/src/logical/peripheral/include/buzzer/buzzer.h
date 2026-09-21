// ROS2
#include <rclcpp/node.hpp>
#include <std_srvs/srv/trigger.hpp>
// Robot Controller Message
#include "peripheral_msg/msg/buzzer_request.hpp"

class Buzzer : public rclcpp::Node {
public: 
    Buzzer();

public:
    void set_state(decltype(peripheral_msg::msg::BuzzerRequest::frequency) freq, 
                   decltype(peripheral_msg::msg::BuzzerRequest::on_duration) on_duration, 
                   decltype(peripheral_msg::msg::BuzzerRequest::off_duration) off_duration, 
                   decltype(peripheral_msg::msg::BuzzerRequest::repeat_count) repeat_count);

private:
    rclcpp::Publisher<peripheral_msg::msg::BuzzerRequest>::SharedPtr pub;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;
};