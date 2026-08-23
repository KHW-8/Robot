// ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
// LED Node
#include "robot_controller_msg/msg/led_request.hpp"

class LEDNode : public rclcpp::Node {
public: 
    LEDNode();

public:
    void set_state(decltype(robot_controller_msg::msg::LED::id) led_id, 
                   decltype(robot_controller_msg::msg::LED::on_duration) on_duration, 
                   decltype(robot_controller_msg::msg::LED::off_duration) off_duration, 
                   decltype(robot_controller_msg::msg::LED::repeat_count) repeat_count);

private:
    rclcpp::Publisher<robot_controller_msg::msg::LEDRequest>::SharedPtr pub;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_robot_controller;
};