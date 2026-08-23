#include "buzzer_node.h"

BuzzerNode::BuzzerNode() 
    : Node("buzzer")
{
    this->pub = this->create_publisher<robot_controller_msg::msg::BuzzerRequest>("/robot_controller/buzzer", 1);

    // Waiting for robot controller node to start
    this->client = this->create_client<std_srvs::srv::Trigger>("/robot_controller/initialization_complete");
    this->client->wait_for_service();
}

void BuzzerNode::set_state(decltype(robot_controller_msg::msg::BuzzerRequest::frequency) freq, 
                decltype(robot_controller_msg::msg::BuzzerRequest::on_duration) on_duration, 
                decltype(robot_controller_msg::msg::BuzzerRequest::off_duration) off_duration, 
                decltype(robot_controller_msg::msg::BuzzerRequest::repeat_count) repeat_count) 
{
    auto msg = robot_controller_msg::msg::BuzzerRequest();
    msg.frequency = freq;
    msg.on_duration = on_duration;
    msg.off_duration = off_duration;
    msg.repeat_count= repeat_count;

    // Send message
    this->pub->publish(msg);
    RCLCPP_INFO(
        rclcpp::get_logger(""), 
        "Published Buzzer State: freq=%d, on_duration=%.2f, off_duration=%.2f, repeat=%d",
        msg.frequency,
        msg.on_duration,
        msg.off_duration,
        msg.repeat_count
    );
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // Send buzzer state
    const auto& node = std::make_shared<BuzzerNode>();
    node->set_state(1500, 0.1, 0.5, 10);

    rclcpp::shutdown(); // Shutdown

    return 0;
}