#include "robot_controller_node.h"

auto RobotController::initialization_callback([[maybe_unused]]const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                              const std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> void 
{
    response->success = true;
}

auto RobotController::set_bus_servo_position(robot_controller_msg::msg::Servos::UniquePtr msg) -> void {
    for (auto& servo : msg->servos) {
        RCLCPP_INFO(
            rclcpp::get_logger(""), 
            "Servo Id: %d, Position: %d°, Duration: %.2fs", 
            servo.id,
            servo.position,
            servo.duration
        );
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotController>());
    rclcpp::shutdown();

    return 0;
}