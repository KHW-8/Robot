#include "rclcpp/rclcpp.hpp"
#include "servo_controller_msg/msg/servos_position.hpp"

class ArmController : public rclcpp::Node {
public:
    ArmController() 
        :Node("arm_controller")
    {
        this->sub = this->create_subscription<servo_controller_msg::msg::ServosPosition>(
            "/arm_controller/bus_servo/set_position", 
            10, 
            std::bind(&ArmController::set_bus_servo_pos, this, std::placeholders::_1)
        );
    }

private:
    void set_bus_servo_pos(servo_controller_msg::msg::ServosPosition::UniquePtr msg);

private:
    rclcpp::Subscription<servo_controller_msg::msg::ServosPosition>::SharedPtr sub;
};

void ArmController::set_bus_servo_pos(servo_controller_msg::msg::ServosPosition::UniquePtr msg) {
    for (const auto& i : msg->position) 
        RCLCPP_INFO(rclcpp::get_logger(""), "duration=%.2f, id=%d, pos=%d\n", msg->duration, i.id, i.position);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmController>());
    rclcpp::shutdown();

    return 0;
}