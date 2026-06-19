// ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

class RobotController : public rclcpp::Node {
public:
    RobotController() 
        :Node("robot_controller")
    {
        this->service = this->create_service<std_srvs::srv::Trigger>(
            "~/init_complete", 
            std::bind(&RobotController::inition_callback, this, std::placeholders::_1, std::placeholders::_2)
        );
    }
private:
    void inition_callback([[maybe_unused]]const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                          const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        response->success = true;
    }

private:
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotController>());
    rclcpp::shutdown();

    return 0;
}