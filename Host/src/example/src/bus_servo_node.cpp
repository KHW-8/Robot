// STD
#include <chrono>
// ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
// Host
#include "robot_controller_msg/msg/servo.hpp"
#include "robot_controller_msg/msg/servos.hpp"

using namespace std::chrono_literals;

class BusServoNode : rclcpp::Node {
public:
    BusServoNode() 
        :Node("bus_servo_node")
    {
        this->pub = this->create_publisher<robot_controller_msg::msg::Servos>("/robot_controller/bus_servo", 1);

        // Waiting for robot controller node to start
        this->client = this->create_client<std_srvs::srv::Trigger>("/robot_controller/initialization_complete");
        this->client->wait_for_service();
    }
    
public:
    void set_servo(const size_t& id, const int& position, const double& duration);

private:
    rclcpp::Publisher<robot_controller_msg::msg::Servos>::SharedPtr pub;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;
};

void BusServoNode::set_servo(const size_t& id, const int&  position, const double& duration) {
    // Generate message
    auto servo = robot_controller_msg::msg::Servo();
    servo.id = id;
    servo.position = position;
    servo.duration = duration;
    
    auto msg = robot_controller_msg::msg::Servos();
    msg.servos.emplace_back(servo);

    // Send message
    this->pub->publish(msg);

    // Print log
    for (const auto& servo : msg.servos) 
        RCLCPP_INFO(rclcpp::get_logger(""), "Servo Id: %d, Position: %d°, Duration: %.2fs", servo.id, servo.position, servo.duration);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto servo_controller = std::make_shared<BusServoNode>();

    try {
        while (rclcpp::ok()) {
            servo_controller->set_servo(4, 50, 1);  // Set the position of servo 4 to 50
            std::this_thread::sleep_for(1s);        // Wait 1 sec
            servo_controller->set_servo(4, 150, 1); // Set the position of servo 4 to 150
            std::this_thread::sleep_for(1s);        // Wait 1 sec
        }
    } catch(const rclcpp::exceptions::RCLError& e) {
        RCLCPP_ERROR(
            rclcpp::get_logger(""), 
            "File: %s, Line: %d, Error: %s", 
            __FILE__, 
            __LINE__, 
            e.what()
        );
    } catch(const rclcpp::exceptions::InvalidServiceNameError& e) {
        RCLCPP_ERROR(
            rclcpp::get_logger(""), 
            "File: %s, Line: %d, Error: %s", 
            __FILE__, 
            __LINE__, 
            e.what()
        );
    } catch(...) {
         RCLCPP_ERROR(
            rclcpp::get_logger(""), 
            "File: %s, Line: %d, Error: Unknown error.",
            __FILE__,
            __LINE__
        );
    }

    rclcpp::shutdown();     // Shutdown 

    return 0;
}
