// STD
#include <chrono>
// ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
// ArmPi
#include "servo_controller_msg/msg/servo_position.hpp"
#include "servo_controller_msg/msg/servos_position.hpp"

using namespace std::chrono_literals;

class ServoController : rclcpp::Node {
public:
    ServoController() 
        :Node("servo_controller")
    {
        this->pub = this->create_publisher<servo_controller_msg::msg::ServosPosition>("/arm_controller/bus_servo/set_position", 1);

        // Waiting for robot arm underlying control services to start
        // this->client = this->create_client<std_srvs::srv::Trigger>("/arm_controller/init_finish");
        // this->client->wait_for_service();
    }
    
public:
    void set_servo_position(const size_t& duration, std::vector<std::pair<size_t, size_t> > positions);

private:
    rclcpp::Publisher<servo_controller_msg::msg::ServosPosition>::SharedPtr pub;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;
};

void ServoController::set_servo_position(const size_t& duration, 
    std::vector<std::pair<size_t, size_t> > positions) 
{
    // Generate message
    auto msg = servo_controller_msg::msg::ServosPosition();
    msg.duration = duration;
    for (const auto& i : positions) {
        auto position = servo_controller_msg::msg::ServoPosition();
        position.id = i.first;
        position.position = i.second;

        msg.position.emplace_back(position);
    }

    // Send message
    this->pub->publish(msg);

    // Print log
    for (const auto& i : msg.position) 
        RCLCPP_INFO(rclcpp::get_logger(""), "duration=%.2f, id=%d, position=%d", msg.duration, i.id, i.position);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto arm_controller = std::make_shared<ServoController>();

    try {
        while (rclcpp::ok()) {
            arm_controller->set_servo_position(1, { std::make_pair(4, 300) });  // Set the position of Servo 4 to 300
            std::this_thread::sleep_for(1s);                                // Wait 1 sec
            arm_controller->set_servo_position(1, { std::make_pair(4, 600) });  // Set the position of Servo 4 to 600
            std::this_thread::sleep_for(1s);                                // Wait 1 sec
        }
    } catch(const rclcpp::exceptions::RCLError& e) {
        RCLCPP_ERROR(rclcpp::get_logger(""), "rclcpp::exceptions::RCLError: %s", e.what());
    } catch(const rclcpp::exceptions::InvalidServiceNameError& e) {
        RCLCPP_ERROR(rclcpp::get_logger(""), "rclcpp::exceptions::InvalidServiceNameError: %s", e.what());
    } catch(...) {
        RCLCPP_ERROR(rclcpp::get_logger(""), "Unknown error.");
    }

    arm_controller.reset();     // Clear node
    rclcpp::shutdown();     // Shutdown 

    return 0;
}
