#include "bus_servo_node.h"
#include "bus_servo.h"
#include <vector>

//////////* Headers *//////////

/* STD */

/////////////////////////////////////////////////////

///////////////////////////////

////////////////////* Functions *////////////////////

using namespace std::chrono_literals;


////////////////////* Functions *////////////////////

BusServoNode::BusServoNode() 
    :Node("bus_servo")
{
    initialize();
}

void BusServoNode::initialize() {
    this->pub = this->create_publisher<robot_controller_msg::msg::BusServoRequest>("/robot_controller/bus_servo", 1);

    // Waiting for robot controller node to start
    this->cli_robot_controller = this->create_client<std_srvs::srv::Trigger>("/robot_controller/initialization_complete");
    this->cli_robot_controller->wait_for_service();
}

void BusServoNode::read_pos(const std::vector<decltype(robot_controller_msg::msg::BusServo::id)>& ids) {
    // Create request
    auto request = robot_controller_msg::msg::BusServoRequest();
    request.cmd = static_cast<uint8_t>(BusServoCMD::READ_ANGLE);
    request.query_only = true;

    for (const auto& id : ids) {
        // Generate servo message
        auto servo = robot_controller_msg::msg::BusServo();
        servo.id = id;

        request.servos.emplace_back(servo);
    }

    this->pub->publish(request);

    // Print log
    for (const auto& servo : request.servos) 
        RCLCPP_INFO(rclcpp::get_logger(""), "Servo Id: %d", servo.id);
}

void BusServoNode::set_pos(const decltype(robot_controller_msg::msg::BusServo::id)& id, 
                const decltype(robot_controller_msg::msg::BusServo::angle)& angle, 
                const decltype(robot_controller_msg::msg::BusServo::duration)& duration) 
{
    // Generate servo message
    auto servo = robot_controller_msg::msg::BusServo();
    servo.id = id;
    servo.angle = angle;
    servo.duration = duration;

    auto request = robot_controller_msg::msg::BusServoRequest();
    request.cmd = static_cast<uint8_t>(BusServoCMD::SET_ROTAION_ANGLE_AND_DURATION);
    request.query_only = false;
    request.servos.emplace_back(servo);

    // Send message
    this->pub->publish(request);

    // Print log
    for (const auto& servo : request.servos) 
        RCLCPP_INFO(rclcpp::get_logger(""), "Servo Id: %d, Angle: %d°, Duration: %.2fs", servo.id, servo.angle, servo.duration);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    const auto& bus_servo_node = std::make_shared<BusServoNode>();

    try {
        while (rclcpp::ok()) {
            bus_servo_node->set_pos(4, 50, 1);  // Set the angle of servo 4 to 50
            std::this_thread::sleep_for(2s);        // Wait 1 sec
            bus_servo_node->set_pos(4, 150, 1); // Set the angle of servo 4 to 150
            std::this_thread::sleep_for(2s);        // Wait 1 sec
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

/////////////////////////////////////////////////////