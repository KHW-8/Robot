#include "bus_servo.h"

//////////* Headers *//////////

/* STD */
#include <functional>

/* ROS */
#include <rclcpp/executors.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

/* Core */
//// Global
#include "utility.hpp"
//// Physical
#include "peripheral.h"

///////////////////////////////

////////////////////* Global Variables *////////////////////


////////////////////////////////////////////////////////////


////////////////////* Functions *////////////////////

BusServo::BusServo() 
    :Node("bus_servo")
{
    initialize();
}

void BusServo::initialize() {
    // Waiting for board controller node to start
    this->cli_board_controller = this->create_client<std_srvs::srv::Trigger>("/board_controller/initialization_complete");
    this->cli_board_controller->wait_for_service();

    // Create a publisher of board controller
    this->pub_board_controller = this->create_publisher<board_msg::msg::Packet>("~/packet", 10);

    // Create a subscription of robot controller
    this->sub_robot_controller = this->create_subscription<peripheral_msg::msg::BusServoRequest>(
        "~", 
        10, 
        std::bind(&BusServo::set_pos, this, std::placeholders::_1)
    );

    // Complete initialization and notify all other nodes
    this->srv_ini = this->create_service<std_srvs::srv::Trigger>(
        "~/initialization_complete", 
        std::bind(&BusServo::initialization_complete, this, std::placeholders::_1, std::placeholders::_2)
    );
}

auto BusServo::set_pos(const peripheral_msg::msg::BusServoRequest::UniquePtr& request) -> void {
    // Print request info 
    for (const auto& servo : request->servos) {
        RCLCPP_INFO(
            rclcpp::get_logger(""), 
            "Servo Id: %d, angle: %d°, Duration: %.2fs", 
            servo.id,
            servo.angle,
            servo.duration
        );
    }

    // Create a board RobotController packet
    auto packet = board_msg::msg::Packet();

    // Peripheral ID
    packet.peripheral_id = static_cast<uint8_t>(Peripheral::BUS_SERVO);
    // Command
    packet.array_data.emplace_back(request->cmd);
    // Bus servo count
    packet.array_data.emplace_back(request->servos.size());
    // Bus servo information
    if (request->query_only) {
        for (const auto& servo : request->servos) 
            // Servo ID
            packet.array_data.emplace_back(servo.id);
    } else {
        for (const auto& servo : request->servos) {
            const auto& duration = to_byte_vector<decltype(servo.duration)>(servo.duration); // Convert second to millisecond
            
            // Servo ID
            packet.array_data.emplace_back(servo.id);
            // Angle
            packet.array_data.emplace_back(servo.angle); 
            // Duration
            packet.array_data.insert(packet.array_data.end(), duration.begin(), duration.end()); 

        }
    }

    this->pub_board_controller->publish(packet);
}

auto BusServo::initialization_complete([[maybe_unused]]const std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
                             const std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> void
{
    response->success = true;
}



int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BusServo>());
    rclcpp::shutdown();     // Shutdown 

    return 0;
}

/////////////////////////////////////////////////////