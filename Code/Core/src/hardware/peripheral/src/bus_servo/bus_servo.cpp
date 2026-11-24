//////////* Headers *//////////

#include "bus_servo.h"

/* Core */
//// Hardware
/// Global
#include "peripheral.h"
#include "utility.hpp"
/// Interfaces
#include "board_msgs/srv/packet.hpp"
#include "peripheral_msgs/msg/bus_servo.hpp"
#include "peripheral_msgs/srv/bus_servo_request.hpp"

/* ROS */
#include <rclcpp/client.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

/* STD */
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <rclcpp/qos.hpp>
#include <vector>

///////////////////////////////

///////////* Macros *///////////

#define COMMAND_INDEX 0
#define RESULT_INDEX 1
#define SERVO_COUNT_INDEX 2

////////////////////////////////

////////////////////* Global Variables *////////////////////

////////////////////////////////////////////////////////////


////////////////////* Functions *////////////////////

BusServo::BusServo() 
    :Node("bus_servo")
{
    initialize();
}

void BusServo::initialize() {
    // Create callback group
    this->cli_board_controller_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Waiting for board controller node to start
    this->cli_board_controller_ready = this->create_client<std_srvs::srv::Trigger>("/board_controller/initialization_complete");
    this->cli_board_controller_ready->wait_for_service();

    this->cli_board_controller = this->create_client<board_msgs::srv::Packet>(
        "/board_controller/packet",
        rclcpp::ServicesQoS(),
        this->cli_board_controller_cb_group
    );

    // Create request service
    this->srv_request = this->create_service<peripheral_msgs::srv::BusServoRequest>(
        "~/request", 
        std::bind(&BusServo::handle_request, this, std::placeholders::_1, std::placeholders::_2)
    );

    // Complete initialization and notify all other nodes
    this->srv_ini = this->create_service<std_srvs::srv::Trigger>(
        "~/initialization_complete", 
        std::bind(&BusServo::initialization_complete, this, std::placeholders::_1, std::placeholders::_2)
    );
}

auto BusServo::handle_packet(const std::vector<uint8_t>& packet) -> std::vector<peripheral_msgs::msg::BusServo> {
    if (packet.empty())
        return {};

    uint8_t cmd = 0;
    uint8_t res = 0;
    uint8_t servo_count = 0;

    try {
        cmd = packet.at(COMMAND_INDEX);
        res = packet.at(RESULT_INDEX);
        servo_count = packet.at(SERVO_COUNT_INDEX);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(
            rclcpp::get_logger(""), 
            "File: %s, Function: %s, Line: %d, Error: %s",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            e.what()
        );   
        
        return {};
    }

    if (res != RequestResult::OK) 
        return {};

    std::vector<peripheral_msgs::msg::BusServo> servos;

    switch (static_cast<BusServoCMD>(cmd)) {
    case BusServoCMD::READ_ANGLE: {
        for (size_t i = 0; i < servo_count; i++) {
            uint8_t servo_id = 0;
            uint8_t angle = 0;

            try {
                servo_id = packet.at(3 + 2*i);
                angle = packet.at(3 + 2*i + 1);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(
                    rclcpp::get_logger(""), 
                    "File: %s, Function: %s, Line: %d, Error: %s",
                    __FILE__,
                    __FUNCTION__,
                    __LINE__,
                    e.what()
                );   
        
                return servos;
            }

            peripheral_msgs::msg::BusServo servo;
            servo.id = servo_id;
            servo.angle = angle;

            servos.emplace_back(servo);
        }
    } break;
    default: break;
    }

    return servos;
}

auto BusServo::read_pos(const std::vector<peripheral_msgs::msg::BusServo::_id_type>& servo_ids) -> std::vector<peripheral_msgs::msg::BusServo> {
    // Print information
    for (const auto& id : servo_ids) 
        RCLCPP_INFO(rclcpp::get_logger(""), "Servo Id: %d", id);

    // Create a request
    auto request = std::make_shared<board_msgs::srv::Packet::Request>();
    // Transfer only flag
    request->tx_only = false;
    // Peripheral ID
    request->peripheral_id = static_cast<uint8_t>(Peripheral::BUS_SERVO);
    // Command
    request->data.emplace_back(static_cast<uint8_t>(BusServoCMD::READ_ANGLE));
    // Bus servo count
    request->data.emplace_back(servo_ids.size());
    // Bus servo information
    for (const auto& id : servo_ids) 
        request->data.emplace_back(id); // Servo ID
    
    
    // Send request
    auto future = this->cli_board_controller->async_send_request(request);

    future.wait_for(std::chrono::seconds(10));

    auto response = future.get();
    return handle_packet(response->data);
}

auto BusServo::set_pos(const std::vector<peripheral_msgs::msg::BusServo>& servos) -> void {
    // Print request info 
    for (const auto& servo : servos) {
        RCLCPP_INFO(
            rclcpp::get_logger(""), 
            "Servo Id: %d, angle: %d°, Duration: %.2fs", 
            servo.id,
            servo.angle,
            servo.duration
        );
    }

   // Create a request
    auto request = std::make_shared<board_msgs::srv::Packet::Request>();
    // Transfer only flag
    request->tx_only = true;
    // Peripheral ID
    request->peripheral_id = static_cast<uint8_t>(Peripheral::BUS_SERVO);
    // Command
    request->data.emplace_back(static_cast<uint8_t>(BusServoCMD::SET_ROTAION_ANGLE_AND_DURATION));
    // Bus servo count
    request->data.emplace_back(servos.size());
    // Bus servo information
    for (const auto& servo : servos) {
        const auto& duration = to_byte_vector<decltype(servo.duration)>(servo.duration); // Convert second to millisecond
        
        // Servo ID
        request->data.emplace_back(servo.id);
        // Angle
        request->data.emplace_back(servo.angle); 
        // Duration
        request->data.insert(request->data.end(), duration.begin(), duration.end()); 

    }

    // Send request
    this->cli_board_controller->async_send_request(request);
}

auto BusServo::initialization_complete([[maybe_unused]]const std_srvs::srv::Trigger::Request::SharedPtr& request, 
                                       const std_srvs::srv::Trigger::Response::SharedPtr& response) -> void
{
    response->success = true;
}

auto BusServo::handle_request(const peripheral_msgs::srv::BusServoRequest::Request::SharedPtr& request, 
                              const peripheral_msgs::srv::BusServoRequest::Response::SharedPtr& response) -> void 
{
    switch (static_cast<BusServoCMD>(request->cmd)) {
    case BusServoCMD::SET_ROTAION_ANGLE_AND_DURATION: {
        set_pos(request->servos);
    } break;
    case BusServoCMD::READ_ANGLE: {
        std::vector<uint8_t> servo_ids;
        for (const auto& servo : request->servos) 
            servo_ids.emplace_back(servo.id);

        response->cmd = request->cmd;
        response->servos = read_pos(servo_ids);
    } break;
    default: break;
    }
}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<BusServo>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();     // Shutdown 

    return 0;
}

/////////////////////////////////////////////////////