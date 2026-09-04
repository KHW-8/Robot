#include "board_controller_node.h"

// Boost
#include "boost/asio.hpp"
#include "boost/asio/streambuf.hpp"
#include "boost/asio/serial_port_base.hpp"
#include "boost/system/system_error.hpp"
#include "boost/throw_exception.hpp"
// ROS2
#include "rclcpp/executors.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
// STD
#include <istream>
#include <string>
//
#include "utility.hpp"

BoardController::BoardController() 
    :Node("board_controller"),
    serial(io)
{
    this->serial = boost::asio::serial_port(io);

    initialize();
}

BoardController::~BoardController() {
    if (this->serial.is_open())
        close();
}

auto BoardController::connect(const std::string& port) -> bool {
    try {
        this->serial.open(port);

        using namespace boost::asio;
        this->serial.set_option(serial_port_base::character_size(8));
        this->serial.set_option(serial_port_base::baud_rate(115200));
        this->serial.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
        this->serial.set_option(serial_port_base::parity(serial_port_base::parity::none));
        this->serial.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
    } catch (const boost::wrapexcept<boost::system::system_error>& e) {
        RCLCPP_ERROR(
            rclcpp::get_logger(""), 
            "File: %s, Line: %d, Error: %s", 
            __FILE__, 
            __LINE__, 
            e.what()
        );

        return false;
    } catch (...) {
        RCLCPP_ERROR(rclcpp::get_logger(""), "Unknown error!");

        return false;
    }

    if (!this->serial.is_open()) {
        RCLCPP_ERROR(
            rclcpp::get_logger(""), 
            "File: %s, Line: %d, Error: Failed to open",
            __FILE__,
            __LINE__
        );
    }

    return true;
}

auto BoardController::close() -> void {
    this->serial.close();
}

auto BoardController::initialization_complete([[maybe_unused]]const std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
                                              const std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> void 
{
    response.get()->success = true;
}

auto BoardController::transmit(const std::vector<uint8_t>& vector_data) -> void {
    std::stringstream stream;
    for (const auto& data : vector_data) 
        stream << std::hex << std::showbase << (int)data << " ";
    RCLCPP_INFO(rclcpp::get_logger(""), "%s", stream.str().c_str());

    const auto& bytes = boost::asio::write(this->serial, boost::asio::buffer(vector_data));

    RCLCPP_INFO(rclcpp::get_logger(""), "Transmitted: %ld", bytes);
}

/** 
 * @brief
 * @retval
 */
auto BoardController::receive() -> void {
    boost::asio::streambuf streambuf;

    while (true) {
        boost::asio::read_until(this->serial, streambuf, "\n");

        std::istream is(&streambuf);
        std::string packet;
        std::getline(is, packet);

        if (packet.empty())
            continue;;

        RCLCPP_INFO(rclcpp::get_logger(""), "%s", packet.c_str());
    }
}


auto BoardController::receive_packet(const board_controller_msg::msg::Packet& msg) -> void {
    // Initialize a vector with header
    std::vector<uint8_t> vec{ 0x55, 0x55 }; 
    // Peripheral ID
    vec.emplace_back(msg.peripheral_id);
    // Checksum
    vec.emplace_back(generate_checksum(msg.array_data.to_vector()));
    // Data Length
    vec.emplace_back(msg.array_data.size());
    // Data
    vec.insert(vec.end(), msg.array_data.begin(), msg.array_data.end());

    transmit(vec);
}

auto BoardController::initialize() -> void {
    // Connect to board
    const auto& res = connect("/dev/ttyACM0");

    // Create a thread which receiving packet from board
    if (res)
        this->thread_receive = std::thread(&BoardController::receive, this);
    else {
        RCLCPP_ERROR(rclcpp::get_logger(""), "Connection error!");
        return;
    }

    // Create subscription of packet
    this->sub_packet = this->create_subscription<board_controller_msg::msg::Packet>(
        "/board_controller/packet",
        10,
        std::bind(&BoardController::receive_packet, this, std::placeholders::_1)
    );

    // Complete initialization and notify all other nodes
    this->srv_ini = this->create_service<std_srvs::srv::Trigger>(
        "~/initialization_complete", 
        std::bind(&BoardController::initialization_complete, this, std::placeholders::_1, std::placeholders::_2)
    );
}

auto BoardController::listen() -> void {
    // Create a thread 
    this->thread_receive = std::thread(&BoardController::receive, this);
}

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BoardController>());
    rclcpp::shutdown();

    return 0;
}