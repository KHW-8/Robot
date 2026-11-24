//////////* Headers *//////////

#include "board_controller.h"

/* Boost */
#include <boost/asio.hpp>
#include <boost/asio/impl/read.hpp>
#include <boost/asio/impl/read_until.hpp>
#include <boost/asio/registered_buffer.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/asio/serial_port_base.hpp>
#include <boost/system/detail/error_code.hpp>
#include <boost/throw_exception.hpp>

/* ROS2 */
#include <rclcpp/executors.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>

/* STD */
#include <cstddef>
#include <cstdint>
#include <exception>
#include <utility>
#include <vector>

///////////////////////////////

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

    if (this->thread_receive.joinable())
        this->thread_receive.join();
}

auto BoardController::connect(const std::string& port) -> bool {
    try {
        this->serial.open(port);

        this->serial.set_option(boost::asio::serial_port_base::character_size(8));
        this->serial.set_option(boost::asio::serial_port_base::baud_rate(115200));
        this->serial.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
        this->serial.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        this->serial.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
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
        RCLCPP_ERROR(
            rclcpp::get_logger(""), 
            "File: %s, Line: %d, Error: Unknown error.", 
            __FILE__, 
            __LINE__
        );

        return false;
    }

    if (!this->serial.is_open()) {
        RCLCPP_ERROR(
            rclcpp::get_logger(""), 
            "File: %s, Function: %s, Line: %d, Error: Failed to open",
            __FILE__,
            __FUNCTION__,
            __LINE__
        );
    }

    return true;
}

auto BoardController::close() -> void {
    this->serial.close();
}

auto BoardController::check_packet(const std::vector<uint8_t>& packet) -> std::pair<bool, bool> {
    if (packet.empty())
        return {false, false};

    uint8_t header1 = 0;
    uint8_t header2 = 0;

    try {
        header1 = packet.at(0);
        header2 = packet.at(1);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(
            rclcpp::get_logger(""), 
            "File: %s, Function: %s, Line: %d, Error: %s",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            e.what()
        );
    }

    if (header1 != PACKET_HEADER || header2 != PACKET_HEADER) {
        RCLCPP_ERROR(
            rclcpp::get_logger(""), 
            "File: %s, Function: %s, Line: %d, Error: Header error.",
            __FILE__,
            __FUNCTION__,
            __LINE__
        );

        return {true, true};
    }

    uint8_t chksum = 0;
    uint8_t data_length = 0;

    try {
        chksum = packet.at(3);
        data_length = packet.at(4);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(
            rclcpp::get_logger(""), 
            "File: %s, Function: %s, Line: %d, Error: %s",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            e.what()
        );
    }

    if (data_length == packet.size() - 5) {
        std::vector<uint8_t> data;
        data.insert(data.end(), packet.begin() + 5, packet.end());

        if (chksum == generate_checksum(data))
            return { true, false };
        else {
            RCLCPP_ERROR(
                rclcpp::get_logger(""), 
                "File: %s, Function: %s, Line: %d, Error: Checksum is uncorrect.",
                __FILE__,
                __FUNCTION__,
                __LINE__
            );

            return { true, true };
        }
    } else if (data_length < packet.size() - 5) {
        RCLCPP_ERROR(
            rclcpp::get_logger(""), 
            "File: %s, Function: %s, Line: %d, Error: Packet length error.",
            __FILE__,
            __FUNCTION__,
            __LINE__
        );

        return { true, true };
    } else
        return { false, false };
}


auto BoardController::generate_checksum(const std::vector<uint8_t>& vector) -> uint8_t {
    size_t sum = 0;

    for (const auto& data : vector)
        sum += data;

    return (uint8_t)(~sum);
}


auto BoardController::initialization_complete([[maybe_unused]]const std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
                                              const std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> void 
{
    response->success = true;
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
    std::vector<uint8_t> packet;
    uint8_t byte = 0;
    size_t data_length = 0;
    bool start_to_receive_packet = false;

    while (rclcpp::ok()) {
        boost::asio::read(this->serial, boost::asio::buffer(&byte, 1));

        if (byte == PACKET_HEADER && !start_to_receive_packet) 
            start_to_receive_packet = true;

        if (start_to_receive_packet) { 
            packet.emplace_back(byte);

            if (packet.size() == 5) {
                try {
                    data_length = packet.at(4);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(
                        rclcpp::get_logger(""),
                        "File: %s, Function: %s, Line: %d, Error: Packet length error.",
                        __FILE__,
                        __FUNCTION__,
                        __LINE__
                    );

                    start_to_receive_packet = false;
                    packet.clear();

                    continue;
                }
            }
    
            if (packet.size() - 5 == data_length) {
                const auto& res = check_packet(packet);
                const auto& complete = res.first;
                const auto& haveError = res.second;
                if (complete && !haveError) 
                    handle_packet(packet);

                start_to_receive_packet = false;
                packet.clear();
            }
        }
    }
}


auto BoardController::receive_packet(const board_msgs::msg::Packet& msg) -> void {
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

auto BoardController::receive_packet_from_board() -> void {

}

auto BoardController::receive_msg_from_board() -> void {
    boost::asio::streambuf stream_buf;
    boost::asio::read_until(this->serial, stream_buf, '\n');

    std::istream is(&stream_buf);
    std::string line;
    std::getline(is, line);

    std::cout << line << std::endl;
}

auto BoardController::initialize() -> void {
    // Connect to board
    const auto& res = connect("/dev/ttyACM0");

    // Create a thread which receiving packet from board
    if (res)
        this->thread_receive = std::thread(&BoardController::receive, this);
    else {
        RCLCPP_ERROR(
            rclcpp::get_logger(""), 
            "File: %s, Line: %d, Error: Connection error.", 
            __FILE__, 
            __LINE__
        );

        return;
    }

    // Create subscription of packet
    this->sub_bus_servo = this->create_subscription<board_msgs::msg::Packet>(
        "/bus_servo/packet",
        10,
        std::bind(&BoardController::receive_packet, this, std::placeholders::_1)
    );

    this->sub_buzzer = this->create_subscription<board_msgs::msg::Packet>(
        "/buzzer/packet",
        10,
        std::bind(&BoardController::receive_packet, this, std::placeholders::_1)
    );

    this->sub_led = this->create_subscription<board_msgs::msg::Packet>(
        "/led/packet",
        10,
        std::bind(&BoardController::receive_packet, this, std::placeholders::_1)
    );

    // Complete initialization and notify all other nodes
    this->srv_ini = this->create_service<std_srvs::srv::Trigger>(
        "~/initialization_complete", 
        std::bind(&BoardController::initialization_complete, this, std::placeholders::_1, std::placeholders::_2)
    );
}

auto BoardController::handle_packet(const std::vector<uint8_t>& packet) -> void {
    if (packet.empty()) 
        return;

    for (auto e : packet)
        RCLCPP_INFO(rclcpp::get_logger(""), "%x", e);
}


auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BoardController>());
    rclcpp::shutdown();

    return 0;
}