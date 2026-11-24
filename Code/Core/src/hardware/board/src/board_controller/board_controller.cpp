//////////* Headers *//////////
#include "board_controller.h"

/*Core*/
#include "board_msgs/msg/packet.hpp"
#include "utility.hpp"

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
#include <chrono>
#include <rclcpp/executors.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>

/* STD */
#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <thread>
#include <utility>
#include <vector>

///////////////////////////////

///////////* Macros *///////////

// Packet
#define PACKET_HEADER 0x55

#define PACKET_BASIC_INFORMATION_SIZE 7 // bytes

#define HEADER1_INDEX 0
#define HEADER2_INDEX 1
#define REQUEST_ID_INDEX 2
#define PERIPHERAL_INDEX 4
#define CHECKSUM_INDEX 5
#define DATA_LENGTH_INDEX 6


////////////////////////////////


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
        header1 = packet.at(HEADER1_INDEX);
        header2 = packet.at(HEADER2_INDEX);
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
        chksum = packet.at(CHECKSUM_INDEX);
        data_length = packet.at(DATA_LENGTH_INDEX);
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

    if (data_length == packet.size() - PACKET_BASIC_INFORMATION_SIZE) {
        std::vector<uint8_t> data;
        data.insert(data.end(), packet.begin() + PACKET_BASIC_INFORMATION_SIZE, packet.end());

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
    } else if (data_length < packet.size() - PACKET_BASIC_INFORMATION_SIZE) {
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

auto BoardController::generate_request_id(const std::vector<uint8_t>& vector) -> uint16_t {
   size_t sum = 0;

    for (const auto& data : vector)
        sum += data;

    return (uint16_t)(sum);
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

auto BoardController::transmit(const std::vector<uint8_t>& data) -> void {
    const auto& bytes = boost::asio::write(this->serial, boost::asio::buffer(data));

    RCLCPP_INFO(rclcpp::get_logger(""), "Transmitted: %ld", bytes);

    std::stringstream stream;
    for (const auto& data : data) 
        stream << std::hex << std::showbase << (int)data << " ";
    RCLCPP_INFO(rclcpp::get_logger(""), "%s", stream.str().c_str());
}

/** 
 * @brief
 * @retval
 */
auto BoardController::receive_board_packet() -> void {
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

            if (packet.size() == PACKET_BASIC_INFORMATION_SIZE) {
                try {
                    data_length = packet.at(DATA_LENGTH_INDEX);
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


            if (packet.size() - PACKET_BASIC_INFORMATION_SIZE == data_length) {
                const auto& res = check_packet(packet);
                const auto& complete = res.first;
                const auto& haveError = res.second;
                if (complete && !haveError)  {
                    // Print packet
                    RCLCPP_INFO(rclcpp::get_logger(""), "Received: %ld", packet.size());

                    std::stringstream stream;
                    for (const auto& data : packet) 
                        stream << std::hex << std::showbase << (int)data << " ";
                    RCLCPP_INFO(rclcpp::get_logger(""), "%s", stream.str().c_str());

                    // Store packet
                    store_board_packet(packet);
                }

                start_to_receive_packet = false;
                packet.clear();
            }
        }
    }
}


auto BoardController::handle_packet(const board_msgs::msg::Packet& msg) -> void {
    // Initialize a vector with header
    std::vector<uint8_t> vec{ 0x55, 0x55 }; 
    // Request ID
    auto request_id = to_byte_vector<uint16_t>(generate_request_id(msg.data.to_vector()));
    vec.insert(vec.end(), request_id.begin(), request_id.end());
    // Peripheral ID
    vec.emplace_back(msg.peripheral_id);
    // Checksum
    vec.emplace_back(generate_checksum(msg.data.to_vector()));
    // Data Length
    vec.emplace_back(msg.data.size());
    // Data
    vec.insert(vec.end(), msg.data.begin(), msg.data.end());

    transmit(vec);
}

auto BoardController::receive_msg() -> void {
    boost::asio::streambuf stream_buf;
    boost::asio::read_until(this->serial, stream_buf, '\n');

    std::istream is(&stream_buf);
    std::string line;
    std::getline(is, line);

    std::cout << line << std::endl;
}

auto BoardController::store_board_packet(const std::vector<uint8_t>& packet) -> void {
  if (packet.empty()) 
        return;

    uint16_t request_id = 0;

    try {
        uint8_t request_id1 = packet.at(REQUEST_ID_INDEX);
        uint8_t request_id2 = packet.at(REQUEST_ID_INDEX + 1);

        request_id = request_id2 << 8 | request_id1;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(
            rclcpp::get_logger(""), 
            "File: %s, Function: %s, Line: %d, Error: %s",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            e.what()
        );    

        return;
    }

    packet_map[request_id] = packet;
}


auto BoardController::initialize() -> void {
    // Connect to board
    const auto& res = connect("/dev/ttyACM0");

    // Create a thread which receiving packet from board
    if (res)
        this->thread_receive = std::thread(&BoardController::receive_board_packet, this);
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
    this->srv_packet = this->create_service<board_msgs::srv::Packet>(
        "~/packet",
        std::bind(&BoardController::handle_request, this, std::placeholders::_1, std::placeholders::_2)
    );

    this->sub_buzzer = this->create_subscription<board_msgs::msg::Packet>(
        "/buzzer/packet",
        10,
        std::bind(&BoardController::handle_packet, this, std::placeholders::_1)
    );

    this->sub_led = this->create_subscription<board_msgs::msg::Packet>(
        "/led/packet",
        10,
        std::bind(&BoardController::handle_packet, this, std::placeholders::_1)
    );

    // Complete initialization and notify all other nodes
    this->srv_ini = this->create_service<std_srvs::srv::Trigger>(
        "~/initialization_complete", 
        std::bind(&BoardController::initialization_complete, this, std::placeholders::_1, std::placeholders::_2)
    );
}

auto BoardController::handle_request(const std::shared_ptr<board_msgs::srv::Packet::Request> request, const std::shared_ptr<board_msgs::srv::Packet::Response> response) -> void {
    // Initialize a vector with header
    std::vector<uint8_t> vec{ 0x55, 0x55 }; 
    // Request ID
    auto request_id = generate_request_id(request->data.to_vector());
    auto request_id_vector = to_byte_vector<uint16_t>(request_id);
    vec.insert(vec.end(), request_id_vector.begin(), request_id_vector.end());
    // Peripheral ID
    vec.emplace_back(request->peripheral_id);
    // Checksum
    vec.emplace_back(generate_checksum(request->data.to_vector()));
    // Data Length
    vec.emplace_back(request->data.size());
    // Data
    vec.insert(vec.end(), request->data.begin(), request->data.end());

    transmit(vec);

    if (request->tx_only) 
        return;

    // Wait for response
    auto start = std::chrono::system_clock::now();

    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        auto it = packet_map.find(request_id);
        if (it != packet_map.end()) {
            const auto& packet = it->second;
            uint8_t data_length = 0;

            try {
                data_length = packet.at(DATA_LENGTH_INDEX);
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

            response->data.insert(response->data.end(), packet.end() - data_length, packet.end());
            break;
        }

        // Wait for 10s
        const auto& duration = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start).count();
        if (duration >= 10) { // Time out
            RCLCPP_INFO(this->get_logger(), "Timeout.");
            break;
        }
    }
}


auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BoardController>());
    rclcpp::shutdown();

    return 0;
}