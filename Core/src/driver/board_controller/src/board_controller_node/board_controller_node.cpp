#include "board_controller_node.h"

BoardController::BoardController() 
    :Node("board_controller")
{
    initialize();
}

BoardController::~BoardController() {
    if (this->serial.isOpen())
        close();
}

auto BoardController::list_ports() -> void {
    const auto& devices = serial::list_ports();
    for (const auto& device : devices) {
        RCLCPP_INFO(
            rclcpp::get_logger(""),
            "Port: %s, Description: %s, Hardware ID: %s", 
            device.port.c_str(),
            device.description.c_str(),
            device.hardware_id.c_str()
        );
    }
}

auto BoardController::connect(const std::string& port) -> void {
    auto timeout = serial::Timeout::simpleTimeout(1000);

    try {
        this->serial.setPort(port);
        this->serial.setBaudrate(115200);
        this->serial.setTimeout(timeout);
        this->serial.open();
    } catch (const serial::IOException& e) {
        RCLCPP_ERROR(
            rclcpp::get_logger(""), 
            "File: %s, Line: %d, Error: %s", 
            __FILE__, 
            __LINE__, 
            e.what()
        );
        return;
    }

    if (!this->serial.isOpen())
        RCLCPP_ERROR(
            rclcpp::get_logger(""), 
            "File: %s, Line: %d, Error: Failed to open",
            __FILE__,
            __LINE__
        );
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
    for (const auto& data : vector_data) 
        RCLCPP_INFO(rclcpp::get_logger(""), "%x", data);

    this->serial.write(vector_data);

    RCLCPP_INFO(rclcpp::get_logger(""), "Transmitted.");
}

auto BoardController::receive() -> void {
    while (true) {
        const auto& packet = this->serial.readline();
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
    // Data Length
    vec.emplace_back(msg.array_data.size());
    // Data
    vec.insert(vec.end(), msg.array_data.begin(), msg.array_data.end());

    transmit(vec);
}

auto BoardController::initialize() -> void {
    // Connect to board
    connect("/dev/ttyACM0");

    // Create a thread which receiving packet from board
    this->thread_receive = std::thread(&BoardController::receive, this);

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