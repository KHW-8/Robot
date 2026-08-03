#include "board_controller_node.h"

BoardController::BoardController() 
    :Node("board_controller")
{
    list_ports();

    ini_srv = this->create_service<std_srvs::srv::Trigger>(
        "~/initialization_complete", 
        std::bind(&BoardController::initialization_complete, this, std::placeholders::_1, std::placeholders::_2)
    );
}

BoardController::~BoardController() {
    if (serial.isOpen())
        close();
}

auto BoardController::list_ports() -> void {
    auto devices = serial::list_ports();
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
        serial.setPort(port);
        serial.setBaudrate(115200);
        serial.setTimeout(timeout);
        serial.open();
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

    if (!serial.isOpen())
        RCLCPP_ERROR(
            rclcpp::get_logger(""), 
            "File: %s, Line: %d, Error: Failed to open",
            __FILE__,
            __LINE__
        );
}

auto BoardController::close() -> void {
    serial.close();
}

auto BoardController::initialization_complete([[maybe_unused]]const std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
                             const std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> void 
{
    response.get()->success = true;
}


auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BoardController>());
    rclcpp::shutdown();

    return 0;
}