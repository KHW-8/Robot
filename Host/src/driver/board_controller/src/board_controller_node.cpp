#include "board_controller_node.hpp"


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BoardController>();
    node->list_ports();

    rclcpp::shutdown();

    return 0;
}