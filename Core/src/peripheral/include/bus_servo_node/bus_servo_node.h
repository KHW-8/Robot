//////////* Headers *//////////

/* ROS2 */
#include "rclcpp/node.hpp"
#include "std_srvs/srv/trigger.hpp"
/* Core */
#include "robot_controller_msg/msg/bus_servo_request.hpp"

///////////////////////////////

////////////////////* Class *////////////////////

class BusServoNode : rclcpp::Node {
public:
    BusServoNode();
    
public:
    void initialize();

    void read_pos(const std::vector<decltype(robot_controller_msg::msg::BusServo::id)>& ids);

    void set_pos(const decltype(robot_controller_msg::msg::BusServo::id)& id, 
                 const decltype(robot_controller_msg::msg::BusServo::angle)& angle, 
                 const decltype(robot_controller_msg::msg::BusServo::duration)& duration);

private:
    rclcpp::Publisher<robot_controller_msg::msg::BusServoRequest>::SharedPtr pub;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_robot_controller;
};

/////////////////////////////////////////////////////