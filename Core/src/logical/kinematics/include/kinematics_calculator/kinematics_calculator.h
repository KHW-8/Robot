#ifndef KINEMATICS_KinematicsCalculator_NODE_H
#define KINEMATICS_KinematicsCalculator_NODE_H

//////////* Headers *//////////

// ROS
#include "std_srvs/srv/trigger.hpp"
#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>
#include <std_srvs/srv/trigger.hpp>

///////////////////////////////

//////////* Enumeration *//////////

enum class KinematicsType {
    NONE,
    FORWARD,
    INVERSE
};

///////////////////////////////////

//////////* Classes *//////////

class KinematicsCalculator : public rclcpp::Node {
public:
    KinematicsCalculator();

private:
    // Calculate
    auto calculate(KinematicsType type) -> void;

    // Callback
    auto initialization_complete([[maybe_unused]]std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> void;
    
    // Initialization
    auto initialize() -> void;

private:
    // Client
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_robot_controller;

    // Service
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_ini;
};

///////////////////////////////

#endif
