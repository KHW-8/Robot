#ifndef KINEMATICS_KinematicsCalculator_NODE_H
#define KINEMATICS_KinematicsCalculator_NODE_H

//////////* Headers *//////////

/* ROS */
#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <symengine/matrix.h>
#include <vector>
/* SymEngine */

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
    auto rpy(const SymEngine::DenseMatrix& rotation_matrix) -> void;

    // Callback
    auto initialization_complete([[maybe_unused]]std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> void;

    // Generate
    auto generate_forward_kinematics_equation() -> void;
    
    // Initialization
    auto initialize() -> void;

private:
    // Client
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_robot_controller;

    // Service
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_ini;

    // Kinematics
    std::vector<SymEngine::DenseMatrix> transformation_operators;
    SymEngine::DenseMatrix forward_kinematics_equation;
};

///////////////////////////////

#endif
