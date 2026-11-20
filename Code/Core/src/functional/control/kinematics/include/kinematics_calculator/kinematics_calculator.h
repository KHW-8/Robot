#ifndef KINEMATICS_CALCULATOR_H
#define KINEMATICS_CALCULATOR_H

//////////* Headers *//////////

/* Core */
#include "inverse_kinematics.h"

/* ROS */
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>
#include "kinematics_msgs/srv/pose.hpp"
#include "std_srvs/srv/trigger.hpp"

/* STD */
#include <cstdint>
#include <vector>

/* SymEngine */
#include <symengine/matrix.h>



///////////////////////////////

//////////* Enumeration *//////////

enum class KinematicsType {
    NONE,
    FORWARD,
    INVERSE
};

///////////////////////////////////

//////////* Classes *//////////

struct ServoAngleRelation {
    uint8_t servo_id;
    std::pair<double, double> servo_angle_interval;

    uint8_t theta_id;
    std::pair<double, double> theta_interval;
};

class KinematicsCalculator : public rclcpp::Node {
public:
    KinematicsCalculator();

private:
    // Calculate
    auto calculate(KinematicsType type) -> void;
    auto rpy(const SymEngine::DenseMatrix& rotation_matrix) -> void;

    // Callback
    auto receive_pose(std::shared_ptr<kinematics_msgs::srv::Pose::Request> request, std::shared_ptr<kinematics_msgs::srv::Pose::Response> response) -> void;

    auto initialization_complete([[maybe_unused]]std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> void;

    // Convert
    auto servo_angle_to_theta(uint8_t servo_id, uint8_t angle) -> std::pair<bool, double>;
    auto theta_to_servo_angle(uint8_t theta_id, double angle) -> std::pair<bool, double>;

    // Generate
    auto generate_forward_kinematics_equation() -> void;
    
    // Initialization
    auto initialize() -> void;
    auto initialize_servo_angle_relation_map() -> void;

    // Print
    auto print_solutions(const std::vector<Solution>& solutions) -> void;

private:
    // Client
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_robot_controller;

    // Service
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_ini;
    rclcpp::Service<kinematics_msgs::srv::Pose>::SharedPtr srv_pose;

    // Kinematics
    std::vector<SymEngine::DenseMatrix> transformation_operators;
    SymEngine::DenseMatrix forward_kinematics_equation;

    std::map<uint8_t, ServoAngleRelation> servo_angle_relation_map;
    std::map<uint8_t, std::pair<double, double> > theta_interval_map;
};

///////////////////////////////

#endif
