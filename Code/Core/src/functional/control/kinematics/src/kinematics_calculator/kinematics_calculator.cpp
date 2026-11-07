#include "kinematics_calculator.h"

//////////* Headers *//////////

/* Core */
//// Kinematics
#include "forward_kinematics.h"
#include "inverse_kinematics.h"
//// Global
#include "kinematics_msgs/msg/transformation.hpp"
#include "math.h"
#include "utility.hpp"

/* ROS */
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <rclcpp/executors.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <std_srvs/srv/trigger.hpp>

/* STD */
#include <cmath>
#include <functional>

/* SymEngine */
#include <symengine/basic.h>
#include <symengine/matrix.h>
#include <symengine/real_double.h>
#include <symengine/symbol.h>

///////////////////////////////

//////////* Global Variables *//////////

auto alpha1 = deg_to_rad(-90);
auto alpha4 = deg_to_rad(90);

auto perpendicular_distance_between_link2_joint_z_axes = 0.101;
auto perpendicular_distance_between_link3_joint_z_axes = 0.095;

////////////////////////////////////////

//////////* Functions *//////////

KinematicsCalculator::KinematicsCalculator() 
    : Node("kinematics_calculator")
{
    initialize();

    calculate(KinematicsType::FORWARD);
}

auto KinematicsCalculator::receive_transformation_from_vision(const kinematics_msgs::msg::Transformation& msg) {
    InverseKinematics::calculate(
        Eigen::Matrix<double, 4, 4, Eigen::RowMajor>(msg.transformation.data()), 
        this->theta_interval_map
    );
}


auto KinematicsCalculator::initialization_complete([[maybe_unused]]std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
                                                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> void 
{
    response->success = true;
}


auto KinematicsCalculator::servo_angle_to_theta(uint8_t servo_id, uint8_t angle) -> std::pair<bool, double> {
    auto it = this->servo_angle_relation_map.find(servo_id);
    if (it == this->servo_angle_relation_map.end())
        return { false, 0};

    auto servo_angle_interval = it->second.servo_angle_interval;
    auto theta_interval = it->second.theta_interval;

    double offset = 0;

    // Compare left and right boundaries, then calculate offset
    if (servo_angle_interval.first < servo_angle_interval.second) {
        // Check whether the angle's value is within the range
        if (angle < servo_angle_interval.first || angle > servo_angle_interval.second)
            return { false, 0 };

        offset = angle - servo_angle_interval.first;
    } else {
        // Check whether the angle's value is within the range
        if (angle > servo_angle_interval.first || angle < servo_angle_interval.second)
            return { false, 0 };

        offset = servo_angle_interval.first - angle;
    }
    
    // Calculate theta value
    double theta = theta_interval.first + offset;

    if (theta < theta_interval.first || theta > theta_interval.second)
        return { false, 0 };

    return { true, theta };
}

auto KinematicsCalculator::theta_to_servo_angle(uint8_t theta_id, double angle) -> std::pair<bool, double> {
    auto it = std::find_if(
        this->servo_angle_relation_map.begin(),
        this->servo_angle_relation_map.end(),
        [=](std::pair<uint8_t, ServoAngleRelation> pair){
            return pair.second.theta_id == theta_id;
        }
    );

    if (it == this->servo_angle_relation_map.end())
        return { false, 0};

    auto servo_angle_interval = it->second.servo_angle_interval;
    auto theta_interval = it->second.theta_interval;

    // Check whether the angle's value is within the range
    if (angle < theta_interval.first || angle > theta_interval.second)
        return { false, 0 };

    const double& offset = angle - theta_interval.first;
    double servo_angle = 0;

    // Compare left and right boundaries, then calculate offset
    if (servo_angle_interval.first < servo_angle_interval.second) {
        servo_angle = servo_angle_interval.first + offset;

        if (servo_angle < servo_angle_interval.first || servo_angle > servo_angle_interval.second)
            return { false, 0 };
    } else {
        servo_angle = servo_angle_interval.first - offset;

        if (servo_angle > servo_angle_interval.first || servo_angle < servo_angle_interval.second)
            return { false, 0 };
    }

    return { true, servo_angle};
}

auto KinematicsCalculator::calculate(KinematicsType type) -> void {
    switch (type) {
    case KinematicsType::FORWARD: {
        auto T = this->forward_kinematics_equation;

        const auto& theta1 = servo_angle_to_theta(6, 120).second;
        const auto& theta2 = servo_angle_to_theta(5, 140).second;
        const auto& theta3 = servo_angle_to_theta(4, 195).second;
        const auto& theta4 = servo_angle_to_theta(3, 30).second;
        const auto& theta5 = servo_angle_to_theta(2, 120).second;

        substitute(T, SymEngine::symbol("α1"), SymEngine::real_double(alpha1));
        substitute(T, SymEngine::symbol("α4"), SymEngine::real_double(alpha4));
        substitute(T, SymEngine::symbol("a2"), SymEngine::real_double(perpendicular_distance_between_link2_joint_z_axes));
        substitute(T, SymEngine::symbol("a3"), SymEngine::real_double(perpendicular_distance_between_link3_joint_z_axes));
        substitute(T, SymEngine::symbol("Θ1"), SymEngine::real_double(deg_to_rad(theta1)));
        substitute(T, SymEngine::symbol("Θ2"), SymEngine::real_double(deg_to_rad(theta2)));
        substitute(T, SymEngine::symbol("Θ3"), SymEngine::real_double(deg_to_rad(theta3)));
        substitute(T, SymEngine::symbol("Θ4"), SymEngine::real_double(deg_to_rad(theta4)));
        substitute(T, SymEngine::symbol("Θ5"), SymEngine::real_double(deg_to_rad(theta5)));
    } break;
    case KinematicsType::INVERSE: {
    } break;
    default: break;
    }
}

auto KinematicsCalculator::rpy(const SymEngine::DenseMatrix& rotation_matrix) -> void {
    // Check whether all elements are floating point number
    for (size_t i = 0; i < rotation_matrix.nrows(); i++) {
        for (size_t j = 0; j < rotation_matrix.ncols(); j++) {
            if (!SymEngine::is_a_Number(*rotation_matrix.get(i, j))) 
                return;
        }
    }

    const auto& r11 = get_value(rotation_matrix.get(0, 0));
    const auto& r21 = get_value(rotation_matrix.get(1, 0));
    const auto& r31 = get_value(rotation_matrix.get(2, 0));
    const auto& r32 = get_value(rotation_matrix.get(2, 1));
    const auto& r33 = get_value(rotation_matrix.get(2, 2));

    auto roll = std::atan2(r32, r33);
    auto pitch = std::atan2(r21, r11);
    auto yaw  = std::atan2(-r31, std::sqrt(std::pow(r11, 2) + std::pow(r21, 2)));

    RCLCPP_INFO(rclcpp::get_logger(""), "Roll: %f", roll);
    RCLCPP_INFO(rclcpp::get_logger(""), "Pitch: %f", pitch);
    RCLCPP_INFO(rclcpp::get_logger(""), "Yaw: %f", yaw);
}


auto KinematicsCalculator::generate_forward_kinematics_equation() -> void {
    this->transformation_operators = ForwardKinematics::generate_transformation_operators({
        DHParameter(SymEngine::integer(0),          SymEngine::integer(0),          SymEngine::integer(0),     SymEngine::symbol("Θ1")),
        DHParameter(SymEngine::symbol("α1"),     SymEngine::integer(0),          SymEngine::integer(0),     SymEngine::symbol("Θ2")),
        DHParameter(SymEngine::integer(0),          SymEngine::symbol("a2"),     SymEngine::integer(0),     SymEngine::symbol("Θ3")),
        DHParameter(SymEngine::integer(0),          SymEngine::symbol("a3"),     SymEngine::integer(0),     SymEngine::symbol("Θ4")),
        DHParameter(SymEngine::symbol("α4"),     SymEngine::integer(0),          SymEngine::integer(0),     SymEngine::symbol("Θ5")),
    });

    this->forward_kinematics_equation = ForwardKinematics::generate_equation(this->transformation_operators);
}


auto KinematicsCalculator::initialize() -> void {
    this->sub_vision = this->create_subscription<kinematics_msgs::msg::Transformation>(
        "/vision/extrinsic_parameters", 
        1,
        std::bind(&KinematicsCalculator::receive_transformation_from_vision, this, std::placeholders::_1)
    );

    initialize_servo_angle_relation_map();

    generate_forward_kinematics_equation();

    this->srv_ini = this->create_service<std_srvs::srv::Trigger>(
        "~/initialization_complete", 
        std::bind(&KinematicsCalculator::initialization_complete, this, std::placeholders::_1, std::placeholders::_2)
    );
}

auto KinematicsCalculator::initialize_servo_angle_relation_map() -> void {
    this->servo_angle_relation_map = {
        { 2, ServoAngleRelation{2, {240, 0},    5, {-120, 120}} },
        { 3, ServoAngleRelation{3, {240, 10},   4, {-30, 200}} },
        { 4, ServoAngleRelation{4, {0, 240},    3, {-120, 120}} },
        { 5, ServoAngleRelation{5, {30, 210},   2, {-180, 0}} },
        { 6, ServoAngleRelation{6, {0, 190},    1, {-120, 70}} }
    };

    this->theta_interval_map = {
        { 1, {-120, 70} },
        { 2, {-180, 0} },
        { 3, {-120, 120}, },
        { 4, {-30, 200} },
        { 5, {-120, 120} }
    };
}


auto KinematicsCalculator::print_solutions(const std::vector<Solution>& solutions) -> void {
    for (auto s : solutions) {
        std::string str;

        uint8_t i = 1;
        for (auto pair : s.theta_value_map) {
            char buf[20];
            std::sprintf(buf, "Θ%d: %f ", i++, pair.second);

            str += buf;
        }
        str += "\n";

        RCLCPP_INFO(rclcpp::get_logger(""), "%s", str.c_str());
    }
}

auto main(int argc, const char* argv[]) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KinematicsCalculator>());
    rclcpp::shutdown();

    return 0;
}




/////////////////////////////////