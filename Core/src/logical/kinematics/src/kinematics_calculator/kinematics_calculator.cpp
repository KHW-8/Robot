#include "kinematics_calculator.h"

//////////* Headers *//////////

/* Core */
//// Kinematics
#include "forward_kinematics.h"
#include "inverse_kinematics.h"
//// Global
#include "math.h"
#include "utility.hpp"

/* ROS */
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

auto link1 = 0.05;
auto link2 = 0.10048;
auto link3 = 0.1;
auto link4 = 0.055;

////////////////////////////////////////

//////////* Functions *//////////

KinematicsCalculator::KinematicsCalculator() 
    : Node("kinematics_calculator")
{
    initialize();

    calculate(KinematicsType::FORWARD);
}

auto KinematicsCalculator::initialization_complete([[maybe_unused]]std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
                                                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> void 
{
    response->success = true;
}


auto KinematicsCalculator::calculate(KinematicsType type) -> void {
    switch (type) {
    case KinematicsType::FORWARD: {
        auto T = forward_kinematics_equation;

        substitute(T, SymEngine::symbol("α1"), SymEngine::real_double(alpha1));
        substitute(T, SymEngine::symbol("α4"), SymEngine::real_double(alpha4));
        substitute(T, SymEngine::symbol("a2"), SymEngine::real_double(link2));
        substitute(T, SymEngine::symbol("a3"), SymEngine::real_double(link3));
        substitute(T, SymEngine::symbol("d2"), SymEngine::real_double(link1));
        substitute(T, SymEngine::symbol("d5"), SymEngine::real_double(link4));
        substitute(T, SymEngine::symbol("Θ1"), SymEngine::real_double(pulse_to_rad(500)));
        substitute(T, SymEngine::symbol("Θ2"), SymEngine::real_double(pulse_to_rad(600)));
        substitute(T, SymEngine::symbol("Θ3"), SymEngine::real_double(pulse_to_rad(820)));
        substitute(T, SymEngine::symbol("Θ4"), SymEngine::real_double(pulse_to_rad(110)));
        substitute(T, SymEngine::symbol("Θ5"), SymEngine::real_double(pulse_to_rad(500)));
        // substitute(T, SymEngine::symbol("Θ1"), SymEngine::real_double(deg_to_rad(10)));
        // substitute(T, SymEngine::symbol("Θ2"), SymEngine::real_double(deg_to_rad(20)));
        // substitute(T, SymEngine::symbol("Θ3"), SymEngine::real_double(deg_to_rad(30)));
        // substitute(T, SymEngine::symbol("Θ4"), SymEngine::real_double(deg_to_rad(40)));
        // substitute(T, SymEngine::symbol("Θ5"), SymEngine::real_double(deg_to_rad(50)));

        InverseKinematics::calculate(T);
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
        DHParameter(SymEngine::integer(0),          SymEngine::integer(0),          SymEngine::integer(0),          SymEngine::symbol("Θ1")),
        DHParameter(SymEngine::symbol("α1"),     SymEngine::integer(0),          SymEngine::symbol("d2"),     SymEngine::symbol("Θ2")),
        DHParameter(SymEngine::integer(0),          SymEngine::symbol("a2"),     SymEngine::integer(0),          SymEngine::symbol("Θ3")),
        DHParameter(SymEngine::integer(0),          SymEngine::symbol("a3"),     SymEngine::integer(0),          SymEngine::symbol("Θ4")),
        DHParameter(SymEngine::symbol("α4"),     SymEngine::integer(0),          SymEngine::symbol("d5"),     SymEngine::symbol("Θ5")),
    });

    this->forward_kinematics_equation = ForwardKinematics::generate_equation(this->transformation_operators);
}


auto KinematicsCalculator::initialize() -> void {
    generate_forward_kinematics_equation();

    // this->cli_robot_controller = this->create_client<std_srvs::srv::Trigger>("/robot_controller/initialization_complete");
    // this->cli_robot_controller->wait_for_service();

    this->srv_ini = this->create_service<std_srvs::srv::Trigger>(
        "~/initialization_complete", 
        std::bind(&KinematicsCalculator::initialization_complete, this, std::placeholders::_1, std::placeholders::_2)
    );
}

auto main(int argc, const char* argv[]) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KinematicsCalculator>());
    rclcpp::shutdown();

    return 0;
}

/////////////////////////////////