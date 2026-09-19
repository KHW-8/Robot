#include "kinematics_calculator.h"

//////////* Headers *//////////

/* Core */
//// Kinematics
#include "forward_kinematics.h"
//// Global
#include "math.hpp"

/* ROS */
#include <rclcpp/executors.hpp>
#include <rclcpp/logger.hpp>

/* STD */
#include <functional>

/* SymEngine */
#include <symengine/real_double.h>

///////////////////////////////

//////////* Global Variables *//////////


////////////////////////////////////////

//////////* Functions *//////////

KinematicsCalculator::KinematicsCalculator() 
    : Node("kinematics_calculator")
{
    initialize();
}

auto KinematicsCalculator::initialization_complete([[maybe_unused]]std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
                                                        std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> void 
{
    response->success = true;
}


auto KinematicsCalculator::calculate(KinematicsType type) -> void {
    switch (type) {
    case KinematicsType::FORWARD: {
        ForwardKinematics::calculate({
            DHParameter(SymEngine::integer(0), SymEngine::integer(0), SymEngine::integer(0), SymEngine::symbol("Θ1")),
            DHParameter(SymEngine::real_double(deg_to_rad(-90)), SymEngine::integer(0), SymEngine::symbol("d2"), SymEngine::symbol("Θ2")),
            DHParameter(SymEngine::real_double(deg_to_rad(90)), SymEngine::integer(0), SymEngine::symbol("d3"), SymEngine::real_double(deg_to_rad(180))),
            DHParameter(SymEngine::integer(0), SymEngine::symbol("a3"), SymEngine::symbol("d4"), SymEngine::symbol("Θ4")),
            DHParameter(SymEngine::real_double(deg_to_rad(90)), SymEngine::integer(0), SymEngine::integer(0), SymEngine::symbol("Θ5")),
            DHParameter(SymEngine::real_double(deg_to_rad(-90)), SymEngine::integer(0), SymEngine::integer(0), SymEngine::symbol("Θ6"))
        });
    } break;
    case KinematicsType::INVERSE: break;
    default: break;
    }
   
}

auto KinematicsCalculator::initialize() -> void {
    this->srv_ini = this->create_service<std_srvs::srv::Trigger>(
        "~/initialization_complete", 
        std::bind(&KinematicsCalculator::initialization_complete, this, std::placeholders::_1, std::placeholders::_2)
    );
}

auto main(int argc, const char* argv[]) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin( std::make_shared<KinematicsCalculator>());
    rclcpp::shutdown();

    return 0;
}

/////////////////////////////////