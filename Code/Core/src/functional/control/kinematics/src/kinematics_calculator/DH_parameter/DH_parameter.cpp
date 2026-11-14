#include "DH_parameter.h"

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

DHParameter::DHParameter(SymEngine::RCP<const SymEngine::Basic> alpha, 
            SymEngine::RCP<const SymEngine::Basic> a,
            SymEngine::RCP<const SymEngine::Basic> d,
            SymEngine::RCP<const SymEngine::Basic> theta) 
{
    this->alpha = alpha;
    this->a = a;
    this->d = d;
    this->theta = theta;
}


auto DHParameter::print() -> void {
}