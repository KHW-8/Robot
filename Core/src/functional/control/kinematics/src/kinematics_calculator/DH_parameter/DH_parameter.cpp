#include "DH_parameter.h"

#include <rclcpp/logger.hpp>

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
    std::cout << "α: " << *this->alpha << std::endl;
    std::cout << "a: " << *this->a << std::endl;
    std::cout << "d: " << *this->d << std::endl;
    std::cout << "Θ: " << *this->theta << std::endl;
    std::cout << std::endl;
}