#include "inverse_kinematics.h"
#include "math.h"

//////////* Headers *//////////

/* Core */
#include "utility.hpp"

/* STD */
#include <cmath>

/* SymEngine */
#include <ostream>
#include <symengine/basic.h>
#include <symengine/integer.h>
#include <symengine/matrices/transpose.h>
#include <symengine/matrix.h>
#include <symengine/real_double.h>

///////////////////////////////


//////////* Global Variables *//////////

extern double link2;
extern double link3;
extern double link4;

////////////////////////////////////////

/////////////* Functions */////////////

auto InverseKinematics::calculate(SymEngine::DenseMatrix T) -> void {
    /* Check whether all elements are number */
    for (size_t i = 0; i < T.nrows(); i++) {
        for (size_t j = 0; j < T.ncols(); j++) {
            if (!SymEngine::is_a_Number(*T.get(i, j))) 
                return;
        }
    }

    /* Calculate all joint revolution angle */
    const auto& r13 = get_value(T.get(0, 2));
    const auto& r23 = get_value(T.get(1, 2));
    const auto& r31 = get_value(T.get(2, 0));
    const auto& r32 = get_value(T.get(2, 1));
    const auto& r33 = get_value(T.get(2, 2));
    const auto& px = get_value(T.get(0, 3));
    const auto& py = get_value(T.get(1, 3));
    const auto& pz = get_value(T.get(2, 3));

    // Θ1
    const auto theta1 = rad_to_deg(std::atan2(r23, r13));
    // Θ5
    const auto theta5 = rad_to_deg(std::atan2(r32, -r31));
    // Θ2 + Θ3 + Θ4
    const auto& c234 = r33;
    const auto s234 = r32 / std::sin(deg_to_rad(theta5));
    auto theta234 = rad_to_deg(std::atan2(s234, c234));
    // Θ3
    const auto A = px*std::cos(deg_to_rad(theta1)) + py*std::sin(deg_to_rad(theta1)) - link4*std::sin(deg_to_rad((theta234))); //  px*c1 - py*s1 - d5*s234 = a2*c2 + a3c23
    const auto B = link4*std::cos(deg_to_rad(theta234)) - pz; // d5*c234 - pz = a2*s2 + a3s23
    const auto theta3 = rad_to_deg(std::acos((std::pow(A, 2) + std::pow(B, 2) - std::pow(link2, 2) - std::pow(link3, 2))/(2*link2*link3)));
    // Θ2
    const auto C = link2 + link3*std::cos(deg_to_rad(theta3));
    const auto D = link3*std::sin(deg_to_rad(theta3));
    const auto theta2 = rad_to_deg(std::atan2(B, A) - std::atan2(D, C));
    // Θ4
    const auto theta4 = theta234 - theta2 - theta3;

    std::cout << theta1 << std::endl;
    std::cout << theta2 << std::endl;
    std::cout << theta3 << std::endl;
    std::cout << theta4 << std::endl;
    std::cout << theta5 << std::endl;
}

///////////////////////////////////////