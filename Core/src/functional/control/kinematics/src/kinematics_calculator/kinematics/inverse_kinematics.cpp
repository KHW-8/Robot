#include "inverse_kinematics.h"
#include "math.h"

//////////* Headers *//////////

/* Core */
#include "utility.hpp"

/* STD */
#include <cmath>
#include <ostream>

/* SymEngine */
#include <symengine/basic.h>
#include <symengine/integer.h>
#include <symengine/matrices/transpose.h>
#include <symengine/matrix.h>
#include <symengine/real_double.h>

///////////////////////////////


//////////* Global Variables *//////////

extern double perpendicular_distance_between_link2_joint_z_axes;
extern double perpendicular_distance_between_link3_joint_z_axes;
extern double perpendicular_distance_between_link4_joint_x_axes;

////////////////////////////////////////

/////////////* Functions */////////////

auto InverseKinematics::calculate(Eigen::Matrix4d T) -> void {
     /* Calculate all joint revolution angle */
    const auto& r13 = T(0, 2);
    const auto& r23 = T(1, 2);
    const auto& r31 = T(2, 0);
    const auto& r32 = T(2, 1);
    const auto& r33 = T(2, 2);
    const auto& px = T(0, 3);
    const auto& py = T(1, 3);
    const auto& pz = T(2, 3);

    // Θ1
    auto theta1 = rad_to_deg(std::atan2(r23, r13));
    theta1 = theta1 < 0 ? theta1 + 180 : theta1;

    // Θ5
    auto theta5 = rad_to_deg(std::atan2(r32, -r31));
    theta5 = theta5 < 0 ? theta5 + 180 : theta5;

    // Θ2 + Θ3 + Θ4
    const auto& c234 = r33;
    const auto s234 = r32 / std::sin(deg_to_rad(theta5));
    auto theta234 = rad_to_deg(std::atan2(s234, c234));
    theta234 = theta234 < 0 ? theta234 + 360 : theta234;

    /* Θ3
     * (1)
     * A a2*c2 + a3c23 = px*c1 - py*s1 - d5*s234
     * B = a2*s2 + a3s23 = d5*c234 - pz
     *
     * (2)
     * A^2 + B^2 = a2^2 + a3^2 + 2a2*a3*(c2c23 + s2s23)
     *           = a2^2 + a3^2 + 2a2*a3*c3
     *
     * (3)
     * c3 = ( a2^2 + a3^2 ) / 2a2*a3
     */
    const auto A = px*std::cos(deg_to_rad(theta1)) + py*std::sin(deg_to_rad(theta1)) - perpendicular_distance_between_link4_joint_x_axes*std::sin(deg_to_rad((theta234))); 
    const auto B = perpendicular_distance_between_link4_joint_x_axes*std::cos(deg_to_rad(theta234)) - pz; 
    const auto theta3 = rad_to_deg(std::acos((std::pow(A, 2) + std::pow(B, 2) - std::pow(perpendicular_distance_between_link2_joint_z_axes, 2) - std::pow(perpendicular_distance_between_link3_joint_z_axes, 2))/(2*perpendicular_distance_between_link2_joint_z_axes*perpendicular_distance_between_link3_joint_z_axes)));
    /* Θ2
     * C = a2 + a3*c3
     * D = a3s3
     */
    const auto C = perpendicular_distance_between_link2_joint_z_axes + perpendicular_distance_between_link3_joint_z_axes*std::cos(deg_to_rad(theta3)); 
    const auto D = perpendicular_distance_between_link3_joint_z_axes*std::sin(deg_to_rad(theta3)); 
    const auto theta2 = rad_to_deg(std::atan2(B, A) - std::atan2(D, C));
    // Θ4
    const auto theta4 = theta234 - theta2 - theta3;
}

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
    auto theta1 = rad_to_deg(std::atan2(r23, r13));
    theta1 = theta1 < 0 ? theta1 + 180 : theta1;

    // Θ5
    auto theta5 = rad_to_deg(std::atan2(r32, -r31));
    theta5 = theta5 < 0 ? theta5 + 180 : theta5;

    // Θ2 + Θ3 + Θ4
    const auto& c234 = r33;
    const auto s234 = r32 / std::sin(deg_to_rad(theta5));
    auto theta234 = rad_to_deg(std::atan2(s234, c234));
    theta234 = theta234 < 0 ? theta234 + 360 : theta234;

    /* Θ3
     * (1)
     * A a2*c2 + a3c23 = px*c1 - py*s1 - d5*s234
     * B = a2*s2 + a3s23 = d5*c234 - pz
     *
     * (2)
     * A^2 + B^2 = a2^2 + a3^2 + 2a2*a3*(c2c23 + s2s23)
     *           = a2^2 + a3^2 + 2a2*a3*c3
     *
     * (3)
     * c3 = ( a2^2 + a3^2 ) / 2a2*a3
     */
    const auto A = px*std::cos(deg_to_rad(theta1)) + py*std::sin(deg_to_rad(theta1)) - perpendicular_distance_between_link4_joint_x_axes*std::sin(deg_to_rad((theta234))); 
    const auto B = perpendicular_distance_between_link4_joint_x_axes*std::cos(deg_to_rad(theta234)) - pz; 
    const auto theta3 = rad_to_deg(std::acos((std::pow(A, 2) + std::pow(B, 2) - std::pow(perpendicular_distance_between_link2_joint_z_axes, 2) - std::pow(perpendicular_distance_between_link3_joint_z_axes, 2))/(2*perpendicular_distance_between_link2_joint_z_axes*perpendicular_distance_between_link3_joint_z_axes)));
    /* Θ2
     * C = a2 + a3*c3
     * D = a3s3
     */
    const auto C = perpendicular_distance_between_link2_joint_z_axes + perpendicular_distance_between_link3_joint_z_axes*std::cos(deg_to_rad(theta3)); 
    const auto D = perpendicular_distance_between_link3_joint_z_axes*std::sin(deg_to_rad(theta3)); 
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