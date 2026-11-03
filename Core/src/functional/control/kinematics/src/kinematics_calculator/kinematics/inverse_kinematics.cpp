#include "inverse_kinematics.h"
#include "math.h"

//////////* Headers *//////////

/* Core */
#include "utility.hpp"

/* STD */
#include <cmath>

/* SymEngine */
#include <symengine/basic.h>
#include <symengine/integer.h>
#include <symengine/matrices/transpose.h>
#include <symengine/matrix.h>
#include <symengine/real_double.h>
#include <vector>

///////////////////////////////


//////////* Global Variables *//////////

extern double perpendicular_distance_between_link2_joint_z_axes;
extern double perpendicular_distance_between_link3_joint_z_axes;
extern double perpendicular_distance_between_link4_joint_x_axes;

////////////////////////////////////////

/////////////* Functions */////////////

auto InverseKinematics::calculate(Eigen::Matrix4d T, std::map<uint8_t, std::pair<double, double> > angle_interval_map) -> std::vector<Solution> {
    /*
     * This transformation matrix have 4 inverse kinematics solutions
     *      Θ1       Θ5         Θ234        Θ3
     * 1    Θ1       Θ5         +           +
     * 2    Θ1       Θ5         +           -
     * 3    Θ1 + Π   Θ5 + Π     -           +
     * 4    Θ1 + Π   Θ5 + Π     -           -
     */
    std::vector<Solution> solutions;

    // Get needed elements from transformation matrix
    const auto& r13 = T(0, 2);
    const auto& r23 = T(1, 2);
    const auto& r31 = T(2, 0);
    const auto& r32 = T(2, 1);
    const auto& r33 = T(2, 2);
    const auto& px = T(0, 3);
    const auto& py = T(1, 3);
    const auto& pz = T(2, 3);

    /* The first two solution */
    // Θ1
    const auto theta1 = rad_to_deg(std::atan2(r23, r13));

    // Θ5
    const auto theta5 = rad_to_deg(std::atan2(r32, -r31));

    // Θ2 + Θ3 + Θ4
    const auto& c234 = r33;
    const auto& s234 = r32 / std::sin(deg_to_rad(theta5));
    const auto theta234 = rad_to_deg(std::atan2(s234, c234));

    auto v = calculate_theta2_theta3_theta4(px, py, pz, theta1, theta234);
    for (auto& s : v) {
        s.theta1 = theta1;
        s.theta5 = theta5;
    }

    solutions.insert(solutions.end(), v.begin(), v.end());

    /* The last two solution */
    v = calculate_theta2_theta3_theta4(px, py, pz, theta1 + 180, -theta234);
    for (auto& s : v) {
        s.theta1 = theta1 + 180;
        s.theta5 = theta5 + 180;
    }

    solutions.insert(solutions.end(), v.begin(), v.end());

    return solutions;
}

auto InverseKinematics::calculate(SymEngine::DenseMatrix T, std::map<uint8_t, std::pair<double, double> > angle_interval_map) -> std::vector<Solution> {
    /* Check whether all elements are number */
    std::vector<double> v;

    for (size_t i = 0; i < T.nrows(); i++) {
        for (size_t j = 0; j < T.ncols(); j++) {
            if (!SymEngine::is_a_Number(*T.get(i, j))) 
                return {};
            else
                v.emplace_back( get_value(T.get(i, j)));
        }
    }

    return calculate(
        Eigen::Matrix<double, 4, 4, Eigen::RowMajor>(v.data()),
        angle_interval_map
    );
}

auto InverseKinematics::calculate_theta2_theta3_theta4(double px, double py, double pz,double theta1, double theta234) -> std::vector<Solution> {
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

    // First solution
    const auto A = px*std::cos(deg_to_rad(theta1)) + py*std::sin(deg_to_rad(theta1)) - perpendicular_distance_between_link4_joint_x_axes*std::sin(deg_to_rad((theta234))); 
    const auto B = perpendicular_distance_between_link4_joint_x_axes*std::cos(deg_to_rad(theta234)) - pz; 
    const auto c3 = (std::pow(A, 2) + std::pow(B, 2) - std::pow(perpendicular_distance_between_link2_joint_z_axes, 2) - std::pow(perpendicular_distance_between_link3_joint_z_axes, 2))/(2*perpendicular_distance_between_link2_joint_z_axes*perpendicular_distance_between_link3_joint_z_axes);
    const auto s3 = std::sqrt(1 - std::pow(c3, 2));
    const auto theta3 = rad_to_deg(std::atan2(s3, c3));
    
    auto s1 = calculate_theta2_theta4(theta3, theta234, A, B);
    s1.theta3 = theta3;
    
    // Second solution
    auto s2 = calculate_theta2_theta4(-theta3, theta234, A, B);
    s2.theta3 = -theta3;

    return { s1, s2 };
}

auto InverseKinematics::calculate_theta2_theta4(double theta3, double theta234, double A, double B) -> Solution {
    /* Θ2
     * C = a2 + a3*c3
     * D = a3s3
     */
    const auto C = perpendicular_distance_between_link2_joint_z_axes + perpendicular_distance_between_link3_joint_z_axes*std::cos(deg_to_rad(theta3)); 
    const auto D = perpendicular_distance_between_link3_joint_z_axes*std::sin(deg_to_rad(theta3)); 
    const auto theta2 = rad_to_deg(std::atan2(B, A) - std::atan2(D, C));
    // Θ4
    const auto theta4 = theta234 - theta2 - theta3;

    Solution s;
    s.theta2 = theta2;
    s.theta4 = theta4;

    return s;
}

///////////////////////////////////////