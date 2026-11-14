#include "inverse_kinematics.h"
#include "math.h"

//////////* Headers *//////////

/* Core */
#include "utility.hpp"
#include "math.h"

/* STD */
#include <cmath>

/* SymEngine */
#include <cstddef>
#include <eigen3/Eigen/src/Core/Matrix.h>
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
    auto theta1 = rad_to_deg(std::atan2(r23, r13));
    theta1 = to_equivalent_angle(theta1);

    // Θ5
    auto theta5 = rad_to_deg(std::atan2(r32, -r31));
    theta5 = to_equivalent_angle(theta5);
    
    // Θ2 + Θ3 + Θ4, -320 <= Θ_234 <= 320
    const auto& s234 = r13*std::cos(deg_to_rad(theta1)) + r23*std::sin(deg_to_rad(theta1));
    const auto& c234 = r33;
    auto theta234 = rad_to_deg(std::atan2(s234, c234)); 
    theta234 = theta234 < 0 ? theta234 + 360 : theta234;

    auto v = calculate_theta2_theta3_theta4(px, py, pz, theta1, theta234);
    for (auto& s : v) {
        s.theta_value_map[1] = theta1;
        s.theta_value_map[5] = theta5;
    }

    solutions.insert(solutions.end(), v.begin(), v.end());

    /* The last two solution */
    const auto& theta1_prime = to_equivalent_angle(theta1 + 180);
    const auto& theta5_prime = to_equivalent_angle(theta5 + 180);
    const auto& theta234_prime = -theta234 < 0 ? -theta234 + 360 : -theta234;

    v = calculate_theta2_theta3_theta4(px, py, pz, theta1_prime, theta234_prime);
    for (auto& s : v) {
        s.theta_value_map[1] = theta1_prime;
        s.theta_value_map[5] = theta5_prime;
    }

    solutions.insert(solutions.end(), v.begin(), v.end());

    filter_solutions(solutions, angle_interval_map);

    return solutions;
}

auto InverseKinematics::calculate(SymEngine::DenseMatrix T, std::map<uint8_t, std::pair<double, double> > angle_interval_map) -> std::vector<Solution> {
    /* Check whether all elements are number */
    Eigen::Matrix4d m;

    for (size_t i = 0; i < T.nrows(); i++) {
        for (size_t j = 0; j < T.ncols(); j++) {
            if (!SymEngine::is_a_Number(*T.get(i, j))) 
                return {};
            else
                m(i, j) = get_value(T.get(i, j));
        }
    }

    return calculate(
        m,
        angle_interval_map
    );
}

auto InverseKinematics::calculate_theta2_theta3_theta4(double px, double py, double pz,double theta1, double theta234) -> std::vector<Solution> {
    /* Θ3
     * (1)
     * A a2*c2 + a3c23 = px*c1 - py*s1
     * B = a2*s2 + a3s23 = -pz
     *
     * (2)
     * A^2 + B^2 = a2^2 + a3^2 + 2a2*a3*(c2c23 + s2s23)
     *           = a2^2 + a3^2 + 2a2*a3*c3
     *
     * (3)
     * c3 = ( a2^2 + a3^2 ) / 2a2*a3
     */

    // First solution
    const auto A = px*std::cos(deg_to_rad(theta1)) + py*std::sin(deg_to_rad(theta1)); 
    const auto B = -pz; 
    const auto c3 = (std::pow(A, 2) + std::pow(B, 2) - std::pow(perpendicular_distance_between_link2_joint_z_axes, 2) - std::pow(perpendicular_distance_between_link3_joint_z_axes, 2))/(2*perpendicular_distance_between_link2_joint_z_axes*perpendicular_distance_between_link3_joint_z_axes);
    const auto s3 = std::sqrt(1 - std::pow(c3, 2));
    auto theta3 = rad_to_deg(std::atan2(s3, c3));
    theta3 = to_equivalent_angle(theta3);

    
    auto s1 = calculate_theta2_theta4(theta3, theta234, A, B);
    s1.theta_value_map[3] = theta3;
    
    // Second solution
    auto s2 = calculate_theta2_theta4(-theta3, theta234, A, B);
    s2.theta_value_map[3] = -theta3;

    return { s1, s2 };
}

auto InverseKinematics::calculate_theta2_theta4(double theta3, double theta234, double A, double B) -> Solution {
    /* Θ2
     * C = a2 + a3*c3
     * D = a3s3
     */
    const auto C = perpendicular_distance_between_link2_joint_z_axes + perpendicular_distance_between_link3_joint_z_axes*std::cos(deg_to_rad(theta3)); 
    const auto D = perpendicular_distance_between_link3_joint_z_axes*std::sin(deg_to_rad(theta3)); 
    auto theta2 = rad_to_deg(std::atan2(B, A) - std::atan2(D, C));
    theta2 = to_equivalent_angle(theta2);

    // Θ4
    auto theta4 = theta234 - theta2 - theta3;
    theta4 = to_equivalent_angle(theta4);


    Solution s;
    s.theta_value_map[2] = theta2;
    s.theta_value_map[4] = theta4;

    return s;
}

auto InverseKinematics::filter_solutions(std::vector<Solution>& solutions, 
                                         std::map<uint8_t, std::pair<double, double> > angle_interval_map) -> void 
{
    std::vector<size_t> v;
    size_t i = 0;

    for (auto it = solutions.begin(); it != solutions.end(); it++, i++) {
        for (auto& pair : it->theta_value_map) {
            const auto& theta_id = pair.first;
            const auto& theta = pair.second;

            auto it2 = angle_interval_map.find(theta_id);
            if (it2 == angle_interval_map.end()) {
                v.emplace_back(i);
                break;
            }

            const auto& left_boundary = it2->second.first;
            const auto& right_boundary = it2->second.second;

            if (theta < left_boundary || theta > right_boundary) {
                v.emplace_back(i);
                break;
            }
        }
    }

    for (auto it = v.rbegin(); it != v.rend(); it++)  
        solutions.erase(solutions.begin() + *it);
}


///////////////////////////////////////