#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

//////////* Headers *//////////

/* Eigen */
#include <cstdint>
#include <eigen3/Eigen/Dense>

/* STD*/
#include <sys/types.h>
#include <vector>

/* SymEngine */
#include <symengine/matrix.h>

///////////////////////////////

struct Solution {
    std::map<uint8_t, double> theta_value_map;
};

class InverseKinematics {
public:
    static auto calculate(SymEngine::DenseMatrix T, std::map<uint8_t, std::pair<double, double> > angle_interval_map) -> std::vector<Solution>;
    static auto calculate(Eigen::Matrix4d T, std::map<uint8_t, std::pair<double, double> > angle_interval_map) -> std::vector<Solution>;

private:
    static auto calculate_theta2_theta3_theta4(double px, double py, double pz, double theta1, double theta234) -> std::vector<Solution>;
    static auto calculate_theta2_theta4(double theta3, double theta234, double A, double B) -> Solution;

    static auto filter_solutions(std::vector<Solution>& solutions, std::map<uint8_t, std::pair<double, double> > angle_interval_map) -> void;
};

#endif