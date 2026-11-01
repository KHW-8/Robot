#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

//////////* Headers *//////////

/* Eigen */
#include <eigen3/Eigen/Dense>

/* SymEngine */
#include <symengine/matrix.h>

///////////////////////////////

class InverseKinematics {
public:
    static auto calculate(SymEngine::DenseMatrix T) -> void;
    static auto calculate(Eigen::Matrix4d T) -> void;
};

#endif