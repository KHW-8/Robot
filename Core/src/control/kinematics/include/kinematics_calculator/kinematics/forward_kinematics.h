#ifndef FORWARD_KINEMATICS_H
#define FORWARD_KINEMATICS_H

//////////* Headers *//////////

/* Core */
//// Kinematics
#include "DH_parameter.h"
/* STD */
#include <vector>
/* SymEngine*/
#include <symengine/matrix.h>

///////////////////////////////

//////////* Classes *//////////


class ForwardKinematics {
public:
    static auto generate_transformation_operators(std::vector<DHParameter> v) -> std::vector<SymEngine::DenseMatrix>;
    static auto generate_equation(std::vector<SymEngine::DenseMatrix> v) -> SymEngine::DenseMatrix;
    static auto generate_equation(std::vector<DHParameter> v) -> SymEngine::DenseMatrix;
};

///////////////////////////////

#endif