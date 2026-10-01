#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

//////////* Headers *//////////

/* SymEngine */
#include <symengine/matrix.h>

///////////////////////////////

class InverseKinematics {
public:
    static auto calculate(SymEngine::DenseMatrix T) -> void;
};

#endif