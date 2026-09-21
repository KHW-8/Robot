#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

//////////* Headers *//////////

/* Core */
//// Kinematics
#include "DH_parameter.h"
/* STD */
#include <vector>

///////////////////////////////

class InverseKinematics {
public:
    static auto calculate(std::vector<DHParameter> v) -> void;
};

#endif