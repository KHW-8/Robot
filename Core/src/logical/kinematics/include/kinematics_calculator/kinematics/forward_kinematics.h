#ifndef FORWARD_KINEMATICS_H
#define FORWARD_KINEMATICS_H

//////////* Headers *//////////

/* Core */
//// Kinematics
#include "DH_parameter.h"
/* STD */
#include <vector>

///////////////////////////////

//////////* Classes *//////////


class ForwardKinematics {
public:
    static auto calculate(std::vector<DHParameter> v) -> void;
};

///////////////////////////////

#endif