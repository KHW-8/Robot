//////////* Headers *//////////
/* Core */
#include "kinematics.h"
#include "math.h"

/* STD */
#include <cmath>

///////////////////////////////

//////////* Functions *//////////

auto rotation_matrix_to_rpy(Eigen::Matrix3d R) -> Eigen::Vector3d {
    const auto& r11 = R(0, 0);
    const auto& r21 = R(1, 0);
    const auto& r31 = R(2, 0);
    const auto& r32 = R(2, 1);
    const auto& r33 = R(2, 2);

    const auto& roll = rad_to_deg(std::atan2(r32, r33));
    const auto& pitch = rad_to_deg(std::atan2(-r31, std::sqrt(std::pow(r11, 2) + std::pow(r21, 2))));
    const auto& yaw = rad_to_deg(std::atan2(r21, r11));

    return Eigen::Vector3d{ roll, pitch, yaw};
}

auto inverse_transformation_matrix(Eigen::Matrix4d T) -> Eigen::Matrix4d {
     auto R = T.block(0, 0,3, 3);
     auto t = T.topRightCorner<3, 1>();

    auto RT = R.transpose();
    
    auto p = - RT * t;

    Eigen::Matrix4d inversed_T;
    inversed_T.block<3, 3>(0, 0) = RT;
    inversed_T.block<3, 1>(0, 3) = p;
    inversed_T.block<1, 4>(3, 0) = Eigen::Vector4d{0, 0, 0 , 1};

    return inversed_T;
}

/////////////////////////////////