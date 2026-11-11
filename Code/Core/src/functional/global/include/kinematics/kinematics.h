#ifndef KINEMATICS_H
#define KINEMATICS_H

//////////* Headers *//////////

/* Eigen*/
#include <eigen3/Eigen/Dense>

///////////////////////////////

//////////* Functions *//////////

/**/
auto rotation_matrix_to_rpy(Eigen::Matrix3d R) -> Eigen::Vector3d;

/* Inverse */
auto inverse_transformation_matrix(Eigen::Matrix4d T) -> Eigen::Matrix4d;

/////////////////////////////////

#endif