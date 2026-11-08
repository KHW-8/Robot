#ifndef MATH_H
#define MATH_H

//////////* Headers *//////////

/* Eigen*/
#include <eigen3/Eigen/Dense>

/* SymEngine */
#include <symengine/basic.h>
#include <symengine/matrix.h>

///////////////////////////////

///////////* Macros *///////////
#define EPSILON 1e-12

////////////////////////////////

//////////* Functions *//////////

/* Compare */
auto equals(double a, double b) -> bool;

/* Converter */
auto deg_to_rad(double degree) -> double;

auto rad_to_deg(double radian) -> double;

auto pulse_to_deg(size_t pulse) -> double;

auto pulse_to_rad(size_t pulse) -> double;

auto to_equivalent_angle(double angle) -> double;

/* Inverse */
auto inverse_transformation_matrix(Eigen::Matrix4d T) -> Eigen::Matrix4d;

/* Simplify */
auto nsimplify(const double& num) -> double;
auto nsimplify(Eigen::Vector3d& v) -> void;
auto nsimplify(Eigen::Matrix4d& m) -> void;
auto nsimplify(SymEngine::DenseMatrix& m) -> void;
auto nsimplify(SymEngine::RCP<const SymEngine::Basic> element) -> std::pair<bool, SymEngine::RCP<const SymEngine::Basic> >;

/* Matrix */
auto create_identity_matrix(size_t row, size_t col) -> SymEngine::DenseMatrix;

auto expand(SymEngine::DenseMatrix& m) -> void;

auto substitute(SymEngine::DenseMatrix& m,  SymEngine::RCP<const SymEngine::Basic> _old,  SymEngine::RCP<const SymEngine::Basic> _new)-> void;

auto simplify(SymEngine::DenseMatrix& m) -> void;

auto product(std::vector<SymEngine::DenseMatrix> v, size_t begin=0, size_t end=0) -> SymEngine::DenseMatrix;

#endif

/////////////////////////////////