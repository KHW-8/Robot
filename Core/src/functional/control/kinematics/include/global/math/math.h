#ifndef MATH_H
#define MATH_H

//////////* Headers *//////////

/* SymEngine */
#include <symengine/basic.h>
#include <symengine/matrix.h>

///////////////////////////////

//////////* Functions *//////////

/* Converter */
auto deg_to_rad(double degree) -> double;

auto rad_to_deg(double radian) -> double;

auto pulse_to_deg(size_t pulse) -> double;

auto pulse_to_rad(size_t pulse) -> double;

/* Simplify */
auto nsimplify(SymEngine::RCP<const SymEngine::Basic> element) -> std::pair<bool, SymEngine::RCP<const SymEngine::Basic> >;

/* Matrix */
auto create_identity_matrix(size_t row, size_t col) -> SymEngine::DenseMatrix;

auto expand(SymEngine::DenseMatrix& m) -> void;

auto substitute(SymEngine::DenseMatrix& m,  SymEngine::RCP<const SymEngine::Basic> _old,  SymEngine::RCP<const SymEngine::Basic> _new)-> void;

auto simplify(SymEngine::DenseMatrix& m) -> void;

auto product(std::vector<SymEngine::DenseMatrix> v, size_t begin=0, size_t end=0) -> SymEngine::DenseMatrix;

#endif

/////////////////////////////////