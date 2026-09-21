#ifndef MATH_H
#define MATH_H

#include <symengine/matrix.h>

auto substitute(SymEngine::DenseMatrix& m,  SymEngine::RCP<const SymEngine::Basic> _old,  SymEngine::RCP<const SymEngine::Basic> _new)-> void;

auto create_identity_matrix(size_t row, size_t col) -> SymEngine::DenseMatrix;

auto simplify(SymEngine::DenseMatrix& m) -> void;

auto expand(SymEngine::DenseMatrix& m) -> void;

auto product(std::vector<SymEngine::DenseMatrix> v, size_t begin=0, size_t end=0) -> SymEngine::DenseMatrix;

#endif