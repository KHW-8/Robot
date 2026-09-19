#include "math.h"

#include <symengine/simplify.h>


auto substitute(SymEngine::DenseMatrix& m, 
                SymEngine::RCP<const SymEngine::Basic> _old, 
                SymEngine::RCP<const SymEngine::Basic> _new)-> void {
   for (size_t i = 0; i < m.nrows(); i++) {
        for (size_t j = 0; j < m.ncols(); j++) {
            m.set(
                i, 
                j, 
                m.get(i, j)->subs({ { _old, _new } })
            );
            
            // Some element is float number and approaches 0, then change it to integer number with 0.
            if (m.get(i, j)->get_type_code() == SymEngine::TypeID::SYMENGINE_REAL_DOUBLE) {
                auto& num = down_cast<const SymEngine::RealDouble&>(*m.get(i, j));
                if (num.as_double() == static_cast<double>(-1)) // Equals to -1
                    m.set(i, j, SymEngine::integer(-1));
                else if (std::abs(num.as_double()) < std::numeric_limits<double>::epsilon()) // Equals to 0
                    m.set(i, j, SymEngine::integer(0));
                else if (num.as_double() == static_cast<double>(1)) // Equals to 1
                    m.set(i, j, SymEngine::integer(1));
            }
        }
    }
}

auto create_identity_matrix(size_t row, size_t col) -> SymEngine::DenseMatrix {
    if (row == 0 || col == 0)
        return SymEngine::DenseMatrix();

    SymEngine::vec_basic v;
    for (size_t i = 0; i < row; i++) {
        for (size_t j = 0; j < col; j++) {
            if (i == j)
                v.emplace_back(SymEngine::integer(1));
            else
                v.emplace_back(SymEngine::integer(0));
        }
    }

    SymEngine::DenseMatrix I(row, col, v);


    return I;
}

auto simplify(SymEngine::DenseMatrix& m) -> void {
    for (size_t i = 0; i < m.nrows(); i++) {
        for (size_t j = 0; j < m.ncols(); j++) {
            m.set(i, j, SymEngine::simplify(m.get(i, j)));
        }
    }
}

auto expand(SymEngine::DenseMatrix& m) -> void {
    for (size_t i = 0; i < m.nrows(); i++) {
        for (size_t j = 0; j < m.ncols(); j++) {
            m.set(i, j, SymEngine::expand(m.get(i, j)));
        }
    }
}

auto product(std::vector<SymEngine::DenseMatrix> v, size_t begin, size_t end) -> SymEngine::DenseMatrix {
    auto res = create_identity_matrix(4, 4);

    if (end != 0) {
        for (size_t i = begin; i < end; i++) 
            res.mul_matrix(v.at(i), res);
    } else {
        for (auto& T : v)
            res.mul_matrix(T, res);
    }


    return res;
}
