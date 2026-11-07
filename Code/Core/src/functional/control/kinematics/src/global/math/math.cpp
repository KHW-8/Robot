#include "math.h"


//////////* Headers *//////////

/* STD */
#include <cstddef>
#include <cstdlib>
#include <numbers>

/* SymEngine */
#include <symengine/simplify.h>

///////////////////////////////


//////////* Functions *//////////

auto equals(double a, double b) -> bool {
    return std::abs(a - b) < EPSILON;
}

auto deg_to_rad(double degree) -> double {
    return degree * std::numbers::pi / 180;
}

auto rad_to_deg(double radian) -> double {
    return radian * 180 / std::numbers::pi;
}

auto pulse_to_deg(size_t pulse) -> double {
    return static_cast<double>(pulse) * 240 / 1000;
}

auto pulse_to_rad(size_t pulse) -> double {
    return deg_to_rad(pulse_to_deg(pulse));
}

auto to_equivalent_angle(double angle) -> double {
    
    if (angle > 0) {
        while (equals(angle, 360) || angle > 360)
            angle -= 360;
    } else if (angle < 0) {
        while (equals(angle, -360) || angle < -360)
            angle += 360;
    }

    return angle;
}

auto substitute(SymEngine::DenseMatrix& m, 
                SymEngine::RCP<const SymEngine::Basic> _old, 
                SymEngine::RCP<const SymEngine::Basic> _new)-> void {
   for (size_t i = 0; i < m.nrows(); i++) {
        for (size_t j = 0; j < m.ncols(); j++) {
            auto element = m.get(i, j)->subs({ { _old, _new } });

            // Simplify elements
            auto res = nsimplify(element);
            if (res.first) 
                element = res.second;

            m.set(i, j, element);
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

auto nsimplify(SymEngine::DenseMatrix& m) -> void {
    for (size_t i = 0; i < m.nrows(); i++) {
        for (size_t j = 0; j < m.ncols(); j++) {
            auto element = m.get(i, j);

            // Simplify elements
            auto res = nsimplify(element);
            if (res.first) 
                element = res.second;

            m.set(i, j, element);
        }
    }
}


auto nsimplify(SymEngine::RCP<const SymEngine::Basic> element) -> std::pair<bool, SymEngine::RCP<const SymEngine::Basic> > {
    std::pair<bool, SymEngine::RCP<const SymEngine::Basic> > res;

    switch (element->get_type_code()) {
    case SymEngine::TypeID::SYMENGINE_REAL_DOUBLE: {
        const auto& num = dynamic_cast<const SymEngine::RealDouble*>(element.get())->as_double();
        const auto& abs_num = std::abs(num);

        auto integer_part = std::floor(abs_num);
        auto fractional_part = abs_num - integer_part;

        if (fractional_part < EPSILON) {
            res.first = true;
            res.second = SymEngine::integer(static_cast<int>(num < 0 ? -integer_part : integer_part));
        }
    } break;
    default:
        res.first = false;
        break;
    }

    return res;
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

/////////////////////////////////