#include "forward_kinematics.h"

//////////* Headers *//////////

/* SymEngine */
#include <symengine/matrix.h>

/* Kinematics */
#include "math.h"

///////////////////////////////

//////////* Global Variables *//////////

auto alpha = SymEngine::symbol("α");
auto a = SymEngine::symbol("a");
auto d = SymEngine::symbol("d");
auto theta = SymEngine::symbol("Θ");

auto R_x_alpha = SymEngine::DenseMatrix(4, 4,
    {
        SymEngine::integer(1),SymEngine::integer(0),SymEngine::integer(0),SymEngine::integer(0),
        SymEngine::integer(0),SymEngine::cos(alpha),SymEngine::neg(SymEngine::sin(alpha)),SymEngine::integer(0), 
        SymEngine::integer(0),SymEngine::sin(alpha),SymEngine::cos(alpha),SymEngine::integer(0), 
        SymEngine::integer(0),SymEngine::integer(0),SymEngine::integer(0),  SymEngine::integer(1),
    }
);

auto D_x_a = SymEngine::DenseMatrix(4, 4,
    {
        SymEngine::integer(1), SymEngine::integer(0),   SymEngine::integer(0),  a,
        SymEngine::integer(0), SymEngine::integer(1),   SymEngine::integer(0),  SymEngine::integer(0),
        SymEngine::integer(0), SymEngine::integer(0),   SymEngine::integer(1),  SymEngine::integer(0),
        SymEngine::integer(0),SymEngine::integer(0),  SymEngine::integer(0),    SymEngine::integer(1),
    }
);

auto R_z_theta = SymEngine::DenseMatrix(4, 4,
    {
        SymEngine::cos(theta), SymEngine::neg(SymEngine::sin(theta)),   SymEngine::integer(0),  SymEngine::integer(0),
        SymEngine::sin(theta), SymEngine::cos(theta),   SymEngine::integer(0),  SymEngine::integer(0),
        SymEngine::integer(0), SymEngine::integer(0),   SymEngine::integer(1),  SymEngine::integer(0),
        SymEngine::integer(0),SymEngine::integer(0),  SymEngine::integer(0),    SymEngine::integer(1),
    }
);

auto D_z_d = SymEngine::DenseMatrix(4, 4,
    {
        SymEngine::integer(1), SymEngine::integer(0),   SymEngine::integer(0),  SymEngine::integer(0),
        SymEngine::integer(0), SymEngine::integer(1),   SymEngine::integer(0),  SymEngine::integer(0),
        SymEngine::integer(0), SymEngine::integer(0),   SymEngine::integer(1),  d,
        SymEngine::integer(0),SymEngine::integer(0),  SymEngine::integer(0),    SymEngine::integer(1),
    }
);


////////////////////////////////////////

/////////////* Functions */////////////

auto ForwardKinematics::calculate(std::vector<DHParameter> v) -> void {
    std::vector<SymEngine::DenseMatrix> vec;

    for (auto& param : v) {
        auto R_x_alpha_tmp = R_x_alpha;
        auto D_x_a_tmp = D_x_a;
        auto R_z_theta_tmp = R_z_theta;
        auto D_z_d_tmp = D_z_d;

        substitute(R_x_alpha_tmp, alpha, param.get_alpha());        
        substitute(R_x_alpha_tmp, alpha, param.get_alpha());        
        substitute(D_x_a_tmp, a, param.get_a());
        substitute(R_z_theta_tmp, theta, param.get_theta());
        substitute(D_z_d_tmp, d, param.get_d());


        auto T = create_identity_matrix(4, 4);
        T.mul_matrix(R_x_alpha_tmp, T);
        T.mul_matrix(D_x_a_tmp, T);
        T.mul_matrix(R_z_theta_tmp, T);
        T.mul_matrix(D_z_d_tmp, T);

        std::cout << T << std::endl;

        vec.emplace_back(T);
    }
}

///////////////////////////////////////