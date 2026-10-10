#include "forward_kinematics.h"

//////////* Headers *//////////

/* Core */
//// Kinematics
#include "math.h"

///////////////////////////////

//////////* Global Variables *//////////

auto R_x_alpha = SymEngine::DenseMatrix(4, 4,
    { 
        SymEngine::integer(1),      SymEngine::integer(0),                           SymEngine::integer(0),                                             SymEngine::integer(0),
        SymEngine::integer(0),      SymEngine::cos(SymEngine::symbol("α")),   SymEngine::neg(SymEngine::sin(SymEngine::symbol("α"))), SymEngine::integer(0), 
        SymEngine::integer(0),      SymEngine::sin(SymEngine::symbol("α")),  SymEngine::cos(SymEngine::symbol("α")),                    SymEngine::integer(0), 
        SymEngine::integer(0),    SymEngine::integer(0),                          SymEngine::integer(0),                                            SymEngine::integer(1),
    }
);

auto D_x_a = SymEngine::DenseMatrix(4, 4,
    {
        SymEngine::integer(1),  SymEngine::integer(0),   SymEngine::integer(0),   SymEngine::symbol("a"),
        SymEngine::integer(0),  SymEngine::integer(1),   SymEngine::integer(0),   SymEngine::integer(0),
        SymEngine::integer(0),  SymEngine::integer(0),  SymEngine::integer(1),  SymEngine::integer(0),
        SymEngine::integer(0),SymEngine::integer(0),  SymEngine::integer(0),  SymEngine::integer(1),
    }
);

auto R_z_theta = SymEngine::DenseMatrix(4, 4,
    {
        SymEngine::cos(SymEngine::symbol("Θ")),  SymEngine::neg(SymEngine::sin(SymEngine::symbol("Θ"))),   SymEngine::integer(0),    SymEngine::integer(0),
        SymEngine::sin(SymEngine::symbol("Θ")),  SymEngine::cos(SymEngine::symbol("Θ")),                      SymEngine::integer(0),    SymEngine::integer(0),
        SymEngine::integer(0),                          SymEngine::integer(0),                                              SymEngine::integer(1),   SymEngine::integer(0),
        SymEngine::integer(0),                        SymEngine::integer(0),                                              SymEngine::integer(0),   SymEngine::integer(1),
    }
);

auto D_z_d = SymEngine::DenseMatrix(4, 4,
    {
        SymEngine::integer(1),  SymEngine::integer(0),   SymEngine::integer(0),    SymEngine::integer(0),
        SymEngine::integer(0),  SymEngine::integer(1),   SymEngine::integer(0),    SymEngine::integer(0),
        SymEngine::integer(0),  SymEngine::integer(0),  SymEngine::integer(1),   SymEngine::symbol("d"),
        SymEngine::integer(0),SymEngine::integer(0),  SymEngine::integer(0),   SymEngine::integer(1),
    }
);


////////////////////////////////////////

/////////////* Functions */////////////

auto ForwardKinematics::generate_transformation_operators(std::vector<DHParameter> v) -> std::vector<SymEngine::DenseMatrix> {
    std::vector<SymEngine::DenseMatrix> vec;

    for (auto& param : v) {
        auto R_x_alpha_tmp = R_x_alpha;
        auto D_x_a_tmp = D_x_a;
        auto R_z_theta_tmp = R_z_theta;
        auto D_z_d_tmp = D_z_d;

        substitute(R_x_alpha_tmp, SymEngine::symbol("α"), param.get_alpha());        
        substitute(R_x_alpha_tmp, SymEngine::symbol("α"), param.get_alpha());        
        substitute(D_x_a_tmp, SymEngine::symbol("a"), param.get_a());
        substitute(R_z_theta_tmp, SymEngine::symbol("Θ"), param.get_theta());
        substitute(D_z_d_tmp, SymEngine::symbol("d"), param.get_d());


        auto T = R_x_alpha_tmp;
        T.mul_matrix(D_x_a_tmp, T);
        T.mul_matrix(R_z_theta_tmp, T);
        T.mul_matrix(D_z_d_tmp, T);

        vec.emplace_back(T);
    }

    return vec;
}


auto ForwardKinematics::generate_equation(std::vector<SymEngine::DenseMatrix> v) -> SymEngine::DenseMatrix {
    return product(v);
}

auto ForwardKinematics::generate_equation(std::vector<DHParameter> v) -> SymEngine::DenseMatrix {
    std::vector<SymEngine::DenseMatrix> vec;

    for (auto& param : v) {
        auto R_x_alpha_tmp = R_x_alpha;
        auto D_x_a_tmp = D_x_a;
        auto R_z_theta_tmp = R_z_theta;
        auto D_z_d_tmp = D_z_d;

        substitute(R_x_alpha_tmp, SymEngine::symbol("α"), param.get_alpha());        
        substitute(R_x_alpha_tmp, SymEngine::symbol("α"), param.get_alpha());        
        substitute(D_x_a_tmp, SymEngine::symbol("a"), param.get_a());
        substitute(R_z_theta_tmp, SymEngine::symbol("Θ"), param.get_theta());
        substitute(D_z_d_tmp, SymEngine::symbol("d"), param.get_d());


        auto T = R_x_alpha_tmp;
        T.mul_matrix(D_x_a_tmp, T);
        T.mul_matrix(R_z_theta_tmp, T);
        T.mul_matrix(D_z_d_tmp, T);

        vec.emplace_back(T);
    }

    auto T = product(vec);

    return T;
}

///////////////////////////////////////