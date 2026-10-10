#ifndef DH_PARAMETER_H
#define DH_PARAMETER_H

#include <symengine/matrix.h>

class  DHParameter {
public:
    DHParameter(SymEngine::RCP<const SymEngine::Basic> alpha, SymEngine::RCP<const SymEngine::Basic> a, SymEngine::RCP<const SymEngine::Basic> d, SymEngine::RCP<const SymEngine::Basic> theta);

public:
    auto get_alpha() -> SymEngine::RCP<const SymEngine::Basic> { return this->alpha; }
    auto get_a() -> SymEngine::RCP<const SymEngine::Basic> { return this->a; }
    auto get_d() -> SymEngine::RCP<const SymEngine::Basic> { return this->d; }
    auto get_theta() -> SymEngine::RCP<const SymEngine::Basic> { return this->theta; }

    auto print() -> void;

private:
    SymEngine::RCP<const SymEngine::Basic> alpha;
    SymEngine::RCP<const SymEngine::Basic> a;
    SymEngine::RCP<const SymEngine::Basic> d;
    SymEngine::RCP<const SymEngine::Basic> theta;
};


#endif