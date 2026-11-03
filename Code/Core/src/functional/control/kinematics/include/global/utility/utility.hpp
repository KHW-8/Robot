// SymEngine
#include <symengine/basic.h>
#include <symengine/real_double.h>
#include <symengine/symengine_rcp.h>

inline auto get_value(SymEngine::RCP<const SymEngine::Basic> element) -> double {
    double value = 0;

    switch (element->get_type_code()) {
    case SymEngine::SYMENGINE_INTEGER:
        value = dynamic_cast<const SymEngine::Integer*>(element.get())->as_int();
        break;
    case SymEngine::SYMENGINE_REAL_DOUBLE:
        value = dynamic_cast<const SymEngine::RealDouble*>(element.get())->as_double();
        break;
    default: 
        break;
    }

    return value;
}

