// STD
#include <cstddef>
#include <cstdint>
#include <array>
#include <vector>
// SymEngine
#include <symengine/basic.h>
#include <symengine/real_double.h>
#include <symengine/symengine_rcp.h>

inline auto generate_checksum(const std::vector<uint8_t>& vector) -> uint8_t {
    size_t sum = 0;

    for (const auto& data : vector)
        sum += data;

    return (uint8_t)(~sum);
}

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

template<typename T>
auto to_byte_vector(const T& data) -> std::array<uint8_t, sizeof(T)> {
    return std::bit_cast<std::array<uint8_t, sizeof(T)> >(data);
}