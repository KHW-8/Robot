// C STD
#include <stdint.h>
// C++ STD
#include <vector>

template<std::integral T>
auto to_byte_vector(const T& data) -> std::vector<uint8_t> {
    std::vector<uint8_t> v;
    for (size_t i = 0; i < sizeof(T); i++) 
        v.emplace_back(static_cast<uint8_t>(data >> (8 * i)));

    return v;
}