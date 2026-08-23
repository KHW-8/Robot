// C STD
#include <stdint.h>
// C++ STD
#include <vector>
#include <array>

template<typename T>
auto to_byte_vector(const T& data) -> std::array<uint8_t, sizeof(T)> {
    return std::bit_cast<std::array<uint8_t, sizeof(T)>>(data);
}