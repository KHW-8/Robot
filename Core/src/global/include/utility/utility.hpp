// C STD
#include <cstddef>
#include <cstdint>
// C++ STD
#include <array>
#include <vector>

inline auto generate_checksum(const std::vector<uint8_t>& vector) -> uint8_t {
    size_t sum = 0;

    for (const auto& data : vector)
        sum += data;

    return (uint8_t)(~sum);
}

template<typename T>
auto to_byte_vector(const T& data) -> std::array<uint8_t, sizeof(T)> {
    return std::bit_cast<std::array<uint8_t, sizeof(T)> >(data);
}