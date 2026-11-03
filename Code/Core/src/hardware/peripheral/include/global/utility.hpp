#ifndef UTTILITY_HPP
#define UTTILITY_HPP

// STD
#include <cstdint>
#include <array>

template<typename T>
auto to_byte_vector(const T& data) -> std::array<uint8_t, sizeof(T)> {
    return std::bit_cast<std::array<uint8_t, sizeof(T)> >(data);
}

#endif