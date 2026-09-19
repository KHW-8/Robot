#include <numbers>

inline auto deg_to_rad(double degree) -> double {
    return degree * std::numbers::pi / 180;
}