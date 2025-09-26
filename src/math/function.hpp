#pragma once

#include <algorithm>
#include <cmath>
#include <type_traits>

#include "math/utils.hpp"
#include "type_alias.hpp"

namespace pbpt::math {

template <typename T>
inline constexpr T epsilon_v = static_cast<T>(FLOAT_EPS);  // float 默认

template <>
inline constexpr double epsilon_v<double> = DOUBLE_EPS;  // double 特化

template <typename T>
inline constexpr T pi_v = static_cast<T>(M_PI);

template<typename T>
constexpr inline T gamma(int n) {
    T ε = std::numeric_limits<T>::epsilon();
    return (n * ε) / (1 - n * ε);
}

template <typename T>
    requires std::is_arithmetic_v<T>
constexpr T abs(T x) {
    if (x < 0) {
        return -x;
    }
    return x;
}

template <typename T>
inline constexpr bool is_zero(T a) {
    if constexpr (std::is_floating_point_v<T>) {
        return abs(static_cast<T>(a)) < epsilon_v<T>;
    } else {
        return a == 0;
    }
}

template <typename T>
    requires std::is_floating_point_v<T>
constexpr T rad2deg(T rad) {
    return rad * 180.0 / pi_v<T>;
}

template <typename T>
    requires std::is_floating_point_v<T>
constexpr T deg2rad(T deg) {
    return deg * pi_v<T> / 180.0;
}

template <typename T>
constexpr T safe_asin(T x) {
    return std::asin(std::clamp(x, T(-1), T(1)));
}

template <typename T>
constexpr T safe_acos(T x) {
    return std::acos(std::clamp(x, T(-1), T(1)));
}

template<typename T>
constexpr T pow(T x, int n) {
    if (n < 0) return 1 / pow(x, -n);
    if (n == 0) return 1;
    if (n == 1) return x;
    T half = pow(x, n / 2);
    return n % 2 == 0 ? half * half : half * half * x;
}

//TODO: implement fast_exp
template<typename T>
constexpr T fast_exp(T x) {
    return std::exp(x);
}

template<typename T>
constexpr T sigmoid(T x) {
    return 0.5 + 0.5 * x / std::sqrt(1 + x * x);
}

template<typename T>
constexpr T inverse_sigmoid(T s) {
    math::assert_if(s <= 0 && s >= 1, "s must be in (0, 1)");
    return (s - 0.5) / std::sqrt(s * (1.0 - s));
}


}  // namespace pbpt::math