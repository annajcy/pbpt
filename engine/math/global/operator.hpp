#pragma once

#include "type_alias.hpp"

#include <type_traits>

namespace pbpt::math {

template <typename T>
requires std::is_floating_point_v<T>
constexpr bool is_equal(T a, T b) {
    return abs(a - b) < EPSILON;
}

template <typename T>
requires std::is_integral_v<T>
constexpr bool is_equal(T a, T b) {
    return a == b;
}

template <typename T>
constexpr bool is_not_equal(T a, T b) {
    return !is_equal(a, b);
}

template <typename T>
requires std::is_floating_point_v<T>
constexpr bool is_greater(T a, T b) {
    return a - b > EPSILON;
}

template <typename T>
requires std::is_integral_v<T>
constexpr bool is_greater(T a, T b) {
    return a > b;
}

template <typename T>
constexpr bool is_less_equal(T a, T b) {
    return !is_greater(a, b);
}

template <typename T>
requires std::is_floating_point_v<T>
constexpr bool is_less(T a, T b) {
    return b - a > EPSILON;
}

template <typename T>
requires std::is_integral_v<T>
constexpr bool is_less(T a, T b) {
    return b > a;
}

template <typename T>
constexpr bool is_greater_equal(T a, T b) {
    return !is_less(a, b);
}

}