/**
 * @file
 * @brief Comparison helpers with epsilon-aware floating-point predicates.
 */
#pragma once

#include "function.hpp"

namespace pbpt::math {

/**
 * @brief Equality comparison with epsilon tolerance for floats.
 */
template <typename A, typename B>
inline constexpr bool is_equal(A a, B b) {
    using T = std::common_type_t<A, B>;
    if constexpr (std::is_floating_point_v<T>) {
        return pbpt::math::abs(static_cast<T>(a) - static_cast<T>(b)) < epsilon_v<T>;
    } else {
        return a == b;
    }
}

/// @brief Negation of is_equal.
template <typename A, typename B>
inline constexpr bool is_not_equal(A a, B b) {
    return !is_equal(a, b);
}

/**
 * @brief Greater-than comparison aware of floating-point epsilon.
 */
template <typename A, typename B>
inline constexpr bool is_greater(A a, B b) {
    using T = std::common_type_t<A, B>;
    if constexpr (std::is_floating_point_v<T>) {
        return static_cast<T>(a) - static_cast<T>(b) > epsilon_v<T>;
    } else {
        return a > b;
    }
}

/// @brief Less-than comparison aware of floating-point epsilon.
template <typename A, typename B>
inline constexpr bool is_less(A a, B b) {
    return is_greater(b, a);
}

/// @brief Less-or-equal comparison aware of floating-point epsilon.
template <typename A, typename B>
inline constexpr bool is_less_equal(A a, B b) {
    return !is_greater(a, b);
}

/// @brief Greater-or-equal comparison aware of floating-point epsilon.
template <typename A, typename B>
inline constexpr bool is_greater_equal(A a, B b) {
    return !is_less(a, b);
}

}  // namespace pbpt::math
