/**
 * @file
 * @brief Epsilon-aware floating-point predicates and tolerance-based comparison helpers.
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <type_traits>

#include "pbpt/math/basic/algebra_concepts.hpp"
#include "pbpt/math/basic/function.hpp"

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

template <typename T>
struct Tolerance {
    T abs_tol{};
    T rel_tol{};
};

template <typename T>
constexpr Tolerance<T> default_tolerance() {
    if constexpr (std::is_floating_point_v<T>) {
        const T eps = std::numeric_limits<T>::epsilon();
        return {T(64) * eps, T(64) * eps};
    } else {
        return {T(0), T(0)};
    }
}

template <typename A, typename B>
constexpr bool is_close(A a, B b, Tolerance<promote_binary_t<A, B>> tol = default_tolerance<promote_binary_t<A, B>>()) {
    using T = promote_binary_t<A, B>;
    if constexpr (std::is_floating_point_v<T>) {
        const T aa = static_cast<T>(a);
        const T bb = static_cast<T>(b);
        const T diff = std::abs(aa - bb);
        const T scale = std::max(std::abs(aa), std::abs(bb));
        return diff <= tol.abs_tol + tol.rel_tol * scale;
    } else {
        return a == b;
    }
}

template <typename T>
constexpr bool is_zero_tol(T value, Tolerance<T> tol) {
    if constexpr (std::is_floating_point_v<T>) {
        return std::abs(value) <= tol.abs_tol;
    } else {
        return value == T(0);
    }
}

/**
 * @brief Component-wise approximate equality for any indexed math type.
 *
 * Required by the `MathObject` concept for all tuple-based types.
 */
template <typename V>
    requires HasAlgebraTraits<V> &&
             requires(const V& v, int i) { { v[i] }; }
constexpr bool is_approx(
    const V& a,
    const V& b,
    Tolerance<algebra_scalar_t<V>> tol = default_tolerance<algebra_scalar_t<V>>()) {
    for (int i = 0; i < algebra_dim_v<V>; ++i)
        if (!is_close(a[i], b[i], tol))
            return false;
    return true;
}

}  // namespace pbpt::math
