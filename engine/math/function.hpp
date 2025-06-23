#pragma once

#include <cmath>
#include <stdexcept>
#include <type_traits>

/**
 * @file function.hpp
 * @brief Provides basic, constexpr-aware mathematical functions.
 * @details This file contains fundamental math utilities like `sqrt` and `abs`.
 * The functions are designed to be "context-aware," meaning they will use a
 * `constexpr` implementation during compile-time evaluation and fall back to
 * the highly optimized standard library versions (e.g., `std::sqrt`) at runtime.
 * This is achieved by checking `std::is_constant_evaluated()`.
 */

namespace pbpt {
/**
 * @brief The main namespace for the Physically Based Path Tracer project.
 */
namespace math {
/**
 * @brief Contains internal implementation details for the math library.
 * @warning Users should not directly use anything from this namespace.
 */
namespace detail {

/**
 * @internal
 * @brief A `constexpr` implementation of the square root function using the Newton-Raphson method.
 * @details This function recursively approximates the square root. It is intended for
 * compile-time use only and is called by the public `pbpt::math::sqrt` function.
 *
 * @tparam T The floating-point or integral type of the number.
 * @param x The non-negative number to find the square root of.
 * @param curr The current guess for the square root.
 * @param prev The previous guess, used to check for convergence.
 * @return The approximate square root of `x`.
 */
template<typename T>
constexpr T sqrt_newton_raphson(T x, T curr, T prev) {
    if (curr == prev) {
        return curr;
    }
    // Avoid division by zero if the initial guess is bad, though the public `sqrt` handles x=0.
    if (curr == 0) {
        return 0;
    }
    T next = 0.5 * (curr + x / curr);
    return sqrt_newton_raphson(x, next, curr);
}

} // namespace detail

/**
 * @brief A `constexpr` implementation of the absolute value function.
 * @tparam T The arithmetic type of the number.
 * @param x The number to get the absolute value of.
 * @return The absolute value of `x`.
 */
template<typename T>
constexpr T abs(T x) {
    if (x < 0) {
        return -x;
    }
    return x;
}

/**
 * @brief Calculates the square root of a number with `constexpr` support.
 * @details This function intelligently switches its implementation based on the evaluation context.
 * - **At compile-time** (inside a `constexpr` expression), it uses a `constexpr`-friendly
 * Newton-Raphson implementation (`detail::sqrt_newton_raphson`).
 * - **At runtime**, it calls the standard library's `std::sqrt`, which is typically faster and
 * more accurate due to hardware-specific optimizations (e.g., FPU instructions).
 *
 * This dual approach provides both compile-time evaluation capabilities and runtime performance.
 *
 * @tparam T The arithmetic type of the number.
 * @param x The non-negative number to find the square root of.
 * @return The square root of `x`.
 * @throw std::runtime_error If a negative `x` is passed at runtime.
 * @note Providing a negative `x` at compile-time will result in a compilation error.
 */
template<typename T>
constexpr T sqrt(T x) {
    // Handle negative input, which is invalid for real square roots.
    if (x < 0) {
        if (std::is_constant_evaluated()) {
            // Triggers a descriptive compile-time error.
            throw "Compile-time error: sqrt of negative number";
        } else {
            // Throws a standard exception at runtime.
            // Returning std::sqrt(x) would produce a quiet NaN, which can be harder to debug.
            throw std::runtime_error("sqrt of negative number");
        }
    }
    
    // Handle zero separately to avoid potential division-by-zero in Newton's method.
    if (x == 0) {
        return 0;
    }

    // Choose the implementation based on the context.
    if (std::is_constant_evaluated()) {
        // Compile-time context: call our custom constexpr implementation.
        // We start with an initial guess of x itself.
        return detail::sqrt_newton_raphson(x, x, T(0));
    } else {
        // Runtime context: call the highly optimized standard library function.
        return std::sqrt(x);
    }
}

} // namespace math
} // namespace pbpt