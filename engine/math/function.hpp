#pragma once

#include "math/global.hpp"
#include <cmath>
#include <stdexcept>
#include <type_traits>

/**
 * @brief Define _USE_MATH_DEFINES for M_PI on MSVC, or just define pi ourselves for portability.
 */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * @file function.hpp
 * @brief Provides basic, constexpr-aware mathematical functions.
 * @details This file contains fundamental math utilities like `sqrt` and `abs`.
 * The functions are designed to be "context-aware," meaning they will use a
 * `constexpr` implementation during compile-time evaluation and fall back to
 * the highly optimized standard library versions (e.g., `std::sqrt`) at runtime.
 * This is achieved by checking `std::is_constant_evaluated()`.
 */

 /**
 * @brief The namespace for math library implementation
 */
namespace pbpt::math {
/**
 * @brief Contains internal implementation details for the math library.
 * @warning Users should not directly use anything from this namespace.
 */
namespace detail {

/**
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


// ... 已有的 sqrt_newton_raphson ...

/**
 * @brief A constexpr implementation of the sine function using a Taylor series expansion.
 * @details This is intended for compile-time use. For best precision, the input angle `x`
 * should be in the range [-PI/2, PI/2].
 * @tparam T Floating-point type.
 * @param x The angle in radians.
 * @return The sine of `x`.
 */
template <typename T>
constexpr T sin_taylor(T x) {
    // Taylor series for sin(x) = x - x^3/3! + x^5/5! - x^7/7! + ...
    // We use a limited number of terms for a balance of precision and compile time.
    T result = x;
    T power = x;
    long double fact = 1.0; // Use long double for factorial to maintain precision
    
    for (int i = 1; i < 10; ++i) { // 10 terms is a good trade-off
        power *= x * x; // From x^1 to x^3, x^3 to x^5, etc.
        fact *= (2 * i) * (2 * i + 1); // From 1! to 3!, 3! to 5!, etc.

        if (i % 2 == 1) {
            result -= power / fact; // Subtraction for x^3, x^7, etc.
        } else {
            result += power / fact; // Addition for x^5, x^9, etc.
        }
    }
    return result;
}

/**
 * @brief A constexpr implementation of the cosine function using a Taylor series expansion.
 * @details This is intended for compile-time use. For best precision, the input angle `x`
 * should be in the range [-PI/2, PI/2].
 * @tparam T Floating-point type.
 * @param x The angle in radians.
 * @return The cosine of `x`.
 */
template <typename T>
constexpr T cos_taylor(T x) {
    // Taylor series for cos(x) = 1 - x^2/2! + x^4/4! - x^6/6! + ...
    T result = 1.0;
    T power = 1.0;
    long double fact = 1.0;

    for (int i = 1; i < 10; ++i) {
        power *= x * x; // From x^0 to x^2, x^2 to x^4, etc.
        fact *= (2 * i - 1) * (2 * i); // From 1 to 2!, 2! to 4!, etc.
        
        if (i % 2 == 1) {
            result -= power / fact; // Subtraction for x^2, x^6, etc.
        } else {
            result += power / fact; // Addition for x^4, x^8, etc.
        }
    }
    return result;
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
 * @brief Converts an angle from radians to degrees.
 * @details This function converts an angle from radians to degrees.
 * @tparam T The floating-point type of the angle.
 * @param rad The angle in radians.
 * @return requires constexpr 
 */
template <typename T>
requires std::is_floating_point_v<T>
constexpr T rad2deg(T rad) {
    return rad * 180.0 / M_PI;
}

/**
 * @brief Converts an angle from degrees to radians.
 * @tparam T The floating-point type of the angle.
 * @param deg The angle in degrees.
 * @return The angle in radians.
 */
template <typename T>
requires std::is_floating_point_v<T>
constexpr T deg2rad(T deg) {
    return deg * M_PI / 180.0;
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

/**
 * @brief Calculates the sine of an angle with `constexpr` support.
 * @details At compile-time, uses a Taylor series. At runtime, calls `std::sin`.
 * The compile-time version performs range reduction to maintain precision.
 * @tparam T Floating-point type of the angle.
 * @param x The angle in radians.
 * @return The sine of `x`.
 */
template <typename T>
requires std::is_floating_point_v<T>
constexpr T sin(T x) {
    if (std::is_constant_evaluated()) {
        constexpr T pi = T(M_PI);
        constexpr T two_pi = 2 * pi;

        // Manual fmod to bring angle into [-2*PI, 2*PI] range
        T reduced_x = x;
        while (reduced_x > two_pi) { reduced_x -= two_pi; }
        while (reduced_x < -two_pi) { reduced_x += two_pi; }

        // Use trigonometric identities to map the angle to [0, PI/2] for best Taylor precision
        if (reduced_x < 0) {
            return -sin(-reduced_x); // sin(-x) = -sin(x)
        }
        if (reduced_x > pi) {
            return -sin(reduced_x - pi); // sin(x) = -sin(x-pi) for x > pi
        }
        if (reduced_x > pi / 2) {
            return sin(pi - reduced_x); // sin(x) = sin(pi-x) for x > pi/2
        }
        return detail::sin_taylor(reduced_x);
    } else {
        return std::sin(x);
    }
}

/**
 * @brief Calculates the cosine of an angle with `constexpr` support.
 * @details At compile-time, uses a Taylor series. At runtime, calls `std::cos`.
 * It can be implemented efficiently by shifting the angle and calling `pbpt::math::sin`.
 * @tparam T Floating-point type of the angle.
 * @param x The angle in radians.
 * @return The cosine of `x`.
 */
template <typename T>
requires std::is_floating_point_v<T>
constexpr T cos(T x) {
    if (std::is_constant_evaluated()) {
        // Use the identity cos(x) = sin(x + PI/2)
        return sin(x + T(M_PI) / 2.0);
    } else {
        return std::cos(x);
    }
}

/**
 * @brief Calculates the tangent of an angle with `constexpr` support.
 * @details This function is implemented as sin(x) / cos(x), automatically leveraging
 * their context-aware constexpr behavior.
 * @tparam T Floating-point type of the angle.
 * @param x The angle in radians.
 * @return The tangent of `x`.
 */
template <typename T>
requires std::is_floating_point_v<T>
constexpr T tan(T x) {
    return sin(x) / cos(x);
}

/**
 * @brief Compares two floating-point values for equality.
 * @details This function checks if the absolute difference between `a` and `b` is less than
 * a predefined epsilon (`EPSILON`). This is a conservative approach, but it's useful in
 * `constexpr` contexts where exact comparisons are not always possible.
 * @tparam T Floating-point type of the values.
 * @param a The first value.
 * @param b The second value.
 * @return `true` if `abs(a - b) < EPSILON`, `false` otherwise.
 */
template <typename T>
requires std::is_floating_point_v<T>
constexpr bool is_equal(T a, T b) {
    return abs(a - b) < EPSILON;
}

} // namespace math