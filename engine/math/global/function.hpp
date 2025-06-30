#pragma once

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

template<typename T>
requires std::is_floating_point_v<T>
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

template <typename T>
requires std::is_floating_point_v<T>
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

template <typename T>
requires std::is_floating_point_v<T>
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

template <typename T>
requires std::is_arithmetic_v<T>
constexpr T abs(T x) {
    if (x < 0) {
        return -x;
    }
    return x;
}


template <typename T>
requires std::is_floating_point_v<T>
constexpr T rad2deg(T rad) {
    return rad * 180.0 / M_PI;
}

template <typename T>
requires std::is_floating_point_v<T>
constexpr T deg2rad(T deg) {
    return deg * M_PI / 180.0;
}

template<typename T>
requires std::is_floating_point_v<T>
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

template <typename T>
requires std::is_floating_point_v<T>
constexpr T tan(T x) {
    return sin(x) / cos(x);
}

} // namespace math