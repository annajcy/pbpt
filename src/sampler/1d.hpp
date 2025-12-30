/**
 * @file
 * @brief Sampling routines for 1D/2D intervals, disks, spheres and hemispheres.
 */
#pragma once

#include <array>
#include "math/function.hpp"
#include "math/point.hpp"
#include "math/vector.hpp"

namespace pbpt::sampler {

/**
 * @brief Sample a value uniformly distributed in [a, b].
 * @tparam T Numeric type.
 * @param u Uniform random variable in [0, 1].
 * @param a Lower bound of the interval.
 * @param b Upper bound of the interval.
 * @return T Sampled value in [a, b].
 */
template<typename T>
inline T sample_uniform(T u, T a = T(0), T b = T(1)) {
    return a + (b - a) * u;
}

/**
 * @brief Compute the probability density function for uniform sampling in [a, b].
 * The PDF is the reciprocal of the length of the interval.
 * @tparam T Numeric type.
 * @param x Value to evaluate the PDF at.
 * @param a Lower bound of the interval.
 * @param b Upper bound of the interval.
 * @return T Probability density function value.
 */
template<typename T>
inline T sample_uniform_pdf(T x, T a = T(0), T b = T(1)) {
    if (x < a || x > b) {
        return T(0);
    }
    return 1.0 / (b - a);
}

/**
 * @brief Sample a value from a tent distribution centered at 0 with range [-r, r].
 * The symmetric tent PDF is f(x) = (r - |x|) / r^2 for |x| ≤ r and zero elsewhere. Integrating
 * this PDF yields the CDF:
 *
 *   F(x) = ((x + r)^2) / (2 r^2),     x ∈ [-r, 0]
 *   F(x) = 1 - ((r - x)^2) / (2 r^2), x ∈ [0, r]
 *
 * Since F(0) = 1/2, inverting the CDF leads to two analytic branches governed by whether the
 * uniform variate u falls below 0.5. Solving F(x) = u on each branch produces
 * x = -r + r * sqrt(2u) for u < 0.5 and x = r - r * sqrt(2(1 - u)) for u ≥ 0.5.
 *
 * @tparam T Numeric type.
 * @param u Uniform random variable in [0, 1].
 * @param r Range of the tent distribution.
 * @return T Sampled value from the tent distribution.
 */
template <typename T>
inline T sample_tent(T u, T r = 1.0) {
    if (u < 0.5) {
        return -r + r * std::sqrt(2.0 * u);
    } else {
        return  r - r * std::sqrt(2.0 * (1.0 - u));
    }
}

/**
 * @brief Compute the probability density function for tent distribution centered at 0 with range [-r, r].
 * The PDF is triangular with unit area, reaching f(0) = 1 / r at the peak and dropping linearly to 0 at ±r,
 * which gives the closed form f(x) = (r - |x|) / r^2 for |x| ≤ r.
 * @tparam T Numeric type.
 * @param x Value to evaluate the PDF at.
 * @param r Range of the tent distribution.
 * @return T Probability density function value.
 */
template <typename T>
inline T sample_tent_pdf(T x, T r = 1.0) {
    if (std::abs(x) > r) {
        return T(0);
    }
    return 1.0 / r - std::abs(x) / (r * r);
}

/**
 * @brief Approximates the inverse error function.
 * This is a standard rational approximation used in graphics (e.g., PBRT, Mitsuba).
 * Maximum error is approx 1.2e-7.
 */
template <typename T>
inline T erf_inv(T x) {
    T w, p;
    x = std::clamp(x, T(-0.99999), T(0.99999));
    w = -std::log((1.0 - x) * (1.0 + x));

    if (w < 5.000000) {
        w = w - 2.500000;
        p = 2.81022636e-08;
        p = 3.43273939e-07 + p * w;
        p = -3.5233877e-06 + p * w;
        p = -4.39150654e-06 + p * w;
        p = 0.00021858087 + p * w;
        p = -0.00125372503 + p * w;
        p = -0.00417768164 + p * w;
        p = 0.246640727 + p * w;
        p = 1.50140941 + p * w;
    } else {
        w = std::sqrt(w) - 3.000000;
        p = -0.000200214257;
        p = 0.000100950558 + p * w;
        p = 0.00134934322 + p * w;
        p = -0.00367342844 + p * w;
        p = 0.00573950773 + p * w;
        p = -0.0076224613 + p * w;
        p = 0.00943887047 + p * w;
        p = 1.00167406 + p * w;
        p = 2.83297682 + p * w;
    }
    return p * x;
}

/**
 * @brief Sample a value from a Gaussian (Normal) distribution.
 * Uses the Inverse CDF method: x = mean + stddev * sqrt(2) * erf_inv(2u - 1).
 *
 * @tparam T Numeric type.
 * @param u Uniform random variable in [0, 1].
 * @param mean Mean (center) of the distribution.
 * @param stddev Standard deviation of the distribution.
 * @return T Sampled value.
 */
template<typename T>
inline T sample_gaussian(T u, T mean = T(0), T stddev = T(1)) {
    // Map u from [0, 1] to [-1, 1] for erf_inv
    T x = 2 * u - 1;
    // Apply inverse CDF formula
    T val = std::sqrt(T(2)) * erf_inv(x);
    // Scale and shift
    return mean + stddev * val;
}

/**
 * @brief Compute the probability density function for a Gaussian distribution.
 * Formula: f(x) = (1 / (sigma * sqrt(2*pi))) * exp(-0.5 * ((x - mu) / sigma)^2)
 *
 * @tparam T Numeric type.
 * @param x Value to evaluate the PDF at.
 * @param mean Mean (center) of the distribution.
 * @param stddev Standard deviation of the distribution.
 * @return T Probability density function value.
 */
template<typename T>
inline T sample_gaussian_pdf(T x, T mean = T(0), T stddev = T(1)) {
    constexpr T inv_sqrt_2pi = 0.39894228040143267794; // 1 / sqrt(2 * pi)
    
    if (stddev <= T(0)) return T(0); // Avoid division by zero

    T t = (x - mean) / stddev;
    return (inv_sqrt_2pi / stddev) * std::exp(-0.5 * t * t);
}

}
