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

}
