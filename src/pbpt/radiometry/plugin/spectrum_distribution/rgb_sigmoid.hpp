#pragma once

#include <array>

#include "pbpt/math/function.hpp"
#include "pbpt/math/polynomial.hpp"

namespace pbpt::radiometry {

/**
 * @brief Quadratic polynomial in wavelength with sigmoid shaping.
 *
 * Evaluates a quadratic polynomial in wavelength and passes it through
 * a sigmoid function to obtain a smooth, bounded spectrum.
 *
 * @tparam T Scalar type.
 */
template<typename T>
struct RGBSigmoidPolynomial {
    /// Quadratic coefficient (constant term).
    T c0;
    /// Quadratic coefficient (linear term).
    T c1;
    /// Quadratic coefficient (squared term).
    T c2;

    /// Constructs the polynomial from explicit coefficients.
    RGBSigmoidPolynomial(T c0, T c1, T c2) : c0(c0), c1(c1), c2(c2) {}
    /// Constructs the polynomial from a coefficient array [c0, c1, c2].
    RGBSigmoidPolynomial(const std::array<T, 3>& coeffs) : c0(coeffs[0]), c1(coeffs[1]), c2(coeffs[2]) {}

    /**
     * @brief Evaluates the sigmoid-shaped polynomial at a wavelength.
     *
     * A quadratic polynomial in the wavelength is evaluated and then
     * passed through a sigmoid function to keep the spectrum bounded
     * between 0 and 1.
     *
     * @param lambda Wavelength parameter (typically in nanometers).
     */
    constexpr T at(T lambda) const {
        return math::sigmoid(math::Polynomial<T>::evaluate(lambda, c0, c1, c2));
    }
};

/**
 * @brief Normalized quadratic sigmoid-polynomial for RGB basis spectra.
 *
 * Similar to RGBSigmoidPolynomial, but defined over a normalized
 * wavelength parameter t in [0,1], making the coefficients more
 * stable across different wavelength ranges.
 *
 * @tparam T Scalar type.
 */
template<typename T>
struct RGBSigmoidPolynomialNormalized {
    /// Normalized constant coefficient.
    T c0;
    /// Normalized linear coefficient.
    T c1;
    /// Normalized squared coefficient.
    T c2;

    /// Constructs the normalized polynomial from explicit coefficients.
    RGBSigmoidPolynomialNormalized(T c0, T c1, T c2) : c0(c0), c1(c1), c2(c2) {}
    /// Constructs the normalized polynomial from a coefficient array [c0, c1, c2].
    RGBSigmoidPolynomialNormalized(const std::array<T, 3>& coeffs) : c0(coeffs[0]), c1(coeffs[1]), c2(coeffs[2]) {}

    /**
     * @brief Evaluates the normalized sigmoid-polynomial at a wavelength.
     *
     * The input wavelength is first mapped to a normalized range [0,1]
     * and the quadratic polynomial and sigmoid are applied to that
     * normalized parameter.
     *
     * @param lambda Wavelength in nanometers.
     */
    constexpr T at(T lambda) const {
        const T t = (lambda - T(360)) / T(830 - 360);   // 归一化
        return math::sigmoid(math::Polynomial<T>::evaluate(t, c0, c1, c2));
    }

    /**
     * @brief Converts the normalized polynomial back to wavelength space.
     *
     * Applies the inverse change of variables to obtain coefficients
     * defined directly in terms of wavelength rather than the normalized
     * parameter.
     */
    RGBSigmoidPolynomial<T> to_unnormalized() const {
        // 转换公式基于归一化变换: t = (lambda - 360) / (830 - 360)
        // 原多项式: A*t^2 + B*t + C
        // 转换为原始wavelength域: A'*lambda^2 + B'*lambda + C'
        double c0_val = 360.0, c1_val = 1.0 / (830.0 - 360.0);
        double A = c2, B = c1, C = c0;
        double c1_squared = c1_val * c1_val;
        double c0_c1 = c0_val * c1_val;
        
        return RGBSigmoidPolynomial<T>{
            T(C - B * c0_val * c1_val + A * c0_c1 * c0_c1),
            T(B * c1_val - 2 * A * c0_val * c1_squared), 
            T(A * c1_squared)
        };
    }
};

}  // namespace pbpt::radiometry
