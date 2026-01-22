/**
 * @file
 * @brief Optimization utilities for fitting RGB colors to smooth spectra.
 */
#pragma once

#include <array>

#include "math/matrix.hpp"
#include "math/operator.hpp"
#include "math/vector.hpp"
#include "math/format.hpp"

#include "color.hpp"
#include "color_space.hpp"
#include "constant/illuminant_spectrum.hpp"
#include "constant/standard_color_spaces.hpp"
#include "radiometry/plugin/spectrum_distribution/rgb_sigmoid.hpp"
#include "radiometry/plugin/spectrum_distribution/rgb_spectra.hpp"

namespace pbpt::radiometry {

/**
 * @brief Result of optimizing RGB sigmoid polynomial coefficients.
 *
 * Contains the LAB error vector between target and fitted color and
 * the normalized polynomial coefficients that define the spectrum.
 *
 * @tparam T Scalar type.
 */
template<typename T>
struct RGBSigmoidPolynomialOptimizationResult {
    /// LAB difference between target color and fitted color (L*, a*, b*).
    math::Vector<T, 3> error;
    /// Normalized sigmoid-polynomial coefficients that define the fitted spectrum.
    std::array<T, 3> normalized_coeffs;
};

/**
 * @brief Fits an albedo RGB sigmoid-polynomial spectrum to a target RGB color.
 *
 * The optimization works in CIE LAB space:
 * - The target RGB color is converted to XYZ and then to LAB using the
 *   provided RGB color space and its white point.
 * - A parametric reflectance spectrum is defined by a normalized
 *   RGB sigmoid polynomial with three coefficients.
 * - For a given coefficient vector, the corresponding reflectance is
 *   converted to XYZ under the reference illuminant and then to LAB.
 * - The LAB difference between target and current color is used as
 *   the 3D error vector.
 *
 * A simple Newton-style iteration is performed: the Jacobian matrix
 * of partial derivatives of the LAB error with respect to the three
 * coefficients is approximated by symmetric finite differences. The
 * resulting 3x3 linear system is solved and the coefficients are
 * updated by a scaled step. Iteration stops when all LAB errors are
 * below @p error_threshold or when @p max_iterations is reached.
 *
 * @tparam T                  Scalar type.
 * @tparam LuminantSpectrumType Spectrum type used for the reference illuminant.
 * @param target_rgb          Target RGB color to match.
 * @param color_space         RGB color space describing primaries and white.
 * @param reference_luminant_spectrum Illuminant used to light the reflectance.
 * @param max_iterations      Maximum number of Newton iterations.
 * @param learning_rate       Step size scaling applied to the Newton update.
 * @param delta_x             Finite-difference step for Jacobian estimation.
 * @param error_threshold     Per-channel LAB error threshold for convergence.
 * @param initial_guess       Initial coefficients for the sigmoid polynomial.
 * @param verbose             If true, prints iteration diagnostics to stdout.
 *
 * @return Optimization result containing the final LAB error and coefficients.
 */
template<typename T, typename LuminantSpectrumType>
inline RGBSigmoidPolynomialOptimizationResult<T> optimize_albedo_rgb_sigmoid_polynomial(
    const RGB<T>& target_rgb, 
    const RGBColorSpace<T>& color_space,
    const LuminantSpectrumType& reference_luminant_spectrum,
    int max_iterations = 300, 
    double learning_rate = 1.0,
    double delta_x = 1e-4,
    double error_threshold = 1e-2, 
    const std::array<T, 3>& initial_guess = {0.0, 0.0, 0.0},
    bool verbose = false
) {

    if (verbose) {
        std::cout << "Starting optimization for target RGB: " << target_rgb << std::endl;
    }
    XYZ<T> target_xyz = color_space.to_xyz(target_rgb);
    if (verbose) {
        std::cout << "Target XYZ: " << target_xyz << std::endl;
    }
    LAB<T> target_lab = LAB<T>::from_xyz(target_xyz, color_space.white_point());
    if (verbose) {
        std::cout << "Target LAB: " << target_lab << std::endl;
    }

    auto eval_lab = [&](const std::array<T,3>& c) -> LAB<T> {
        RGBAlbedoSpectrumDistribution<T, RGBSigmoidPolynomialNormalized> albedo({c[0], c[1], c[2]});
        XYZ<T> xyz = XYZ<T>::from_reflectance(albedo, reference_luminant_spectrum);
        return LAB<T>::from_xyz(xyz, color_space.white_point());
    };

    auto eval_error = [&](const LAB<T>& lab) -> math::Vector<T, 3> {
        T dL = target_lab.L() - lab.L();
        T da = target_lab.a() - lab.a();
        T db = target_lab.b() - lab.b();
        return math::Vector<T, 3>{dL, da, db};
    };

    auto eval_jacobian = [&](const std::array<T,3>& c) -> math::Matrix<T, 3, 3> {
        const T epsilon = delta_x;
        math::Matrix<T, 3, 3> J;
        for (int i = 0; i < 3; i++) {
            std::array<T, 3> c_plus = c;
            std::array<T, 3> c_minus = c;
            c_plus[i] += epsilon;
            c_minus[i] -= epsilon;
            auto err_plus = eval_error(eval_lab(c_plus));
            auto err_minus = eval_error(eval_lab(c_minus));
            J[0][i] = (err_plus[0] - err_minus[0]) / (2 * epsilon);
            J[1][i] = (err_plus[1] - err_minus[1]) / (2 * epsilon);
            J[2][i] = (err_plus[2] - err_minus[2]) / (2 * epsilon);
        }
        return J;
    };

    bool converged = false;
    std::array<T, 3> coeffs = initial_guess;
    auto error = math::Vector<T, 3>::filled(std::numeric_limits<double>::max());
    for (int i = 0; i < max_iterations; i ++) {
        auto object_lab = eval_lab(coeffs);
        //std::cout << "Object LAB: " << object_lab << std::endl;
        error = eval_error(object_lab);

        if (verbose) {
            std::cout << "Iteration " << i << ": Error = " << error
                      << ", Coeffs = (" << coeffs[0] << ", " << coeffs[1] << ", " << coeffs[2] << ")\n";
        }

        if ([&]() -> bool {
            for (int j = 0; j < 3; j++) {
                if (!math::is_less_equal(math::abs(error[j]), error_threshold)) {
                    return false;
                }
            }
            return true;
        }()) {
            if (verbose) {
                std::cout << "Converged after " << i << " iterations.\n";
                std::cout << "Final Coeffs = (" << coeffs[0] << ", " << coeffs[1] << ", " << coeffs[2] << ")\n";
            } else {
                std::cout << "converged after " << i << " iterations.\n";
            }
            converged = true;
            return RGBSigmoidPolynomialOptimizationResult<T>{error, coeffs};
        }

        auto J = eval_jacobian(coeffs);
        if (verbose) {
            std::cout << "Jacobian:\n" << J << std::endl;
        }

        // Solve the linear system J * delta = error and update coeffs
        auto delta = J.inversed_rref() * error;
        coeffs[0] -= delta[0] * learning_rate;
        coeffs[1] -= delta[1] * learning_rate;
        coeffs[2] -= delta[2] * learning_rate;
    }

    if (!converged) {
        std::cout << "Did not converge after " << max_iterations << " iterations.\n";
        std::cout << "Final Error = " << error << "\n";
        std::cout << "Final Coeffs = (" << coeffs[0] << ", " << coeffs[1] << ", " << coeffs[2] << ")\n";
    }

    return RGBSigmoidPolynomialOptimizationResult<T>{error, coeffs};
}

/**
 * @brief RGB value scaled into a displayable range plus an overall factor.
 *
 * Useful for representing unbounded RGB triplets by factoring out a
 * positive scale and storing a bounded RGB color.
 */
template <typename T>
struct ScaledRGB {
    /// Bounded RGB color after scaling.
    RGB<T> rgb{};
    /// Positive scale factor such that rgb * scale reconstructs the original color.
    T scale = T(1);
};

/**
 * @brief Factorizes an unbounded RGB color into a bounded RGB and a scale.
 *
 * The maximum channel component is used to define a scale factor so
 * that the scaled RGB components remain within a comfortable range
 * (here the maximum becomes 0.5). The original color is recovered as
 *   rgb_scaled * scale.
 *
 * @param rgb Input (potentially unbounded) RGB color.
 * @return ScaledRGB containing a bounded RGB value and a positive scale.
 */
template<typename T>
inline ScaledRGB<T> scale_unbounded_rgb(const RGB<T>& rgb) {
    T max_component = std::max({rgb.r(), rgb.g(), rgb.b()});
    return ScaledRGB<T>{rgb / (2 * max_component), 2 * max_component};
}


/**
 * @brief Optimize a smooth sigmoid-polynomial spectrum to match an RGB color.
 *
 * Finds coefficients of an @c RGBSigmoidPolynomialNormalized that best
 * reproduce the given RGB in the standard sRGB color space under the
 * CIE D65 illuminant.
 */
template<typename T>
RGBSigmoidPolynomialNormalized<T> optimize_rgb_to_rsp(
    const RGB<T>& rgb
) {
    auto optim_res = optimize_albedo_rgb_sigmoid_polynomial(
        rgb,
        constant::sRGB<T>,
        constant::CIE_D65_ilum<T>
    );
    auto coeff = optim_res.normalized_coeffs;
    return RGBSigmoidPolynomialNormalized<T>{coeff};
}

/**
 * @brief Create an albedo spectrum from an sRGB color using the fitted model by optimization.
 *
 * The resulting @c RGBAlbedoSpectrumDistribution encodes a reflectance
 * spectrum whose perceived color matches the given RGB (approximately)
 * under the reference illuminant.
 */
template<typename T>
RGBAlbedoSpectrumDistribution<T, RGBSigmoidPolynomialNormalized> create_srgb_albedo_spectrum_by_optimization(
    const RGB<T>& rgb
) {
    return RGBAlbedoSpectrumDistribution<T, RGBSigmoidPolynomialNormalized>(
        optimize_rgb_to_rsp(rgb)
    );
}


}
