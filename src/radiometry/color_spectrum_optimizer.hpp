#pragma once

#include <array>

#include "math/matrix.hpp"
#include "math/operator.hpp"
#include "math/vector.hpp"
#include "math/format.hpp"

#include "spectrum_distribution.hpp"
#include "color.hpp"
#include "color_space.hpp"

namespace pbpt::radiometry {

template<typename T>
struct RGBSigmoidPolynomialOptimizationResult {
    math::Vector<T, 3> error;
    std::array<T, 3> normalized_coeffs;
};

template<typename T, typename LuminantSpectrumType>
inline auto optimize_albedo_rgb_sigmoid_polynomial(
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

template <typename T>
struct ScaledRGB {
    RGB<T> rgb{};
    T scale = T(1);
};

template<typename T>
inline ScaledRGB<T> scale_unbounded_rgb(const RGB<T>& rgb) {
    T max_component = std::max({rgb.r(), rgb.g(), rgb.b()});
    return ScaledRGB<T>{rgb / (2 * max_component), 2 * max_component};
}

}
