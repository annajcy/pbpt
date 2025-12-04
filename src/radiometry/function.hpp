/**
 * @file
 * @brief Convenience helpers for sampled spectra and RGB-to-spectrum mapping.
 */
#pragma once

#include "radiometry/color.hpp"
#include "radiometry/sampled_spectrum.hpp"
#include "radiometry/spectrum_distribution.hpp"
#include "radiometry/constant/standard_color_spaces.hpp"
#include "radiometry/constant/illuminant_spectrum.hpp"
#include "radiometry/color_spectrum_optimizer.hpp"


namespace pbpt::radiometry {

/// Create N uniformly spaced wavelength samples between lambda_min and lambda_max.
template<typename T, int N>
SampledWavelength<T, N> create_wavelength_samples_uniform() {
    math::Vector<T, N> values{};
    const T min_lambda = radiometry::lambda_min<T>;
    const T max_lambda = radiometry::lambda_max<T>;
    const T span = max_lambda - min_lambda;
    const T step = span / static_cast<T>(N);
    for (int i = 0; i < N; ++i) {
        values[i] = min_lambda + step * (static_cast<T>(i) + T(0.5));
    }
    return SampledWavelength<T, N>(values);
}

/// Create a uniform wavelength PDF over [lambda_min, lambda_max].
template<typename T, int N>
SampledPdf<T, N> create_wavelength_pdf_uniform() {
    const T span = radiometry::lambda_max<T> - radiometry::lambda_min<T>;
    const T pdf_value = T(1) / span;
    return SampledPdf<T, N>(math::Vector<T, N>::filled(pdf_value));
}

/**
 * @brief Optimize a smooth sigmoid-polynomial spectrum to match an RGB color.
 *
 * Finds coefficients of an @c RGBSigmoidPolynomialNormalized that best
 * reproduce the given RGB in the standard sRGB color space under the
 * CIE D65 illuminant.
 */
template<typename T>
radiometry::RGBSigmoidPolynomialNormalized<T> optimize_rgb_to_rsp(
    const radiometry::RGB<T>& rgb
) {
    auto optim_res = radiometry::optimize_albedo_rgb_sigmoid_polynomial(
        rgb,
        radiometry::constant::sRGB<T>,
        radiometry::constant::CIE_D65_ilum<T>
    );
    auto coeff = optim_res.normalized_coeffs;
    return radiometry::RGBSigmoidPolynomialNormalized<T>{coeff};
}

/**
 * @brief Create an albedo spectrum from an RGB color using the fitted model.
 *
 * The resulting @c RGBAlbedoSpectrumDistribution encodes a reflectance
 * spectrum whose perceived color matches the given RGB (approximately)
 * under the reference illuminant.
 */
template<typename T>
radiometry::RGBAlbedoSpectrumDistribution<T, radiometry::RGBSigmoidPolynomialNormalized> create_albedo_spectrum(
    const radiometry::RGB<T>& rgb
) {
    return radiometry::RGBAlbedoSpectrumDistribution<
        T,
        radiometry::RGBSigmoidPolynomialNormalized
    >(optimize_rgb_to_rsp(rgb));
}

}
