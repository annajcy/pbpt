/**
 * @file
 * @brief Containers for Monte Carlo sampled spectra and wavelengths.
 */
#pragma once

#include <array>
#include <cmath>
#include <utility>
#include "math/sampling.hpp"
#include "math/vector.hpp"
#include "radiometry/constant/lambda.hpp"

namespace pbpt::radiometry {

/**
 * @brief Discrete spectrum represented by N samples.
 *
 * Typically stores radiance, reflectance or illuminant values evaluated
 * at a set of sampled wavelengths.
 *
 * @tparam T Scalar type.
 * @tparam N Number of spectral samples.
 */
template <typename T, int N>
class SampledSpectrum : public math::Vector<T, N> {
public:
    /// Default-constructs a spectrum with uninitialized or zeroed samples.
    SampledSpectrum() : math::Vector<T, N>() {}
    /**
     * @brief Constructs a sampled spectrum from an existing vector of samples.
     *
     * The vector entries are interpreted as spectral values at the
     * corresponding sampled wavelengths.
     */
    SampledSpectrum(const math::Vector<T, N>& vec) : math::Vector<T, N>(vec) {}
};

/**
 * @brief N sampled wavelengths in nanometers.
 *
 * Each entry usually lies in the range covered by lambda_min/lambda_max.
 */
template<typename T, int N> 
class SampledWavelength : public math::Vector<T, N> {
public:
    /// Default-constructs an empty wavelength container.
    SampledWavelength() : math::Vector<T, N>() {}
    /**
     * @brief Constructs sampled wavelengths from an existing vector.
     *
     * The vector entries are interpreted as wavelengths in nanometers.
     */
    SampledWavelength(const math::Vector<T, N>& vec) : math::Vector<T, N>(vec) {}
};

/**
 * @brief Sampling PDF values corresponding to sampled wavelengths.
 *
 * Used together with @c SampledSpectrum and @c SampledWavelength to form
 * unbiased Monte Carlo estimators over the spectrum.
 */
template<typename T, int N>
class SampledPdf : public math::Vector<T, N> {
public:
    /// Default-constructs an empty PDF container.
    SampledPdf() : math::Vector<T, N>() {}
    /**
     * @brief Constructs sampling PDF values from an existing vector.
     *
     * Each entry stores the probability density associated with the
     * corresponding sampled wavelength.
     */
    SampledPdf(const math::Vector<T, N>& vec) : math::Vector<T, N>(vec) {}
};

/**
 * @brief Samples a wavelength uniformly from the visible spectrum [360nm, 830nm].
 *
 * @tparam T Numeric type.
 * @param u A uniform random sample in [0, 1).
 * @return T The sampled wavelength in nanometers.
 */
template<typename T>
inline T sample_uniform_wavelength(const T u) {
    return math::sample_uniform(u, radiometry::constant::lambda_min<T>, radiometry::constant::lambda_max<T>);
}

/**
 * @brief Computes the Probability Density Function (PDF) for uniform wavelength sampling.
 *
 * Since the sampling is uniform over [360, 830], the PDF is constant:
 * p(lambda) = 1 / (830 - 360) approx 0.002127
 *
 * @tparam T Numeric type.
 * @param lambda The wavelength in nanometers.
 * @return T The PDF value.
 */
template <typename T>
inline T sample_uniform_wavelength_pdf(const T lambda) {
    return math::sample_uniform_pdf(lambda, radiometry::constant::lambda_min<T>, radiometry::constant::lambda_max<T>);
}

/**
 * @brief Helper to map an array of random numbers to uniform wavelengths.
 *
 * @tparam T Numeric type.
 * @tparam N Number of samples.
 * @param us An array of N uniform random samples in [0, 1).
 * @return SampledWavelength<T, N> The corresponding array of wavelengths.
 */
template <typename T, int N>
inline SampledWavelength<T, N> sample_uniform_wavelengths(const std::array<T, N>& us) {
    SampledWavelength<T, N> wavelengths;
    for (int i = 0; i < N; ++i) {
        wavelengths[i] = sample_uniform_wavelength(us[i]);
    }
    return wavelengths;
}

/**
 * @brief Generates N stratified uniform wavelength samples from a single random number.
 *
 * Combines stratification (splitting [0,1) into strata) with uniform mapping.
 *
 * @tparam T Numeric type.
 * @tparam N Number of samples.
 * @param u A single uniform random variable in [0, 1).
 * @return SampledWavelength<T, N> An array of N stratified wavelengths.
 */
template<typename T, int N>
inline SampledWavelength<T, N> sample_uniform_wavelengths_stratified(T u) {
    auto us = math::generate_strified_array<T, N>(u);
    return sample_uniform_wavelengths<T, N>(us);
}

/**
 * @brief Computes the PDF for a collection of uniformly sampled wavelengths.
 *
 * @tparam T Numeric type.
 * @tparam N Number of samples.
 * @param wavelengths The array of wavelengths.
 * @return SampledPdf<T, N> The array of PDF values.
 */
template <typename T, int N>
inline SampledPdf<T, N> sample_uniform_wavelengths_pdf(const SampledWavelength<T, N>& wavelengths) {
    SampledPdf<T, N> pdfs;
    for (int i = 0; i < N; ++i) {
        pdfs[i] = sample_uniform_wavelength_pdf(wavelengths[i]);
    }
    return pdfs;
}

/**
 * @brief Samples a wavelength using importance sampling based on human visual sensitivity.
 *
 * Uses the inverse CDF method with a fitted distribution that approximates the 
 * CIE Y response curve (and others) to reduce perceptual noise. The distribution 
 * peaks around 538nm (Green).
 *
 * Analytical inverse CDF formula:
 * lambda = 538 - (1 / 0.0072) * atanh(0.85691062 - 1.82750197 * u)
 *
 * @tparam T Numeric type.
 * @param u A uniform random sample in [0, 1).
 * @return T The sampled wavelength in nanometers.
 */
template<typename T>
inline T sample_visible_wavelengths(T u) {
    return T(538) - T(138.888889) * std::atanh(T(0.85691062) - T(1.82750197) * u);
}

/**
 * @brief Computes the PDF for the visual-sensitivity based sampling distribution.
 *
 * The PDF is defined as:
 * p(lambda) proportional to 1 / cosh^2(0.0072 * (lambda - 538))
 *
 * Returns 0 if lambda is outside the visible range [360, 830].
 *
 * @tparam T Numeric type.
 * @param lambda The wavelength in nanometers.
 * @return T The probability density value.
 */
template<typename T>
inline T sample_visible_wavelengths_pdf(const T lambda) {
    if (lambda < radiometry::constant::lambda_min<T> || lambda > radiometry::constant::lambda_max<T>) {
        return T(0);
    }

    T cosh_val = std::cosh(T(0.0072) * (lambda - T(538)));
    // PDF calculation: Constant / cosh^2(...)
    return T(0.0039398042) / (cosh_val * cosh_val);
}

/**
 * @brief Helper to map an array of random numbers to importance-sampled wavelengths.
 *
 * @tparam T Numeric type.
 * @tparam N Number of samples.
 * @param us An array of N uniform random samples in [0, 1).
 * @return SampledWavelength<T, N> The corresponding array of wavelengths.
 */
template <typename T, int N>
inline SampledWavelength<T, N> sample_visible_wavelengths(const std::array<T, N>& us) {
    SampledWavelength<T, N> wavelengths;
    for (int i = 0; i < N; ++i) {
        wavelengths[i] = sample_visible_wavelengths(us[i]);
    }
    return wavelengths;
}

/**
 * @brief Generates N stratified importance-sampled wavelength samples from a single random number.
 *
 * This is the preferred method for spectral rendering. It ensures that the wavelengths 
 * cover the visible spectrum thoroughly according to their visual importance, 
 * minimizing color noise.
 *
 * @tparam T Numeric type.
 * @tparam N Number of samples.
 * @param u A single uniform random variable in [0, 1).
 * @return SampledWavelength<T, N> An array of N stratified, importance-sampled wavelengths.
 */
template<typename T, int N>
inline SampledWavelength<T, N> sample_visible_wavelengths_stratified(T u) {
    auto us = math::generate_strified_array<T, N>(u);
    return sample_visible_wavelengths<T, N>(us);
}

/**
 * @brief Computes the PDF for a collection of importance-sampled wavelengths.
 *
 * @tparam T Numeric type.
 * @tparam N Number of samples.
 * @param wavelengths The array of wavelengths.
 * @return SampledPdf<T, N> The array of PDF values.
 */
template <typename T, int N>
inline SampledPdf<T, N> sample_visible_wavelengths_pdf(const SampledWavelength<T, N>& wavelengths) {
    SampledPdf<T, N> pdfs;
    for (int i = 0; i < N; ++i) {
        pdfs[i] = sample_visible_wavelengths_pdf(wavelengths[i]);
    }
    return pdfs;
};

};
