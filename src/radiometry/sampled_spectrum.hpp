/**
 * @file
 * @brief Containers for Monte Carlo sampled spectra and wavelengths.
 */
#pragma once

#include "math/vector.hpp"

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

};
