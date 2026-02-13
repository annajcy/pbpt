#pragma once

#include <cmath>

#include "pbpt/math/function.hpp"
#include "pbpt/radiometry/spectrum_distribution.hpp"

namespace pbpt::radiometry {

/**
 * @brief Black-body spectral power distribution.
 *
 * Models the spectral radiance of an ideal black body at a given
 * temperature, using Planck's law. Provides helpers for total power
 * and peak wavelength as well.
 *
 * @tparam T Scalar type.
 */
template<typename T>
class BlackBodySpectrumDistribution : public SpectrumDistribution<BlackBodySpectrumDistribution<T>, T> {
    friend class SpectrumDistribution<BlackBodySpectrumDistribution<T>, T>;
public:
    /// Construct a black-body spectrum at the given temperature in kelvin.
    constexpr BlackBodySpectrumDistribution(T temperature) : m_temperature(temperature) {}
    /**
     * @brief Spectral radiance of an ideal black body at temperature t_K.
     *
     * Uses Planck's law in SI units, taking wavelength in nanometers and
     * converting internally to meters.
     *
     * @param t_K       Temperature in kelvin.
     * @param lambda_nm Wavelength in nanometers.
     */
    constexpr static inline T black_body(T t_K, T lambda_nm) {
        // Planck's law formula for black body radiation
        const double h = 6.62607015e-34;  // Planck's constant
        const double c = 299792458;       // Speed of light
        const double k = 1.380649e-23;    // Boltzmann's constant
        double l = lambda_nm * 1e-9;  // Convert nm to m
        double L = (2 * h * c * c) / math::pow(l, 5) * (1 / (math::fast_exp(h * c / (l * k * t_K)) - 1));
        return L;
    }

    /**
     * @brief Total radiant exitance of a black body.
     *
     * Uses the Stefan–Boltzmann law to return power per unit area.
     *
     * @param t_K Temperature in kelvin.
     */
    constexpr static inline T black_body_M(T t_K) {
        // Stefan-Boltzmann law
        const double sigma = 5.670374419e-8;  // Stefan-Boltzmann constant
        double       E     = sigma * std::pow(t_K, 4);
        return E;
    }

    /**
     * @brief Simple non-black-body model using Kirchhoff's law.
     *
     * Scales the black-body spectrum by (1 - p_hd), where p_hd is a
     * hemispherical-directional reflectance or albedo.
     */
    constexpr static inline T non_black_body(T t_K, T lambda_nm, T p_hd) {
        // Kirchhoff's law of thermal radiation
        return black_body(t_K, lambda_nm) * (1 - p_hd);
    }

    /**
     * @brief Wavelength at which the black-body spectrum peaks.
     *
     * Uses Wien's displacement law, which states that the peak
     * wavelength is approximately b / T, where b is about
     * 2.9e-3 meter kelvin.
     *
     * @param t_K Temperature in kelvin.
     * @return Peak wavelength in nanometers.
     */
    constexpr static inline T black_body_max_wavelength(T t_K) {
        // Wien's displacement law
        const double b = 2.897771955e-3;  // Wien's displacement constant
        return b / t_K * 1e9;             // Convert from m to nm
    }

private:
    /// Temperature of the emitter in kelvin.
    T m_temperature;

public:
    /**
     * @brief Peak wavelength of the stored black-body spectrum.
     * @return Wavelength in nanometers.
     */
    constexpr T max_wavelength() const {
        return black_body_max_wavelength(m_temperature);
    }

    /**
     * @brief Maximum spectral radiance value at the peak wavelength.
     */
    constexpr T max_value() const {
        return black_body(m_temperature, max_wavelength());
    }

private:
    constexpr T at_impl(T lambda) const {
        return black_body(m_temperature, lambda);
    }
};

}  // namespace pbpt::radiometry
