#pragma once

#include <array>
#include <iostream>
#include <type_traits>
#include <utility>

#include "radiometry/spectrum_distribution.hpp"

namespace pbpt::radiometry {

/**
 * @brief Compile-time range information for tabular spectra.
 *
 * Stores minimum and maximum wavelength and the number of samples.
 *
 * @tparam LMin Minimum wavelength (nm).
 * @tparam LMax Maximum wavelength (nm).
 */
template <int LMin, int LMax>
struct TabularSpectrumRange {
    /// Minimum wavelength in nanometers.
    static constexpr int LMinValue = LMin;
    /// Maximum wavelength in nanometers.
    static constexpr int LMaxValue = LMax;
    /// Number of integer sample points between LMin and LMax inclusive.
    static constexpr int Count     = LMax - LMin + 1;
};

/**
 * @brief Spectrum stored as samples on a fixed integer wavelength grid.
 *
 * Wavelengths range from LambdaMin to LambdaMax inclusive, with one
 * sample per integer wavelength. Lookup outside the range returns zero
 * and prints a warning to standard output.
 *
 * @tparam T         Scalar sample type.
 * @tparam LambdaMin Minimum wavelength (nm).
 * @tparam LambdaMax Maximum wavelength (nm).
 */
template<typename T, int LambdaMin, int LambdaMax>
class TabularSpectrumDistribution : public SpectrumDistribution<TabularSpectrumDistribution<T, LambdaMin, LambdaMax>, T> {
    friend class SpectrumDistribution<TabularSpectrumDistribution<T, LambdaMin, LambdaMax>, T>;

private:
    std::array<T, TabularSpectrumRange<LambdaMin, LambdaMax>::Count> m_samples;

public:
    /// Default constructor - initializes all samples to zero.
    constexpr TabularSpectrumDistribution() : m_samples{} {}
    
    /// Construct from an array of samples matching the tabular range.
    constexpr TabularSpectrumDistribution(const std::array<T, TabularSpectrumRange<LambdaMin, LambdaMax>::Count>& samples)
        : m_samples(samples) {}

    /// Construct from a flat list of sample values (one per integer wavelength).
    template<typename... Args>
    requires (sizeof...(Args) == (LambdaMax - LambdaMin + 1)) && (std::conjunction_v<std::is_convertible<Args, T>...>)
    constexpr TabularSpectrumDistribution(Args&&... args)
        : m_samples{static_cast<T>(std::forward<Args>(args))...} {}

    /// Number of sample entries stored in the spectrum.
    constexpr int sample_count() const {
        return LambdaMax - LambdaMin + 1;
    }

    /// Minimum wavelength covered by the spectrum (nm).
    constexpr int lambda_min() const {
        return LambdaMin;
    }

    /// Maximum wavelength covered by the spectrum (nm).
    constexpr int lambda_max() const {
        return LambdaMax;
    }

private:
    template<typename U>
    constexpr T at_impl(U lambda) const {
        if(lambda < lambda_min() || lambda > lambda_max()){
            std::cout << "Warning: Wavelength " << lambda << " out of range [" << lambda_min() << ", " << lambda_max() << "]\n";
            return T(0);
        }

        return m_samples[(static_cast<int>(lambda) - LambdaMin)];
    }
};

}  // namespace pbpt::radiometry
