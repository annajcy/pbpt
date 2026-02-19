/**
 * @file
 * @brief Generic spectral distribution interface.
 */
#pragma once

#include <type_traits>

#include "constant/lambda.hpp"
#include "sampled_spectrum.hpp"

namespace pbpt::radiometry {

/**
 * @brief CRTP base class for spectra parameterized by wavelength.
 *
 * Concrete spectra implement @c at_impl(lambda) which is then exposed
 * as @c at(lambda). Utility functions such as @c sample() are provided
 * in terms of this interface.
 *
 * @tparam Derived Concrete spectrum type.
 * @tparam T       Scalar type.
 */
template <typename Derived, typename T>
class SpectrumDistribution {
public:
    /**
     * @brief Evaluate the spectrum at a given wavelength.
     *
     * This forwards to the derived class's @c at_impl() implementation.
     */
    constexpr T at(T lambda) const { return as_derived().at_impl(lambda); }

    template <int N>
    /**
     * @brief Sample the spectrum at a set of discrete wavelengths.
     *
     * @param wavelengths Sampled wavelengths (typically in nanometers).
     * @return Vector of spectrum values at those wavelengths.
     */
    constexpr SampledSpectrum<T, N> sample(const SampledWavelength<T, N>& wavelengths) const {
        SampledSpectrum<T, N> result;
        for (int i = 0; i < N; ++i) {
            result[i] = at(wavelengths[i]);
        }
        return result;
    }

    /// Access the derived spectrum (mutable).
    constexpr Derived& as_derived() noexcept { return static_cast<Derived&>(*this); }

    /// Access the derived spectrum (const).
    constexpr const Derived& as_derived() const noexcept { return static_cast<const Derived&>(*this); }
};

/**
 * @brief Inner product between two spectral distributions.
 *
 * Computes the sum over discrete integer wavelengths of d1(lambda) *
 * d2(lambda) using the common type of their scalar values.
 */
template <typename D1, typename T1, typename D2, typename T2>
inline auto inner_product(const SpectrumDistribution<D1, T1>& d1, const SpectrumDistribution<D2, T2>& d2) {
    using R = std::common_type_t<T1, T2>;
    R result{};
    for (int i = constant::lambda_min<int>; i <= constant::lambda_max<int>; ++i) {
        result += d1.at(i) * d2.at(i);
    }
    return result;
}

}  // namespace pbpt::radiometry
