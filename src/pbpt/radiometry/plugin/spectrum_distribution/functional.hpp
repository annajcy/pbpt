#pragma once

#include <functional>

#include "pbpt/radiometry/spectrum_distribution.hpp"

namespace pbpt::radiometry {

/**
 * @brief Spectrum defined by an arbitrary callable f(lambda).
 *
 * This is useful for plugging in analytic spectra without creating
 * a dedicated class.
 *
 * @tparam T Scalar type.
 */
template<typename T>
class FunctionalSpectrumDistribution : public SpectrumDistribution<FunctionalSpectrumDistribution<T>, T> {
    friend class SpectrumDistribution<FunctionalSpectrumDistribution<T>, T>;
private:
    std::function<T(T)> m_f;

public:
    /// Construct from a callable f(lambda) that returns spectral values.
    constexpr FunctionalSpectrumDistribution(const std::function<T(T)>& f) : m_f(f) {}

private:
    constexpr T at_impl(T lambda) const {
        return m_f(lambda);
    }
};

}  // namespace pbpt::radiometry
