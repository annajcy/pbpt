#pragma once

#include "radiometry/spectrum_distribution.hpp"

namespace pbpt::radiometry {

/**
 * @brief Spectrum with a constant value over all wavelengths.
 */
template<typename T>
class ConstantSpectrumDistribution : public SpectrumDistribution<ConstantSpectrumDistribution<T>, T> {
    friend class SpectrumDistribution<ConstantSpectrumDistribution<T>, T>;

private:
    T m_value;

public:
    /// Constructs a spectrum whose value is constant over all wavelengths.
    ConstantSpectrumDistribution(T value) : m_value(value) {}
    
    /// Returns the maximum value of the spectrum, which equals the constant value.
    constexpr T max_value() const { 
        return m_value; 
    }

private:
    constexpr T at_impl(T lambda) const { return m_value; }
    
};

}  // namespace pbpt::radiometry
