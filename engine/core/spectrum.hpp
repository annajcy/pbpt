#pragma once

#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <string>
#include <format>
#include "math/vector.hpp"

namespace pbpt::core {

template<typename T>
class RGB : public math::Vector<T, 3> {
public:
    RGB() : math::Vector<T, 3>(0, 0, 0) {}
    RGB(T r, T g, T b) : math::Vector<T, 3>(r, g, b) {}
    RGB(const math::Vector<T, 3>& vec) : math::Vector<T, 3>(vec) {}

    T r() const { return this->x(); }
    T g() const { return this->y(); }
    T b() const { return this->z(); }
};

template <typename T, int N>
class SampledSpectrum : public math::Vector<T, N> {
public:
    SampledSpectrum() : math::Vector<T, N>() {}
    SampledSpectrum(const math::Vector<T, N>& vec) : math::Vector<T, N>(vec) {}
};

template<typename T, int N> 
class SampledWavelength : public math::Vector<T, N> {
public:
    SampledWavelength() : math::Vector<T, N>() {}
    SampledWavelength(const math::Vector<T, N>& vec) : math::Vector<T, N>(vec) {}
};

template<typename T, int N>
class SampledPdf : public math::Vector<T, N> {
public:
    SampledPdf() : math::Vector<T, N>() {}
    SampledPdf(const math::Vector<T, N>& vec) : math::Vector<T, N>(vec) {}
};

template<typename T>
std::string to_string(const RGB<T>& spectrum) {
    return std::format("RGBSpectrum<{}>({}, {}, {})", 
                       typeid(T).name(), 
                       spectrum.r(), 
                       spectrum.g(), 
                       spectrum.b());
}

template<typename T>
std::ostream& operator<<(std::ostream& os, const RGB<T>& spectrum) {
    return os << to_string(spectrum);
}

template<typename T>
constexpr inline T black_body(T t_K, T lambda_nm) {
    // Planck's law formula for black body radiation
    const double h = 6.62607015e-34;  // Planck's constant
    const double c = 299792458;       // Speed of light
    const double k = 1.380649e-23;    // Boltzmann's constant

    double l = lambda_nm * 1e-9; // Convert nm to m

    double L = (2 * h * c * c) / math::pow(l, 5) *
                      (1 / (math::fast_exp(h * c / (l * k * t_K)) - 1));

    return L;
}

template<typename T>
constexpr inline T black_body_M(T t_K) {
    // Stefan-Boltzmann law
    const double sigma = 5.670374419e-8;  // Stefan-Boltzmann constant
    double E = sigma * std::pow(t_K, 4);
    return E;
}

template<typename T>
constexpr inline T non_black_body(T t_K, T lambda_nm, T p_hd) {
    // Kirchhoff's law of thermal radiation
    return black_body(t_K, lambda_nm) * (1 - p_hd);
}

// max wavelength
template<typename T>
inline T black_body_max_wavelength(T t_K) {
    // Wien's displacement law
    const double b = 2.897771955e-3;  // Wien's displacement constant
    return b / t_K * 1e9;  // Convert from m to nm
}

template<typename T>
constexpr T lambda_min = 360;

template<typename T>
constexpr T lambda_max = 830;

template<typename Derived, typename T>
class SpectrumDistribution {
public:
    constexpr SpectrumDistribution() = default;

    constexpr T at(T lambda) const {
        return static_cast<Derived&>(*this)->at_impl(lambda);
    }

    template<int N>
    constexpr SampledSpectrum<T, N> sample(const SampledWavelength<T, N>& wavelengths) const {
        SampledSpectrum<T, N> result;
        for (int i = 0; i < N; ++i) {
            result[i] = static_cast<const Derived&>(*this).at_impl(wavelengths[i]);
        }
        return result;
    }
};

template<typename T>
class ConstantSpectrumDistribution : public SpectrumDistribution<ConstantSpectrumDistribution<T>, T> {
private:
    T m_value;

public:
    ConstantSpectrumDistribution(T value) : m_value(value) {}

    constexpr T at_impl(T lambda) const { return m_value; }
    constexpr T max_value() const { return m_value; }
};

template<typename T>
class BlackBodySpectrumDistribution : public SpectrumDistribution<BlackBodySpectrumDistribution<T>, T> {
private:
    T m_temperature;

public:
    constexpr BlackBodySpectrumDistribution(T temperature) : m_temperature(temperature) {}

    constexpr T at_impl(T lambda) const {
        return black_body(m_temperature, lambda);
    }

    constexpr T max_wavelength() const {
        return black_body_max_wavelength(m_temperature);
    }

    constexpr T max_value() const {
        return black_body(m_temperature, max_wavelength());
    }
};

template<typename T>
class FunctionalSpectrumDistribution : public SpectrumDistribution<FunctionalSpectrumDistribution<T>, T> {
private:
    std::function<T(T)> m_f;

public:
    constexpr FunctionalSpectrumDistribution(const std::function<T(T)>& f) : m_f(f) {}

    constexpr T at_impl(T lambda) const {
        return m_f(lambda);
    }
};

template<typename T, int LambdaMin = lambda_min<T>, int LambdaMax = lambda_max<T>>
class TabularSpectrumDistribution : public SpectrumDistribution<TabularSpectrumDistribution<T, LambdaMin, LambdaMax>, T> {
private:
    std::array<T, LambdaMax - LambdaMin + 1> m_samples;

public:

    constexpr int sample_count() const {
        return LambdaMax - LambdaMin + 1;
    }

    constexpr int lambda_min() const {
        return LambdaMin;
    }

    constexpr int lambda_max() const {
        return LambdaMax;
    }

    constexpr TabularSpectrumDistribution(const std::array<T, LambdaMax - LambdaMin + 1>& samples)
        : m_samples(samples) {}

    template<typename U>
    constexpr T at_impl(U lambda) const {
        if(lambda < lambda_min() || lambda > lambda_max()){
            std::cout << "Warning: Wavelength " << lambda << " out of range [" << lambda_min() << ", " << lambda_max() << "]\n";
            return T(0);
        }

        return m_samples[(static_cast<int>(lambda) - LambdaMin)];
    }
};

};