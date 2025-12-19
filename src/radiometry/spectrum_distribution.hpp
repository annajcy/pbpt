/**
 * @file
 * @brief Generic spectral distribution interface and common spectrum models.
 */
#pragma once

#include <array>
#include <functional>
#include <iostream>
#include <type_traits>
#include <memory>
#include <utility>
#include <vector>

#include "math/function.hpp"
#include "math/polynomial.hpp"
#include "sampled_spectrum.hpp"
#include "constant/lambda.hpp"

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
template<typename Derived, typename T>
class SpectrumDistribution {
public:
    /**
     * @brief Evaluate the spectrum at a given wavelength.
     *
     * This forwards to the derived class's @c at_impl() implementation.
     */
    constexpr T at(T lambda) const {
        return as_derived().at_impl(lambda);
    }

    template<int N>
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
    constexpr Derived& as_derived() noexcept {
        return static_cast<Derived&>(*this);
    }

    /// Access the derived spectrum (const).
    constexpr const Derived& as_derived() const noexcept {
        return static_cast<const Derived&>(*this);
    }
};

/**
 * @brief Inner product between two spectral distributions.
 *
 * Computes the sum over discrete integer wavelengths of d1(lambda) *
 * d2(lambda) using the common type of their scalar values.
 */
template<typename D1, typename T1, typename D2, typename T2>
inline auto inner_product(const SpectrumDistribution<D1, T1>& d1, const SpectrumDistribution<D2, T2>& d2) {
    using R = std::common_type_t<T1, T2>;
    R result{};
    for (int i = constant::lambda_min<int>; i <= constant::lambda_max<int>; ++i) {
        result += d1.at(i) * d2.at(i);
    }
    return result;
}

/**
 * @brief Small holder that either owns or references an object.
 *
 * Used to implement spectrum products that can accept both lvalues and
 * rvalues without copying unnecessarily.
 */
template<class S>
struct Hold {
    /// Owned instance when the original object was an rvalue.
    std::shared_ptr<S> owned;
    /// Pointer to the referenced instance when the original object was an lvalue.
    const S* ref = nullptr;

    /// Constructs a holder that references an existing object without taking ownership.
    Hold(const S& s): owned(), ref(&s) {}

    /// Constructs a holder that takes ownership of a temporary object.
    Hold(S&& s): owned(std::make_shared<S>(std::move(s))), ref(owned.get()) {}

    /// Returns a const reference to the held object, regardless of ownership mode.
    const S& get() const { return *ref; }
};

/**
 * @brief Spectrum representing the product of two spectra.
 *
 * Values are evaluated lazily as s1(lambda) * s2(lambda).
 */
template<typename T, class D1, class D2>
class MultipliedSpectrumDistribution : public SpectrumDistribution<MultipliedSpectrumDistribution<T, D1, D2>, T> {
    friend class SpectrumDistribution<MultipliedSpectrumDistribution<T, D1, D2>, T>;

public:
    /// First factor in the product spectrum.
    Hold<D1> s1;
    /// Second factor in the product spectrum.
    Hold<D2> s2;

public:
    /// Constructs from two lvalue spectra (referenced without copying).
    MultipliedSpectrumDistribution(const D1& a, const D2& b): s1(a), s2(b) {}
    /// Constructs from a temporary first spectrum and an lvalue second spectrum.
    MultipliedSpectrumDistribution(D1&& a, const D2& b): s1(std::move(a)), s2(b) {}
    /// Constructs from an lvalue first spectrum and a temporary second spectrum.
    MultipliedSpectrumDistribution(const D1& a, D2&& b): s1(a), s2(std::move(b)) {}
    /// Constructs from two temporary spectra, taking ownership of both.
    MultipliedSpectrumDistribution(D1&& a, D2&& b): s1(std::move(a)), s2(std::move(b)) {}
    
    /**
     * @brief Evaluates the product spectrum at a wavelength.
     *
     * Simply multiplies the values of the two underlying spectra at
     * the same wavelength.
     */
    constexpr T at_impl(T lambda) const { return s1.get().at(lambda) * s2.get().at(lambda); }
};

/// Multiply two @c SpectrumDistribution objects to get a spectrum product.
template<typename D1, typename T1, typename D2, typename T2>
inline auto operator*(const SpectrumDistribution<D1, T1>& s1, const SpectrumDistribution<D2, T2>& s2) {
    using R = std::common_type_t<T1, T2>;
    return MultipliedSpectrumDistribution<R, D1, D2>(static_cast<const D1&>(s1), static_cast<const D2&>(s2));
}

/// Overload for rvalue-lvalue multiplication to avoid dangling references.
template<typename D1, typename T1, typename D2, typename T2>
inline auto operator*(SpectrumDistribution<D1, T1>&& s1, const SpectrumDistribution<D2, T2>& s2) {
    using R = std::common_type_t<T1, T2>;
    return MultipliedSpectrumDistribution<R, D1, D2>(std::move(static_cast<D1&>(s1)), static_cast<const D2&>(s2));
}

/// Overload for lvalue-rvalue multiplication to avoid dangling references.
template<typename D1, typename T1, typename D2, typename T2>
inline auto operator*(const SpectrumDistribution<D1, T1>& s1, SpectrumDistribution<D2, T2>&& s2) {
    using R = std::common_type_t<T1, T2>;
    return MultipliedSpectrumDistribution<R, D1, D2>(static_cast<const D1&>(s1), std::move(static_cast<D2&>(s2)));
}

/// Overload for rvalue-rvalue multiplication to avoid dangling references.
template<typename D1, typename T1, typename D2, typename T2>
inline auto operator*(SpectrumDistribution<D1, T1>&& s1, SpectrumDistribution<D2, T2>&& s2) {
    using R = std::common_type_t<T1, T2>;
    return MultipliedSpectrumDistribution<R, D1, D2>(std::move(static_cast<D1&>(s1)), std::move(static_cast<D2&>(s2)));
}

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

/**
 * @brief Spectrum defined by piecewise linear interpolation of sample points.
 *
 * Sample points are (wavelength, value) pairs; the spectrum value
 * is linearly interpolated between neighboring points.
 */
template<typename T>
class PiecewiseLinearSpectrumDistribution : public SpectrumDistribution<PiecewiseLinearSpectrumDistribution<T>, T> {
    friend class SpectrumDistribution<PiecewiseLinearSpectrumDistribution<T>, T>;
private:
    std::vector<std::pair<T, T>> m_points; // (lambda, value)

public:
    /**
     * @brief Spectrum defined by a set of (wavelength, value) knots.
     *
     * Values between knots are obtained by linear interpolation.
     */
    PiecewiseLinearSpectrumDistribution(const std::vector<std::pair<T, T>>& points)
        : m_points(points) {
            sort_points();
        }

private:
    constexpr T at_impl(T lambda) const {
        if (m_points.empty()) {
            return T(0);
        }

        if (lambda <= m_points.front().first) {
            return m_points.front().second;
        }

        if (lambda >= m_points.back().first) {
            return m_points.back().second;
        }

        auto it = std::lower_bound(
            m_points.begin(), 
            m_points.end(), 
            lambda,
            [](const auto& point, T value) { return point.first < value; }
        );

        const auto& [lambda1, value1] = *it;
        const auto& [lambda0, value0] = *(it - 1);
        
        return value0 + (value1 - value0) * (lambda - lambda0) / (lambda1 - lambda0);
    }

    void sort_points() {
        std::sort(m_points.begin(), m_points.end(), [](const auto& a, const auto& b) {
            return a.first < b.first;
        });
    }

    /**
     * @brief Adds a new knot to the piecewise linear spectrum.
     *
     * The new point is inserted without re-sorting; call sort_points()
     * afterwards if you rely on the ordering of the knot list.
     */
    void add_point(T lambda, T value) {
        m_points.emplace_back(lambda, value);
    }

    /**
     * @brief Removes all knots with the given wavelength.
     *
     * If multiple points share the same wavelength, they are all removed.
     */
    void remove_point(T lambda) {
        std::erase_if(m_points, [lambda](const auto& p) {
            return p.first == lambda;
        });
    }
};

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

/**
 * @brief Quadratic polynomial in wavelength with sigmoid shaping.
 *
 * Evaluates a quadratic polynomial in wavelength and passes it through
 * a sigmoid function to obtain a smooth, bounded spectrum.
 *
 * @tparam T Scalar type.
 */
template<typename T>
struct RGBSigmoidPolynomial {
    /// Quadratic coefficient (constant term).
    T c0;
    /// Quadratic coefficient (linear term).
    T c1;
    /// Quadratic coefficient (squared term).
    T c2;

    /// Constructs the polynomial from explicit coefficients.
    RGBSigmoidPolynomial(T c0, T c1, T c2) : c0(c0), c1(c1), c2(c2) {}
    /// Constructs the polynomial from a coefficient array [c0, c1, c2].
    RGBSigmoidPolynomial(const std::array<T, 3>& coeffs) : c0(coeffs[0]), c1(coeffs[1]), c2(coeffs[2]) {}

    /**
     * @brief Evaluates the sigmoid-shaped polynomial at a wavelength.
     *
     * A quadratic polynomial in the wavelength is evaluated and then
     * passed through a sigmoid function to keep the spectrum bounded
     * between 0 and 1.
     *
     * @param lambda Wavelength parameter (typically in nanometers).
     */
    constexpr T at(T lambda) const {
        return math::sigmoid(math::Polynomial<T>::evaluate(lambda, c0, c1, c2));
    }
};

/**
 * @brief Normalized quadratic sigmoid-polynomial for RGB basis spectra.
 *
 * Similar to RGBSigmoidPolynomial, but defined over a normalized
 * wavelength parameter t in [0,1], making the coefficients more
 * stable across different wavelength ranges.
 *
 * @tparam T Scalar type.
 */
template<typename T>
struct RGBSigmoidPolynomialNormalized {
    /// Normalized constant coefficient.
    T c0;
    /// Normalized linear coefficient.
    T c1;
    /// Normalized squared coefficient.
    T c2;

    /// Constructs the normalized polynomial from explicit coefficients.
    RGBSigmoidPolynomialNormalized(T c0, T c1, T c2) : c0(c0), c1(c1), c2(c2) {}
    /// Constructs the normalized polynomial from a coefficient array [c0, c1, c2].
    RGBSigmoidPolynomialNormalized(const std::array<T, 3>& coeffs) : c0(coeffs[0]), c1(coeffs[1]), c2(coeffs[2]) {}

    /**
     * @brief Evaluates the normalized sigmoid-polynomial at a wavelength.
     *
     * The input wavelength is first mapped to a normalized range [0,1]
     * and the quadratic polynomial and sigmoid are applied to that
     * normalized parameter.
     *
     * @param lambda Wavelength in nanometers.
     */
    constexpr T at(T lambda) const {
        const T t = (lambda - T(360)) / T(830 - 360);   // 归一化
        return math::sigmoid(math::Polynomial<T>::evaluate(t, c0, c1, c2));
    }

    /**
     * @brief Converts the normalized polynomial back to wavelength space.
     *
     * Applies the inverse change of variables to obtain coefficients
     * defined directly in terms of wavelength rather than the normalized
     * parameter.
     */
    RGBSigmoidPolynomial<T> to_unnormalized() const {
        // 转换公式基于归一化变换: t = (lambda - 360) / (830 - 360)
        // 原多项式: A*t^2 + B*t + C
        // 转换为原始wavelength域: A'*lambda^2 + B'*lambda + C'
        double c0_val = 360.0, c1_val = 1.0 / (830.0 - 360.0);
        double A = c2, B = c1, C = c0;
        double c1_squared = c1_val * c1_val;
        double c0_c1 = c0_val * c1_val;
        
        return RGBSigmoidPolynomial<T>{
            T(C - B * c0_val * c1_val + A * c0_c1 * c0_c1),
            T(B * c1_val - 2 * A * c0_val * c1_squared), 
            T(A * c1_squared)
        };
    }
};

/**
 * @brief Albedo (reflectance) spectrum derived from an RGB model.
 *
 * Wraps an underlying parametric RGB spectrum (such as
 * RGBSigmoidPolynomialNormalized) and exposes it as a generic
 * SpectrumDistribution.
 */
template <typename T, template<typename> class RSPType>
class RGBAlbedoSpectrumDistribution : public SpectrumDistribution<RGBAlbedoSpectrumDistribution<T, RSPType>, T>{
    friend class SpectrumDistribution<RGBAlbedoSpectrumDistribution<T, RSPType>, T>;

private:
    RSPType<T> m_rsp;

public:
    /// Constructs an albedo spectrum from an underlying RGB basis spectrum.
    RGBAlbedoSpectrumDistribution(const RSPType<T>& rsp) : m_rsp(rsp) {}

    /// Returns a const reference to the underlying RGB spectrum model.
    const RSPType<T>& rsp() const { return m_rsp; }
    /// Returns a mutable reference to the underlying RGB spectrum model.
    RSPType<T>& rsp() { return m_rsp; }

private:
    T at_impl(T lambda) const {
        return m_rsp.at(lambda);
    }
};

/**
 * @brief Unbounded RGB spectrum with a global scale factor.
 *
 * Similar to RGBAlbedoSpectrumDistribution, but allows scaling the
 * underlying spectrum by an arbitrary factor.
 */
template<typename T, template<typename> class RSPType>
class RGBUnboundedSpectrumDistribution : public SpectrumDistribution<RGBUnboundedSpectrumDistribution<T, RSPType>, T>{
    friend class SpectrumDistribution<RGBUnboundedSpectrumDistribution<T, RSPType>, T>;

private:
    RSPType<T> m_rsp;
    T m_scale;

public:
    /**
     * @brief Constructs an unbounded spectrum from an RGB basis and scale.
     *
     * @param rsp    Underlying RGB basis spectrum.
     * @param m_scale Global multiplicative factor applied to the spectrum.
     */
    RGBUnboundedSpectrumDistribution(const RSPType<T>& rsp, T m_scale) : m_rsp(rsp), m_scale(m_scale) {}

private:
    T at_impl(T lambda) const {
        return m_scale * m_rsp.at(lambda);
    }
};

/**
 * @brief Illuminant spectrum constructed from an RGB basis and reference illuminant.
 *
 * The spectrum is the product of a parametric RGB spectrum and a
 * reference illuminant spectrum, optionally scaled by m_scale.
 */
template<typename T, template<typename> class RSPType, typename IlluminantSpectrumType>
class RGBIlluminantSpectrumDistribution : public SpectrumDistribution<RGBIlluminantSpectrumDistribution<T, RSPType, IlluminantSpectrumType>, T>{
    friend class SpectrumDistribution<RGBIlluminantSpectrumDistribution<T, RSPType, IlluminantSpectrumType>, T>;

private:
    RSPType<T> m_rsp;
    T m_scale;
    IlluminantSpectrumType m_reference_luminant;

public:
    /**
     * @brief Constructs an illuminant spectrum from an RGB basis and reference illuminant.
     *
     * @param rsp                 Underlying RGB basis spectrum.
     * @param reference_luminant  Reference illuminant spectrum to modulate.
     * @param m_scale             Optional global scale factor.
     */
    RGBIlluminantSpectrumDistribution(const RSPType<T>& rsp, const IlluminantSpectrumType& reference_luminant, T m_scale = T{1.0}) 
        : m_rsp(rsp), m_reference_luminant(reference_luminant) , m_scale(m_scale) {}

private:
    T at_impl(T lambda) const {
        return m_scale * m_rsp.at(lambda) * m_reference_luminant.at(lambda);
    }
};

};
