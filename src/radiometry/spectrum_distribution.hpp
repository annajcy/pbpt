#pragma once

#include <functional>
#include <iostream>
#include <type_traits>
#include <memory>

#include "math/function.hpp"
#include "math/polynomial.hpp"

#include "sampled_spectrum.hpp"

namespace pbpt::radiometry {

template <typename T>
constexpr T lambda_min = 360;

template <typename T>
constexpr T lambda_max = 830;

template<typename Derived, typename T>
class SpectrumDistribution {
public:
    constexpr T at(T lambda) const {
        return as_derived().at_impl(lambda);
    }

    template<int N>
    constexpr SampledSpectrum<T, N> sample(const SampledWavelength<T, N>& wavelengths) const {
        SampledSpectrum<T, N> result;
        for (int i = 0; i < N; ++i) {
            result[i] = at(wavelengths[i]);
        }
        return result;
    }

    const Derived& as_derived() const {
        return static_cast<const Derived&>(*this);
    }

    Derived& as_derived() {
        return static_cast<Derived&>(*this);
    }
};

template<typename D1, typename T1, typename D2, typename T2>
inline auto inner_product(const SpectrumDistribution<D1, T1>& d1, const SpectrumDistribution<D2, T2>& d2) {
    using R = std::common_type_t<T1, T2>;
    R result{};
    for (int i = lambda_min<int>; i <= lambda_max<int>; ++i) {
        result += d1.at(i) * d2.at(i);
    }
    return result;
}

template<class S>
struct Hold {
    std::shared_ptr<S> owned;        // 如果是右值，就放到这里
    const S* ref = nullptr;          // 如果是左值，就指向它
    Hold(const S& s): ref(&s) {}
    Hold(S&& s): owned(std::make_shared<S>(std::move(s))), ref(owned.get()) {}
    const S& get() const { return *ref; }
};

template<typename T, class D1, class D2>
class MultipliedSpectrumDistribution : public SpectrumDistribution<MultipliedSpectrumDistribution<T, D1, D2>, T> {
    friend class SpectrumDistribution<MultipliedSpectrumDistribution<T, D1, D2>, T>;

public:
    Hold<D1> s1; Hold<D2> s2;

public:
    MultipliedSpectrumDistribution(const D1& a, const D2& b): s1(a), s2(b) {}
    MultipliedSpectrumDistribution(D1&& a, const D2& b): s1(std::move(a)), s2(b) {}
    MultipliedSpectrumDistribution(const D1& a, D2&& b): s1(a), s2(std::move(b)) {}
    MultipliedSpectrumDistribution(D1&& a, D2&& b): s1(std::move(a)), s2(std::move(b)) {}
    
    constexpr T at_impl(T lambda) const { return s1.get().at(lambda) * s2.get().at(lambda); }
};

template<class D1, class T1, class D2, class T2>
auto operator*(D1&& a, D2&& b) {
  using R = std::common_type_t<T1,T2>;
  using A = std::decay_t<D1>; using B = std::decay_t<D2>;
  return MultipliedSpectrumDistribution<R, A, B>(std::forward<D1>(a), std::forward<D2>(b));
}

template<typename D1, typename T1, typename D2, typename T2>
inline auto operator*(const SpectrumDistribution<D1, T1>& s1, const SpectrumDistribution<D2, T2>& s2) {
    using R = std::common_type_t<T1, T2>;
    return MultipliedSpectrumDistribution<R, D1, D2>(static_cast<const D1&>(s1), static_cast<const D2&>(s2));
}

template<typename T>
class ConstantSpectrumDistribution : public SpectrumDistribution<ConstantSpectrumDistribution<T>, T> {
    friend class SpectrumDistribution<ConstantSpectrumDistribution<T>, T>;

private:
    T m_value;

public:
    ConstantSpectrumDistribution(T value) : m_value(value) {}
    
    constexpr T max_value() const { 
        return m_value; 
    }

private:
    constexpr T at_impl(T lambda) const { return m_value; }
    
};

template <typename T>
constexpr inline T black_body(T t_K, T lambda_nm) {
    // Planck's law formula for black body radiation
    const double h = 6.62607015e-34;  // Planck's constant
    const double c = 299792458;       // Speed of light
    const double k = 1.380649e-23;    // Boltzmann's constant

    double l = lambda_nm * 1e-9;  // Convert nm to m

    double L = (2 * h * c * c) / math::pow(l, 5) * (1 / (math::fast_exp(h * c / (l * k * t_K)) - 1));

    return L;
}

template <typename T>
constexpr inline T black_body_M(T t_K) {
    // Stefan-Boltzmann law
    const double sigma = 5.670374419e-8;  // Stefan-Boltzmann constant
    double       E     = sigma * std::pow(t_K, 4);
    return E;
}

template <typename T>
constexpr inline T non_black_body(T t_K, T lambda_nm, T p_hd) {
    // Kirchhoff's law of thermal radiation
    return black_body(t_K, lambda_nm) * (1 - p_hd);
}

// max wavelength
template <typename T>
inline T black_body_max_wavelength(T t_K) {
    // Wien's displacement law
    const double b = 2.897771955e-3;  // Wien's displacement constant
    return b / t_K * 1e9;             // Convert from m to nm
}

template<typename T>
class BlackBodySpectrumDistribution : public SpectrumDistribution<BlackBodySpectrumDistribution<T>, T> {
    friend class SpectrumDistribution<BlackBodySpectrumDistribution<T>, T>;

private:
    T m_temperature;

public:
    constexpr BlackBodySpectrumDistribution(T temperature) : m_temperature(temperature) {}

    constexpr T max_wavelength() const {
        return black_body_max_wavelength(m_temperature);
    }

    constexpr T max_value() const {
        return black_body(m_temperature, max_wavelength());
    }

private:
    constexpr T at_impl(T lambda) const {
        return black_body(m_temperature, lambda);
    }
};

template<typename T>
class FunctionalSpectrumDistribution : public SpectrumDistribution<FunctionalSpectrumDistribution<T>, T> {
    friend class SpectrumDistribution<FunctionalSpectrumDistribution<T>, T>;
private:
    std::function<T(T)> m_f;

public:
    using base = SpectrumDistribution<FunctionalSpectrumDistribution<T>, T>;
    using base::base;

    constexpr FunctionalSpectrumDistribution(const std::function<T(T)>& f) : m_f(f) {}

private:
    constexpr T at_impl(T lambda) const {
        return m_f(lambda);
    }
};

template<typename T>
class PiecewiseLinearSpectrumDistribution : public SpectrumDistribution<PiecewiseLinearSpectrumDistribution<T>, T> {
    friend class SpectrumDistribution<PiecewiseLinearSpectrumDistribution<T>, T>;
private:
    std::vector<std::pair<T, T>> m_points; // (lambda, value)

public:
    using base = SpectrumDistribution<PiecewiseLinearSpectrumDistribution<T>, T>;
    using base::base;

    PiecewiseLinearSpectrumDistribution(const std::vector<std::pair<T, T>>& points)
        : m_points(points) {}

private:
    constexpr T at_impl(T lambda) const {
        // Find the segment that contains the given lambda
        for (size_t i = 1; i < m_points.size(); ++i) {
            if (lambda < m_points[i].first) {
                // Perform linear interpolation between the two points
                const auto& [lambda0, value0] = m_points[i - 1];
                const auto& [lambda1, value1] = m_points[i];
                return value0 + (value1 - value0) * (lambda - lambda0) / (lambda1 - lambda0);
            }
        }
        // If lambda is out of range, return 0
        return T(0);
    }
};

template <int LMin, int LMax>
struct TabularSpectrumRange {
    static constexpr int LMinValue = LMin;
    static constexpr int LMaxValue = LMax;
    static constexpr int Count     = LMax - LMin + 1;
};

template<typename T, int LambdaMin, int LambdaMax>
class TabularSpectrumDistribution : public SpectrumDistribution<TabularSpectrumDistribution<T, LambdaMin, LambdaMax>, T> {
    friend class SpectrumDistribution<TabularSpectrumDistribution<T, LambdaMin, LambdaMax>, T>;

private:
    std::vector<T> m_samples;

public:
    using base = SpectrumDistribution<TabularSpectrumDistribution<T, LambdaMin, LambdaMax>, T>;
    using base::base;

    constexpr TabularSpectrumDistribution(const std::vector<T>& samples)
        : m_samples(samples) {
        if (samples.size() != static_cast<size_t>(LambdaMax - LambdaMin + 1)) {
            throw std::invalid_argument("Sample size does not match the expected range.");
        }
    }

    template<typename... Args>
    requires (sizeof...(Args) == (LambdaMax - LambdaMin + 1)) && (std::conjunction_v<std::is_convertible<Args, T>...>)
    constexpr TabularSpectrumDistribution(Args&&... args)
        : m_samples{std::forward<Args>(args)...} {}

    constexpr int sample_count() const {
        return LambdaMax - LambdaMin + 1;
    }

    constexpr int lambda_min() const {
        return LambdaMin;
    }

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

template<typename T>
struct RGBSigmoidPolynomial {
    T c0, c1, c2;

    RGBSigmoidPolynomial(T c0, T c1, T c2) : c0(c0), c1(c1), c2(c2) {}

    constexpr T at(T lambda) const {
        return math::sigmoid(math::Polynomial<T>::evaluate(lambda, c0, c1, c2));
    }
};

template<typename T>
struct RGBSigmoidPolynomialNormalized {
    T c0, c1, c2;

    RGBSigmoidPolynomialNormalized(T c0, T c1, T c2) : c0(c0), c1(c1), c2(c2) {}

    constexpr T at(T lambda) const {
        const T t = (lambda - T(360)) / T(830 - 360);   // 归一化
        return math::sigmoid(math::Polynomial<T>::evaluate(t, c0, c1, c2));
    }

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

template <typename T, template<typename> class RSPType>
class RGBAlbedoSpectrumDistribution : public SpectrumDistribution<RGBAlbedoSpectrumDistribution<T, RSPType>, T>{
    friend class SpectrumDistribution<RGBAlbedoSpectrumDistribution<T, RSPType>, T>;

private:
    RSPType<T> m_rsp;

public:
    RGBAlbedoSpectrumDistribution(const RSPType<T>& rsp) : m_rsp(rsp) {}
    const RSPType<T>& rsp() const { return m_rsp; }
    RSPType<T>& rsp() { return m_rsp; }

private:
    T at_impl(T lambda) const {
        return m_rsp.at(lambda);
    }
};

template<typename T, template<typename> class RSPType>
class RGBUnboundedSpectrumDistribution : public SpectrumDistribution<RGBUnboundedSpectrumDistribution<T, RSPType>, T>{
    friend class SpectrumDistribution<RGBUnboundedSpectrumDistribution<T, RSPType>, T>;

private:
    RSPType<T> m_rsp;
    T m_scale;

public:

    RGBUnboundedSpectrumDistribution(const RSPType<T>& rsp, T m_scale) 
        : m_rsp(rsp), m_scale(m_scale) {}

private:
    T at_impl(T lambda) const {
        return m_scale * m_rsp.at(lambda);
    }
};

template<typename T, template<typename> class RSPType, typename LuminantSpectrumType>
class RGBIlluminantSpectrumDistribution : public SpectrumDistribution<RGBIlluminantSpectrumDistribution<T, RSPType, LuminantSpectrumType>, T>{
    friend class SpectrumDistribution<RGBIlluminantSpectrumDistribution<T, RSPType, LuminantSpectrumType>, T>;

private:
    RSPType<T> m_rsp;
    T m_scale;
    LuminantSpectrumType m_reference_luminant;

public:
    RGBIlluminantSpectrumDistribution(const RSPType<T>& rsp, const LuminantSpectrumType& reference_luminant, T m_scale = T{1.0}) 
        : m_rsp(rsp), m_reference_luminant(reference_luminant) , m_scale(m_scale) {}

private:
    T at_impl(T lambda) const {
        return m_scale * m_rsp.at(lambda) * m_reference_luminant.at(lambda);
    }
};

};