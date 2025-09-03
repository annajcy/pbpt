#pragma once

#include <array>
#include <cstdlib>
#include <cmath>
#include <concepts>
#include <algorithm>
#include <utility>

#include "math/function.hpp"
#include "math/normal.hpp"
#include "math/point.hpp"
#include "math/vector.hpp"

namespace pbpt::core {

template<std::floating_point T>
inline std::array<T, 2> concentric_sample_disk(const std::array<T, 2>& u01) {
    const T a = T(2) * u01[0] - T(1);
    const T b = T(2) * u01[1] - T(1);

    if (math::is_zero(a) && math::is_zero(b))
        return {0, 0};

    T r, phi;
    if (math::is_greater(std::abs(a), std::abs(b))) {
        r = std::abs(a);
        phi = (math::pi_v<T> / 4) * (b / a);
    } else {
        r = std::abs(b);
        phi = (math::pi_v<T> / T(2)) - (math::pi_v<T> / T(4)) * (a / b);
    }

    return {r * std::cos(phi), r * std::sin(phi)};
}

template<std::floating_point T>
inline std::array<T, 3> uniform_sample_hemisphere(const std::array<T, 2>& uv) {
    T z = uv[0];
    T r = std::sqrt(std::max(T(0), T(1) - z * z));
    T phi = 2.0 * math::pi_v<T> * uv[1];
    return {r * std::cos(phi), r * std::sin(phi), z};
}

template<typename SampleType, std::floating_point T>
using Sample = std::pair<SampleType, T>;

template<typename Derived, std::floating_point T>
class IntegrableDomain {
public:

    template<typename RNG>
    auto sample_one(RNG& rng2d) const {
        return as_derived().sample_one_impl(rng2d);
    }

    constexpr auto pdf() const {
        return as_derived().pdf_impl();
    }

    template<typename RNG, typename Func>
    void foreach_sample(int count, RNG& rng, Func&& func) const {
        for (int i = 0; i < count; ++i) {
            auto [sample, pdf] = sample_one(rng);
            func(sample, pdf);
        }
    }

    Derived& as_derived() {
        return static_cast<Derived&>(*this);
    }

    const Derived& as_derived() const {
        return static_cast<const Derived&>(*this);
    }

};

template<std::floating_point T>
using DirectionSample = Sample<math::Vector<T, 3>, T>;

template<std::floating_point T>
class UniformHemisphereDomain : public IntegrableDomain<UniformHemisphereDomain<T>, T> {
public:

    using sample_type = math::Vector<T,3>;

    template<typename RNG>
    DirectionSample<T> sample_one_impl(RNG& rng2d) const {
        auto uv = rng2d.generate_uniform(0, 1); // uv ∈ [0,1]^2
        auto dir = math::Vector<T, 3>::from_array(uniform_sample_hemisphere(uv));
        return {dir, pdf_impl()};
    }

    constexpr T pdf_impl() const {
        return T(1) / (2 * math::pi_v<T>);
    }
};

template<std::floating_point T>
class ProjectedHemisphereDomain : public IntegrableDomain<ProjectedHemisphereDomain<T>, T> {
public:
    using sample_type = math::Vector<T, 3>;

    template<typename RNG>
    DirectionSample<T> sample_one_impl(RNG& rng) const {
        auto uv = rng.generate_uniform(T(0), T(1));
        auto d  = concentric_sample_disk<T>(uv);
        const T r2 = d[0] * d[0] + d[1] * d[1];
        const T z  = std::sqrt(std::max(T(0), T(1) - r2));
        return { math::Vector<T,3>{ d[0], d[1], z }, pdf_impl() };
    }

    constexpr T pdf_impl() const { return T(1) / math::pi_v<T>; } // wrt dω⊥
};

template<std::floating_point T>
using DiskSample = Sample<math::Point<T, 2>, T>;

template<std::floating_point T>
class UniformDiskDomain : public IntegrableDomain<UniformDiskDomain<T>, T> {
public:
    using sample_type = math::Point<T, 2>;

    template<typename RNG>
    DiskSample<T> sample_one_impl(RNG& rng2d) const {
        auto u01 = rng2d.generate_uniform(0, 1);
        auto point = math::Point<T, 2>::from_array(concentric_sample_disk(u01));
        return {point, pdf_impl()};
    }

    constexpr T pdf_impl() const {
        return T(1) / (math::pi_v<T>);
    }
};


template<std::floating_point T>
struct SurfaceInfo {
    math::Point<T, 3> position;
    math::Normal<T, 3> normal;
};

template<std::floating_point T>
using SurfaceSample = Sample<SurfaceInfo<T>, T>;

template<std::floating_point T>
class ParallelogramAreaDomain : public IntegrableDomain<ParallelogramAreaDomain<T>, T> {
private:
    math::Point<T, 3> m_origin;
    math::Vector<T, 3> m_edge1;
    math::Vector<T, 3> m_edge2;

    math::Normal<T, 3> m_normal;
    T m_area;

public:
    using sample_type = SurfaceInfo<T>;

    ParallelogramAreaDomain(const math::Point<T, 3>& origin, const math::Vector<T, 3>& edge1, const math::Vector<T, 3>& edge2)
        : m_origin(origin), m_edge1(edge1), m_edge2(edge2) {
            m_normal = math::Normal<T, 3>(cross(m_edge1, m_edge2).normalized());
            m_area = cross(m_edge1, m_edge2).length();
            math::assert_if(math::is_less_equal(m_area, T(0)), "Parallelogram area must be > 0");
        }

    constexpr T area() const {
        return m_area;
    }

    constexpr math::Normal<T, 3> normal() const {
        return m_normal;
    }

    template<typename RNG>
    SurfaceSample<T> sample_one_impl(RNG& rng2d) const {
        auto uv = rng2d.generate_uniform(0.0, 1.0);
        auto p = m_origin + uv[0] * m_edge1 + uv[1] * m_edge2;
        SurfaceInfo<T> info{p, normal()};
        return {info, pdf_impl()};
    }

    constexpr T pdf_impl() const {
        return T(1) / area();
    }

};

template<std::floating_point T, typename Domain, typename Func, typename RNG>
auto integrate(const Domain& domain, const Func& func, int sample_count, RNG& rng) {
    using R = decltype(std::declval<const Func&>()(std::declval<typename Domain::sample_type>()) / static_cast<T>(std::declval<int>()));
    R result = R{};
    domain.foreach_sample(sample_count, rng, [&](const auto& sample, const auto& pdf) {
        result += func(sample) / pdf;
    });
    return result / T(sample_count);
}

} // namespace pbpt::core
