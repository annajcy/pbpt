#pragma once

#include <cstdlib>
#include <utility>

#include "math/function.hpp"
#include "math/normal.hpp"
#include "math/point.hpp"
#include "math/vector.hpp"
#include "math/sampling.hpp"

namespace pbpt::radiometry {

template<typename SampleType, typename T>
using Sample = std::pair<SampleType, T>;

template<typename Derived, typename T>
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

template<typename T>
using DirectionSample = Sample<math::Vector<T, 3>, T>;

template<typename T>
class UniformHemisphereDomain : public IntegrableDomain<UniformHemisphereDomain<T>, T> {
public:

    using sample_type = math::Vector<T,3>;

    template<typename RNG>
    DirectionSample<T> sample_one_impl(RNG& rng2d) const {
        auto uv = math::Point<T, 2>::from_array(rng2d.generate_uniform(T(0), T(1))); // uv ∈ [0,1]^2
        auto dir = math::sample_uniform_hemisphere(uv).to_vector();
        return {dir, pdf_impl()};
    }

    constexpr T pdf_impl() const {
        return T(1) / (2 * math::pi_v<T>);
    }
};

template<typename T>
class ProjectedHemisphereDomain : public IntegrableDomain<ProjectedHemisphereDomain<T>, T> {
public:
    using sample_type = math::Vector<T, 3>;

    template<typename RNG>
    DirectionSample<T> sample_one_impl(RNG& rng) const {
        auto uv = math::Point<T, 2>::from_array(rng.generate_uniform(T(0), T(1)));
        auto dir = math::sample_uniform_hemisphere_concentric(uv).to_vector();
        return { dir, pdf_impl() };
    }

    constexpr T pdf_impl() const { return T(1) / math::pi_v<T>; } // wrt dω⊥
};

template<typename T>
using DiskSample = Sample<math::Point<T, 2>, T>;

template<typename T>
class UniformDiskDomain : public IntegrableDomain<UniformDiskDomain<T>, T> {
public:
    using sample_type = math::Point<T, 2>;

    template<typename RNG>
    DiskSample<T> sample_one_impl(RNG& rng2d) const {
        auto u01 = math::Point<T, 2>::from_array(rng2d.generate_uniform(0, 1));
        auto point = math::sample_uniform_disk_concentric(u01);
        return {point, pdf_impl()};
    }

    constexpr T pdf_impl() const {
        return T(1) / (math::pi_v<T>);
    }
};


template<typename T>
struct SurfaceInfo {
    math::Point<T, 3> position;
    math::Normal<T, 3> normal;
};

template<typename T>
using SurfaceSample = Sample<SurfaceInfo<T>, T>;

template<typename T>
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

template<typename T, typename Domain, typename Func, typename RNG>
auto integrate(const Domain& domain, const Func& func, int sample_count, RNG& rng) {
    using R = decltype(std::declval<const Func&>()(std::declval<typename Domain::sample_type>()) / static_cast<T>(std::declval<int>()));
    R result = R{};
    domain.foreach_sample(sample_count, rng, [&](const auto& sample, const auto& pdf) {
        result += func(sample) / pdf;
    });
    return result / T(sample_count);
}

} 
