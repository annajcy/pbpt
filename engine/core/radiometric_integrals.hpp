#pragma once

#include <array>
#include <cstdlib>
#include <cmath>
#include <concepts>
#include <algorithm>

#include "math/point.hpp"
#include "math/vector.hpp"

namespace pbpt::core {

template<std::floating_point T>
inline std::array<T, 2> concentric_sample_disk(const std::array<T, 2>& u01) {
    const T a = T(2) * u01[0] - T(1);
    const T b = T(2) * u01[1] - T(1);

    if (a == 0 && b == 0)
        return {0, 0};

    T r, phi;
    if (abs(a) > abs(b)) {
        r = abs(a);
        phi = (math::pi_v<T> / 4) * (b / a);
    } else {
        r = abs(b);
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

template<std::floating_point T>
using DirectionSample = std::pair<math::Vector<T, 3>, T>;

template<std::floating_point T>
class UniformHemisphereDomain {
public:
    using sample_type = math::Vector<T,3>;

    template<typename RNG>
    DirectionSample<T> sample_one(RNG& rng2d) const {
        auto uv = rng2d.generate_uniform(0, 1); // uv ∈ [0,1]^2
        auto dir = math::Vector<T, 3>::from_array(uniform_sample_hemisphere(uv));
        return {dir, pdf()};
    }

    template<typename RNG, typename Func>
    void foreach_sample(int count, RNG& rng2d, Func&& func) const {
        for (int i = 0; i < count; ++i) {
            auto [dir, pdf] = sample_one(rng2d);
            func(dir, pdf);
        }
    }

    static constexpr T pdf() {
        return T(1) / (2 * math::pi_v<T>);
    }
};

template<std::floating_point T>
class ProjectedHemisphereDomain {
public:
    using sample_type = math::Vector<T, 3>;

    template<typename RNG>
    DirectionSample<T> sample_one(RNG& rng) const {
        auto uv = rng.generate_uniform(T(0), T(1));
        auto d  = concentric_sample_disk<T>(uv);
        const T r2 = d[0] * d[0] + d[1] * d[1];
        const T z  = std::sqrt(std::max(T(0), T(1) - r2));
        return {math::Vector<T,3>{ d[0], d[1], z }, pdf()};
    }

    template<typename RNG, typename Func>
    void foreach_sample(int count, RNG& rng, Func&& func) const {
        for (int i = 0; i < count; ++i) {
            auto [sample, pdf] = sample_one(rng);
            func(sample, pdf);
        }
    }

    static constexpr T pdf() { return T(1) / math::pi_v<T>; } // wrt dω⊥
};

template<std::floating_point T>
using DiskSample = std::pair<math::Point<T, 2>, T>;

template<std::floating_point T>
class UniformDiskDomain {
public:
    using sample_type = math::Point<T, 2>;

    template<typename RNG>
    DiskSample<T> sample_one(RNG& rng2d) const {
        auto u01 = rng2d.generate_uniform(0, 1);
        auto point = math::Point<T, 2>::from_array(concentric_sample_disk(u01));
        return {point, pdf()};
    }

    template<typename RNG, typename Func>
    void foreach_sample(int count, RNG& rng2d, Func&& func) const {
        for (int i = 0; i < count; ++i) {
            auto [point, pdf] = sample_one(rng2d);
            func(point, pdf);
        }
    }

    static constexpr T pdf() {
        return T(1) / (math::pi_v<T>);
    }
};


template<std::floating_point T, typename Domain, typename Func, typename RNG>
auto integrate(const Domain& domain, const Func& func, int sample_count, RNG& rng) {
    using R = decltype(func(std::declval<typename Domain::sample_type>()) / T(1));
    R result = R{};
    domain.foreach_sample(sample_count, rng, [&](const auto& sample, const auto& pdf) {
        result += func(sample) / pdf;
    });
    return result / T(sample_count);
}

} // namespace pbpt::core
