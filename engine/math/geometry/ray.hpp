#pragma once

#include <array>
#include <type_traits>

#include "point.hpp"
#include "vector.hpp"

namespace pbpt::math {

template <typename T, int N>
    requires(N > 0) && (std::is_floating_point_v<T>)
class Ray {
private:
    Point<T, N>  m_origin{};
    Vector<T, N> m_direction{};

public:

    static constexpr Ray<T, N> from_unit_direction(
        const Point<T, N>& origin,
        const Vector<T, N>& direction
    ) {
        Ray<T, N> ray(origin, direction);
        return ray;
    }

    constexpr Ray<T, N>() : m_origin(Point<T, N>::zeros()), m_direction(Vector<T, N>::zeros()) {
        m_direction.x() = 1.0;
    }

    template<typename U>
    constexpr Ray<T, N>(const Point<U, N>& origin, const Vector<U, N>& direction)
        : m_origin(origin), m_direction(direction.normalized()) {}

    template<typename U>
    constexpr Ray<T, N>(const Point<U, N>& origin, const Point<U, N>& target)
        : m_origin(origin), m_direction((target - origin).normalized()) {}

    constexpr const auto& origin() const { return m_origin; }
    constexpr const auto& direction() const { return m_direction; }  

    constexpr auto& origin() { return m_origin; }
    constexpr auto& direction() { return m_direction; }

    template <typename U>
    constexpr Point<T, N> at(U t) const {
        return m_origin + static_cast<T>(t) * m_direction;
    }
};

using Ray3 = Ray<double, 3>;
using Ray2 = Ray<double, 2>;

template <typename T, int N>
class RayDifferential {
private:
    Ray<T, N>                    m_main_ray{};
    std::array<Ray<T, N>, N - 1> m_differential_rays{};

public:
    constexpr RayDifferential<T, N>() = default;
    constexpr RayDifferential<T, N>(
        const Ray<T, N>& ray,
        const std::array<Ray<T, N>, N - 1>& differential_rays
    ) : m_main_ray(ray), m_differential_rays(differential_rays) {}

    constexpr const Ray<T, N>& x() const requires (N > 0) { return m_differential_rays[0]; }
    constexpr Ray<T, N>& x() requires (N > 0) { return m_differential_rays[0]; }

    constexpr const Ray<T, N>& y() const requires (N > 1) { return m_differential_rays[1]; }
    constexpr Ray<T, N>& y() requires (N > 1) { return m_differential_rays[1]; }

    constexpr Ray<T, N>& differential_ray(int index) {
        assert_if([&]() { return index < 0 || index >= N - 1; }, "RayDifferential index out of range");
        return m_differential_rays[index];
    }

    constexpr const Ray<T, N>& differential_ray(int index) const {
        assert_if([&]() { return index < 0 || index >= N - 1; }, "RayDifferential index out of range");
        return m_differential_rays[index];
    }

    constexpr std::array<Ray<T, N>, N - 1>& differential_rays() { return m_differential_rays; }
    constexpr const std::array<Ray<T, N>, N - 1>& differential_rays() const { return m_differential_rays; }

    constexpr const Ray<T, N>& main_ray() const { return m_main_ray; }
    constexpr Ray<T, N>& main_ray() { return m_main_ray; }

    template<typename U>
    requires std::is_arithmetic_v<U>
    auto& scale(U scale) {
        auto& main_o = m_main_ray.origin();
        auto& main_d = m_main_ray.direction();

        for (size_t i = 0; i < m_differential_rays.size(); ++i) {
            auto& o = m_differential_rays[i].origin();
            o = main_o + (o - main_o) * static_cast<T>(scale);

            auto& d = m_differential_rays[i].direction();
            d = main_d + (d - main_d) * static_cast<T>(scale);
        }
        return *this;
    }

    template<typename U>
    auto& scale(const std::array<U, N - 1>& scale) {
        auto& main_o = m_main_ray.origin();
        auto& main_d = m_main_ray.direction();

        for (size_t i = 0; i < m_differential_rays.size(); ++i) {
            auto& o = m_differential_rays[i].origin();
            o = main_o + (o - main_o) * static_cast<T>(scale[i]);

            auto& d = m_differential_rays[i].direction();
            d = main_d + (d - main_d) * static_cast<T>(scale[i]);
        }
        return *this;
    }

    template<typename U>
    requires std::is_arithmetic_v<U>
    auto scaled(U scale) const {
        auto rd = *this;
        rd.scale(scale);
        return rd;
    }

    template<typename U>
    auto scaled(const std::array<U, N - 1>& scale) const {
        auto rd = *this;
        rd.scale(scale);
        return rd;
    }
    
};

using RayDiff2 = RayDifferential<double, 2>;
using RayDiff3 = RayDifferential<double, 3>;

}  // namespace pbpt::math