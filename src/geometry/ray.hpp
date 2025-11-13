#pragma once

#include <array>
#include <type_traits>

#include "math/function.hpp"
#include "math/utils.hpp"
#include "math/point.hpp"
#include "math/vector.hpp"

using namespace pbpt::math;

namespace pbpt::geometry {

template <typename T, int N>
class Ray {
private:
    Point<T, N>  m_origin{};
    Vector<T, N> m_direction{};
    T m_tmax{};
    T m_tmin{};

public:

    static constexpr Ray<T, N> from_unit_direction(
        const Point<T, N>& origin,
        const Vector<T, N>& direction,
        T tmax = std::numeric_limits<T>::infinity(),
        T tmin = -std::numeric_limits<T>::infinity()
    ) {
        Ray<T, N> ray(origin, direction, tmax, tmin);
        return ray;
    }

    constexpr Ray<T, N>() : 
    m_origin(Point<T, N>::zeros()), 
    m_direction(Vector<T, N>::zeros()), 
    m_tmax(std::numeric_limits<T>::infinity()), 
    m_tmin(-std::numeric_limits<T>::infinity()) {
        m_direction.x() = 1.0;
    }

    template<typename U>
    constexpr Ray<T, N>(const Point<U, N>& origin, const Vector<U, N>& direction, T t_max = std::numeric_limits<T>::infinity(), T t_min = static_cast<T>(0.0))
        : m_origin(origin), m_direction(direction.normalized()), m_tmax(t_max), m_tmin(t_min) {}

    template<typename U>
    constexpr Ray<T, N>(const Point<U, N>& origin, const Point<U, N>& target, T t_max = std::numeric_limits<T>::infinity(), T t_min = static_cast<T>(0.0))
        : m_origin(origin), m_direction((target - origin).normalized()), m_tmax(t_max), m_tmin(t_min) {}

    constexpr const auto& origin() const { return m_origin; }
    constexpr const auto& direction() const { return m_direction; }  

    constexpr auto& origin() { return m_origin; }
    constexpr auto& direction() { return m_direction; }

    constexpr const T& t_max() const { return m_tmax; }
    constexpr T& t_max() { return m_tmax; }
    constexpr const T& t_min() const { return m_tmin; }
    constexpr T& t_min() { return m_tmin; }


    template <typename U>
    constexpr Point<T, N> at(U t) const {
        assert_if(t < m_tmin || t > m_tmax, "Ray parameter t out of bounds");
        return m_origin + static_cast<T>(t) * m_direction;
    }
};

using Ray3 = Ray<double, 3>;
using Ray2 = Ray<double, 2>;

template <typename T, int N>
class RayDifferential : public Ray<T, N> {
private:
    std::array<Ray<T, N>, N - 1> m_differential_rays{};

public:
    using Ray<T, N>::Ray;
    using Ray<T, N>::origin;
    using Ray<T, N>::direction;

    constexpr RayDifferential<T, N>(
        const Ray<T, N>& ray,
        const std::array<Ray<T, N>, N - 1>& differential_rays
    ) : Ray<T, N>(ray), m_differential_rays(differential_rays) {}

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

    constexpr const Ray<T, N>& main_ray() const { return *this; }
    constexpr Ray<T, N>& main_ray() { return *this; }

    template<typename U>
    requires std::is_arithmetic_v<U>
    auto& scale(U scale) {
        auto& main_o = origin();
        auto& main_d = direction();

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
        auto& main_o = origin();
        auto& main_d = direction();

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

template <typename T>
inline T safe_ray_tmax(T dist) {
    return dist * (T(1) - gamma<T>(3));
}

using RayDiff2 = RayDifferential<double, 2>;
using RayDiff3 = RayDifferential<double, 3>;

}  // namespace pbpt::geometry