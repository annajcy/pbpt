#pragma once

#include <array>

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
    constexpr Ray() noexcept : m_origin(Point<T, N>::zeros()), m_direction(Vector<T, N>::zeros()) {
        m_direction.x() = 1.0;
    }

    constexpr Ray(const Point<T, N>& origin, const Vector<T, N>& direction)
        : m_origin(origin), m_direction(direction.normalized()) {}

    constexpr Ray(const Point<T, N>& origin, const Point<T, N>& target)
        : m_origin(origin), m_direction((target - origin).normalized()) {}

    constexpr const Point<T, N>&  origin() const noexcept { return m_origin; }
    constexpr const Vector<T, N>& direction() const noexcept { return m_direction; }

    constexpr Point<T, N> at(T t) const noexcept { return m_origin + t * m_direction; }
};

using Ray3 = Ray<Float, 3>;
using Ray2 = Ray<Float, 2>;

template <typename T, int N>
class RayDifferential {
private:
    Ray<T, N>                    m_ray{};
    std::array<Ray<T, N>, N - 1> m_differential_rays{};

public:
    constexpr RayDifferential() = default;
    constexpr RayDifferential(const Ray<T, N>& ray) : m_ray(ray) {}
    constexpr RayDifferential(const Point<T, N>& origin, const Vector<T, N>& direction) : m_ray(origin, direction) {}
    constexpr RayDifferential(const Point<T, N>& origin, const Point<T, N>& target) : m_ray(origin, target) {}

    constexpr const Ray<T, N>& get_differential_ray(int i) const { return m_differential_rays[i]; }

    constexpr Ray<T, N>& get_differential_ray(int i) { return m_differential_rays[i]; }

    constexpr const Ray<T, N>& ray() const { return m_ray; }

    constexpr Ray<T, N>& ray() { return m_ray; }
};

using RayDiff3 = RayDifferential<Float, 3>;

}  // namespace pbpt::math