#pragma once

#include <array>
#include <type_traits>
#include <optional>
#include <utility>

#include "math/function.hpp"
#include "math/operator.hpp"
#include "math/utils.hpp"
#include "math/point.hpp"
#include "math/vector.hpp"

#include "ray.hpp"

using namespace pbpt::math;

namespace pbpt::geometry {

template <typename T, int N>
class Bounds {
private:
    Point<T, N> m_min;
    Point<T, N> m_max;

public:
    constexpr Bounds<T, N>() noexcept
        : m_min(Point<T, N>::filled(std::numeric_limits<T>::max())),
          m_max(Point<T, N>::filled(std::numeric_limits<T>::lowest())) {}

    template <typename First, typename... Rest>
        requires std::same_as<Point<T, N>, First> && (std::same_as<Point<T, N>, Rest> && ...)
    constexpr explicit Bounds(const First& first, const Rest&... rest)
        : m_min(first), m_max(first) {
        (unite(rest), ...);
    }

    Point<T, N> corner(int index) const {
        Point<T, N> p;
        for (int i = 0; i < N; i ++) {
            if ((index & (1 << i)) == 0) {
                p[i] = m_min[i];
            } else {
                p[i] = m_max[i];
            }
        }
        return p;
    }

    constexpr Bounds<T, N>& unite(const Point<T, N>& point) {
        for (int i = 0; i < N; ++i) {
            m_min[i] = std::min(m_min[i], point[i]);
            m_max[i] = std::max(m_max[i], point[i]);
        }
        return *this;
    }

    constexpr Bounds<T, N>& unite(const Bounds<T, N>& box) {
        unite(box.min());
        unite(box.max());
        return *this;
    }

    constexpr Bounds<T, N> united(const Point<T, N>& point) const {
        Bounds<T, N> box(*this);
        box.unite(point);
        return box;
    }

    constexpr Bounds<T, N> united(const Bounds<T, N>& box) const {
        Bounds<T, N> box_united(*this);
        box_united.unite(box.min());
        box_united.unite(box.max());
        return box_united;
    }

    constexpr bool is_overlapped(const Bounds<T, N>& box) const {
        for (int i = 0; i < N; ++i) {
            if (is_less(m_max[i], box.m_min[i]) || is_greater(m_min[i], box.m_max[i]))
                return false;
        }
        return true;
    }

    constexpr Bounds<T, N> overlapped_box(const Bounds<T, N>& box) const {
        Bounds<T, N> overlapped_box(*this);
        for (int i = 0; i < N; ++i) {
            overlapped_box.m_min[i] = std::max(overlapped_box.m_min[i], box.m_min[i]);
            overlapped_box.m_max[i] = std::min(overlapped_box.m_max[i], box.m_max[i]);
        }
        return overlapped_box;
    }

    constexpr bool contains(const Point<T, N>& point) const {
        for (int i = 0; i < N; ++i) {
            if (point[i] < m_min[i] || point[i] > m_max[i]) {
                return false;
            }
        }
        return true;
    }

    constexpr int max_extent() const {
        int max_extent = 0;
        T   max_diff   = m_max[0] - m_min[0];
        for (int i = 1; i < N; ++i) {
            T diff = m_max[i] - m_min[i];
            if (diff > max_diff) {
                max_diff   = diff;
                max_extent = i;
            }
        }
        return max_extent;
    }

    constexpr Vector<T, N> offset(const Point<T, N>& p) const {
        Vector<T, N> offset;
        for (int i = 0; i < N; ++i) {
            assert_if(is_zero(m_max[i] - m_min[i]), "Zero division error");
            offset[i] = (p[i] - m_min[i]) / (m_max[i] - m_min[i]);
        }
        return offset;
    }

    constexpr const Point<T, N> interpolate(const std::array<T, N>& offset) const {
        Point<T, N> p;
        for (size_t i = 0; i < N; ++i) {
            p[i] = m_min[i] + offset[i] * (m_max[i] - m_min[i]);
        }
        return p;
    }

    constexpr const Point<T, N>& min() const { return m_min; }
    constexpr const Point<T, N>& max() const { return m_max; }

    constexpr Point<T, N>& min() { return m_min; }
    constexpr Point<T, N>& max() { return m_max; }

    constexpr Point<T, N> center() const { return m_min.mid(m_max); }
    constexpr Vector<T, N> diagonal() const { return m_max - m_min; }

    constexpr T volume() const { return diagonal().product(); }

    constexpr T surface_area() const noexcept
        requires(N == 3)
    {
        auto diag = diagonal();
        return 2 * (diag.x() * diag.y() + diag.y() * diag.z() + diag.z() * diag.x());
    }

};

template <typename T, int N>
std::optional<std::pair<T, T>> intersect_ray_bounds(
    const Ray<T, N>& ray,
    const Bounds<T, N>& bounds,
    std::pair<T, T> t_range = {
        -std::numeric_limits<T>::infinity(),
         std::numeric_limits<T>::infinity()
    }
) {
    T t_min = t_range.first;
    T t_max = t_range.second;

    const auto& o = ray.origin();
    const auto& d = ray.direction();

    for (int i = 0; i < N; ++i) {
        if (is_zero(d[i])) {
            // 平行该轴方向的 slab
            if (o[i] < bounds.min()[i] || o[i] > bounds.max()[i])
                return std::nullopt;
        } else {
            T inv_d = T(1) / d[i];
            T t0 = (bounds.min()[i] - o[i]) * inv_d;
            T t1 = (bounds.max()[i] - o[i]) * inv_d;

            if (t0 > t1) std::swap(t0, t1);

            t_min = std::max(t_min, t0);
            t_max = std::min(t_max, t1);

            if (t_min > t_max)
                return std::nullopt;
        }
    }

    return std::pair{t_min, t_max};
}

using Bounds3 = Bounds<Float, 3>;
using Bounds2 = Bounds<Float, 2>;

}  // namespace pbpt::geometry
