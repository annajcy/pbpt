/**
 * @file
 * @brief Axis-aligned bounding boxes and ray–box intersection helpers.
 */
#pragma once

#include <array>
#include <type_traits>
#include <optional>
#include <utility>

#include "pbpt/math/function.hpp"
#include "pbpt/math/operator.hpp"
#include "pbpt/math/utils.hpp"
#include "pbpt/math/point.hpp"
#include "pbpt/math/vector.hpp"

#include "ray.hpp"

using namespace pbpt::math;

namespace pbpt::geometry {

/**
 * @brief Axis-aligned bounding box in N dimensions.
 *
 * The box is represented by a minimum and maximum corner in each
 * dimension. A default-constructed @c Bounds is "empty" (min is set to
 * +infinity and max to -infinity) so that it can be grown by union
 * operations.
 *
 * @tparam T Scalar type.
 * @tparam N Dimension of the box.
 */
template <typename T, int N>
class Bounds {
private:
    Point<T, N> m_min;
    Point<T, N> m_max;

public:
    constexpr Bounds<T, N>() noexcept
        : m_min(Point<T, N>::filled(std::numeric_limits<T>::max())),
          m_max(Point<T, N>::filled(std::numeric_limits<T>::lowest())) {}

    /**
     * @brief Construct a bounding box that encloses a list of points.
     *
     * The resulting box is the minimal axis-aligned box that contains
     * all given points.
     */
    template <typename First, typename... Rest>
        requires std::same_as<Point<T, N>, First> && (std::same_as<Point<T, N>, Rest> && ...)
    constexpr explicit Bounds(const First& first, const Rest&... rest)
        : m_min(first), m_max(first) {
        (unite(rest), ...);
    }

    /**
     * @brief Return the corner corresponding to a bit pattern.
     *
     * For each dimension i, bit i of @p index selects either the min
     * (bit = 0) or max (bit = 1) coordinate.
     */
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

    /// Expand this box to include a point.
    constexpr Bounds<T, N>& unite(const Point<T, N>& point) {
        for (int i = 0; i < N; ++i) {
            m_min[i] = std::min(m_min[i], point[i]);
            m_max[i] = std::max(m_max[i], point[i]);
        }
        return *this;
    }

    /// Expand this box to include another box.
    constexpr Bounds<T, N>& unite(const Bounds<T, N>& box) {
        unite(box.min());
        unite(box.max());
        return *this;
    }

    /// Return a new box equal to this box united with a point.
    constexpr Bounds<T, N> united(const Point<T, N>& point) const {
        Bounds<T, N> box(*this);
        box.unite(point);
        return box;
    }

    /// Return a new box equal to the union of this box and another.
    constexpr Bounds<T, N> united(const Bounds<T, N>& box) const {
        Bounds<T, N> box_united(*this);
        box_united.unite(box.min());
        box_united.unite(box.max());
        return box_united;
    }

    /**
     * @brief Check if this box overlaps another box (including touching).
     */
    constexpr bool is_overlapped(const Bounds<T, N>& box) const {
        for (int i = 0; i < N; ++i) {
            if (is_less(m_max[i], box.m_min[i]) || is_greater(m_min[i], box.m_max[i]))
                return false;
        }
        return true;
    }

    /**
     * @brief Compute the intersection of this box with another box.
     *
     * If the boxes do not overlap, the returned box will have min > max
     * in at least one dimension.
     */
    constexpr Bounds<T, N> overlapped_box(const Bounds<T, N>& box) const {
        Bounds<T, N> overlapped_box(*this);
        for (int i = 0; i < N; ++i) {
            overlapped_box.m_min[i] = std::max(overlapped_box.m_min[i], box.m_min[i]);
            overlapped_box.m_max[i] = std::min(overlapped_box.m_max[i], box.m_max[i]);
        }
        return overlapped_box;
    }

    /**
     * @brief Test whether a point lies inside the box.
     *
     * The check is inclusive: points on the boundary are considered
     * inside.
     */
    constexpr bool contains(const Point<T, N>& point) const {
        for (int i = 0; i < N; ++i) {
            if (point[i] < m_min[i] || point[i] > m_max[i]) {
                return false;
            }
        }
        return true;
    }

    /**
     * @brief Return the index of the axis with the largest extent.
     *
     * This is often used to choose a splitting axis for spatial
     * acceleration structures.
     */
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

    /**
     * @brief Map a point inside the box to normalized [0,1] coordinates.
     *
     * Each component is computed as
     * (p[i] - min[i]) / (max[i] - min[i]).
     */
    constexpr Vector<T, N> offset(const Point<T, N>& p) const {
        Vector<T, N> offset;
        for (int i = 0; i < N; ++i) {
            assert_if(is_zero(m_max[i] - m_min[i]), "Zero division error");
            offset[i] = (p[i] - m_min[i]) / (m_max[i] - m_min[i]);
        }
        return offset;
    }

    /**
     * @brief Linearly interpolate within the box given normalized offsets.
     *
     * Each component is computed as
     * min[i] + offset[i] * (max[i] - min[i]).
     */
    constexpr const Point<T, N> interpolate(const std::array<T, N>& offset) const {
        Point<T, N> p;
        for (size_t i = 0; i < N; ++i) {
            p[i] = m_min[i] + offset[i] * (m_max[i] - m_min[i]);
        }
        return p;
    }

    /// Get the minimum corner (const).
    constexpr const Point<T, N>& min() const { return m_min; }
    /// Get the maximum corner (const).
    constexpr const Point<T, N>& max() const { return m_max; }

    /// Get the minimum corner (mutable).
    constexpr Point<T, N>& min() { return m_min; }
    /// Get the maximum corner (mutable).
    constexpr Point<T, N>& max() { return m_max; }

    /// Get the center of the box.
    constexpr Point<T, N> center() const { return m_min.mid(m_max); }
    /// Get the diagonal vector max - min.
    constexpr Vector<T, N> diagonal() const { return m_max - m_min; }

    /// Get the hyper-volume (product of extents in all dimensions).
    constexpr T volume() const { return diagonal().product(); }

    /**
     * @brief Surface area of a 3D box.
     *
     * Only enabled for N == 3.
     */
    constexpr T surface_area() const noexcept
        requires(N == 3)
    {
        auto diag = diagonal();
        return 2 * (diag.x() * diag.y() + diag.y() * diag.z() + diag.z() * diag.x());
    }

};

/**
 * @brief Intersect a ray with an axis-aligned bounding box.
 *
 * Uses the standard slab method to compute the entry and exit parameters
 * (t_min, t_max) along the ray. If there is no intersection within the
 * given @p t_range, returns @c std::nullopt.
 *
 * @tparam T Scalar type.
 * @tparam N Dimension of the space.
 * @param ray     Input ray.
 * @param bounds  Axis-aligned bounding box.
 * @param t_range Initial valid parameter range for the ray.
 * @return Optional pair (t_min, t_max) if the box is hit.
 */
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

/// Axis-aligned bounding box in 3D using the default scalar type.
using Bounds3 = Bounds<Float, 3>;
/// Axis-aligned bounding box in 2D using the default scalar type.
using Bounds2 = Bounds<Float, 2>;

}  // namespace pbpt::geometry
