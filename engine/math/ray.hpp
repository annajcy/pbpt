#pragma once

#include "point.hpp"
#include "global.hpp"
#include "vector.hpp"

/**
 * @file ray.hpp
 * @brief Defines a generic, N-dimensional geometric Ray class.
 * @details This file provides a `Ray` class, a fundamental component in ray tracing
 * and other computer graphics algorithms. A ray is defined by an origin point
 * and a direction vector.
 */

namespace pbpt::math {


/**
 * @class Ray
 * @brief A template class for an N-dimensional geometric ray.
 * @details This class represents a ray in N-dimensional space, defined by a starting
 * point (`origin`) and a direction vector. A key design feature of this class
 * is that the direction vector is **always stored as a normalized (unit) vector**.
 * This invariant is enforced by the constructors, simplifying calculations and
 * ensuring consistent behavior.
 *
 * The ray is described by the parametric equation `P(t) = origin + t * direction`.
 *
 * @tparam T The underlying floating-point type of the ray's components.
 * @tparam N The number of dimensions.
 * @see Point
 * @see Vec
 */
template<typename T, int N>
class Ray {
private:
    Point<T, N> m_origin{};
    Vector<T, N> m_direction{};

public:
    // --- 构造函数 (Constructors) ---

    /**
     * @brief Default constructor.
     * @details Creates a ray starting at the origin `(0,0,...)` and pointing
     * along the positive x-axis `(1,0,...)`.
     */
    constexpr Ray() noexcept
        : m_origin(Point<T, N>::zeros()), m_direction(Vector<T, N>::zeros()) {
        // Default direction is along the x-axis.
        m_direction.x() = 1.0;
    }

    /**
     * @brief Constructs a ray from an origin point and a direction vector.
     * @param origin The starting point of the ray.
     * @param direction The direction vector of the ray.
     * @note The provided `direction` vector will be normalized upon construction
     * to ensure the internal direction is always a unit vector.
     */
    constexpr Ray(const Point<T, N>& origin, const Vector<T, N>& direction)
        : m_origin(origin), m_direction(direction.normalized()) {}

    /**
     * @brief Constructs a ray from a starting point to a target point.
     * @details This creates a ray that starts at `origin` and points towards `target`.
     * The direction is calculated as `(target - origin)` and then normalized.
     * @param origin The starting point of the ray.
     * @param target The point the ray will travel towards.
     */
    constexpr Ray(const Point<T, N>& origin, const Point<T, N>& target)
        : m_origin(origin), m_direction((target - origin).normalized()) {}

    // --- 访问器 (Accessors) ---

    /**
     * @brief Gets the origin point of the ray.
     * @return A const reference to the ray's origin.
     */
    constexpr const Point<T, N>& origin() const noexcept { return m_origin; }

    /**
     * @brief Gets the direction vector of the ray.
     * @return A const reference to the ray's direction vector.
     * @note The returned vector is guaranteed to be a unit vector.
     */
    constexpr const Vector<T, N>& direction() const noexcept { return m_direction; }

    // --- 核心方法 (Core Methods) ---

    /**
     * @brief Calculates a point along the ray using the parametric equation.
     * @details This method evaluates `P(t) = origin + t * direction` to find a
     * point at a specific distance along the ray.
     * @param t The distance parameter. `t` can be thought of as the distance
     * from the origin in units of the direction vector's length (which is 1).
     * Negative values of `t` will yield points "behind" the ray's origin.
     * @return The `Point<T, N>` at the specified parameter `t`.
     */
    constexpr Point<T, N> at(T t) const noexcept {
        return m_origin + m_direction * t;
    }
};

template <typename T, int N>
class RayDifferential : public Ray<T, N> {
public:
    RayDifferential() = default;
    RayDifferential(const Point<T, N>& origin, const Vector<T, N>& direction)
        : Ray<T, N>(origin, direction) {}
    RayDifferential(const Point<T, N>& origin, const Point<T, N>& target)
        : Ray<T, N>(origin, target) {}
};

// --- 类型别名 (Type Aliases) ---

/** @brief A 3-dimensional ray of type `Float`, commonly used in 3D graphics. */
using Ray3 = Ray<Float, 3>;

/** @brief A 2-dimensional ray of type `Float`. */
using Ray2 = Ray<Float, 2>;

} // namespace math