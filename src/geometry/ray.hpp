/**
 * @file
 * @brief Ray and ray-differential primitives used throughout the renderer.
 */
#pragma once

#include <array>
#include <type_traits>

#include "math/function.hpp"
#include "math/utils.hpp"
#include "math/point.hpp"
#include "math/vector.hpp"

using namespace pbpt::math;

namespace pbpt::geometry {

/**
 * @brief Geometric ray with a normalized direction and parameter range.
 *
 * A ray is represented as
 *   origin + t * direction
 * where the direction is kept normalized by the constructors. The valid
 * parameter interval is `[t_min, t_max]`, which is used for intersection
 * routines and range checks in @c at().
 *
 * @tparam T Scalar type (e.g. float or double).
 * @tparam N Dimension of the embedding space.
 */
template <typename T, int N>
class Ray {
private:
    Point<T, N>  m_origin{};
    Vector<T, N> m_direction{};
    T m_tmax{};
    T m_tmin{};

public:

    /**
     * @brief Construct a ray from an origin and a direction assumed to be unit length.
     *
     * This helper does not renormalize the direction and is therefore
     * slightly cheaper when the caller already guarantees a unit vector.
     *
     * @param origin    Ray origin.
     * @param direction Ray direction (assumed normalized).
     * @param tmax      Upper bound of the valid parameter interval.
     * @param tmin      Lower bound of the valid parameter interval.
     * @return Ray with the given attributes.
     */
    static constexpr Ray<T, N> from_unit_direction(
        const Point<T, N>& origin,
        const Vector<T, N>& direction,
        T tmax = std::numeric_limits<T>::infinity(),
        T tmin = -std::numeric_limits<T>::infinity()
    ) {
        Ray<T, N> ray(origin, direction, tmax, tmin);
        return ray;
    }

    /**
     * @brief Default-constructed ray along +x with infinite extent.
     *
     * The origin is set to the zero point, the direction to (1, 0, ...),
     * and the parameter range to [-inf, +inf].
     */
    constexpr Ray<T, N>() : 
    m_origin(Point<T, N>::zeros()), 
    m_direction(Vector<T, N>::zeros()), 
    m_tmax(std::numeric_limits<T>::infinity()), 
    m_tmin(-std::numeric_limits<T>::infinity()) {
        m_direction.x() = 1.0;
    }

    /**
     * @brief Construct a ray from origin and direction.
     *
     * The direction is normalized internally. The default parameter
     * range is [0, +inf), suitable for standard tracing use.
     *
     * @tparam U  Coordinate type convertible to T.
     * @param origin    Ray origin.
     * @param direction Ray direction (will be normalized).
     * @param t_max     Upper bound of the valid parameter interval.
     * @param t_min     Lower bound of the valid parameter interval.
     */
    template<typename U>
    constexpr Ray<T, N>(const Point<U, N>& origin, const Vector<U, N>& direction, T t_max = std::numeric_limits<T>::infinity(), T t_min = static_cast<T>(0.0))
        : m_origin(origin), m_direction(direction.normalized()), m_tmax(t_max), m_tmin(t_min) {}

    /**
     * @brief Construct a ray that points from @p origin to @p target.
     *
     * The direction is the normalized vector (target - origin). The
     * default parameter range is [0, +inf).
     */
    template<typename U>
    constexpr Ray<T, N>(const Point<U, N>& origin, const Point<U, N>& target, T t_max = std::numeric_limits<T>::infinity(), T t_min = static_cast<T>(0.0))
        : m_origin(origin), m_direction((target - origin).normalized()), m_tmax(t_max), m_tmin(t_min) {}

    /// Get the ray origin (const).
    constexpr const auto& origin() const { return m_origin; }
    /// Get the ray direction (const).
    constexpr const auto& direction() const { return m_direction; }  

    /// Get the ray origin (mutable).
    constexpr auto& origin() { return m_origin; }
    /// Get the ray direction (mutable).
    constexpr auto& direction() { return m_direction; }

    /// Get the upper bound of the valid parameter interval (const).
    constexpr const T& t_max() const { return m_tmax; }
    /// Get the upper bound of the valid parameter interval (mutable).
    constexpr T& t_max() { return m_tmax; }
    /// Get the lower bound of the valid parameter interval (const).
    constexpr const T& t_min() const { return m_tmin; }
    /// Get the lower bound of the valid parameter interval (mutable).
    constexpr T& t_min() { return m_tmin; }


    /**
     * @brief Evaluate a point on the ray at parameter t.
     *
     * Asserts that t lies within [t_min, t_max]. The returned point is
     * origin + t * direction.
     *
     * @tparam U Parameter type convertible to T.
     * @param t  Ray parameter.
     * @return Point corresponding to this parameter.
     */
    template <typename U>
    constexpr Point<T, N> at(U t) const {
        assert_if(t < m_tmin || t > m_tmax, "Ray parameter t out of bounds");
        return m_origin + static_cast<T>(t) * m_direction;
    }
};

/// Double-precision 3D ray.
using Ray3 = Ray<double, 3>;
/// Double-precision 2D ray.
using Ray2 = Ray<double, 2>;

/**
 * @brief Ray together with differentials describing small perturbations.
 *
 * A @c RayDifferential stores a "main" ray and up to N-1 offset rays that
 * represent derivatives with respect to image-plane coordinates. These are
 * used by the renderer to estimate footprint size for texture filtering and
 * anti-aliasing.
 *
 * @tparam T Scalar type.
 * @tparam N Dimension of the embedding space.
 */
template <typename T, int N>
class RayDifferential : public Ray<T, N> {
private:
    std::array<Ray<T, N>, N - 1> m_differential_rays{};

public:
    using Ray<T, N>::Ray;
    using Ray<T, N>::origin;
    using Ray<T, N>::direction;

    /**
     * @brief Construct from a main ray and its differential rays.
     *
     * @param ray              Main ray.
     * @param differential_rays Array of offset rays (size N-1).
     */
    constexpr RayDifferential<T, N>(
        const Ray<T, N>& ray,
        const std::array<Ray<T, N>, N - 1>& differential_rays
    ) : Ray<T, N>(ray), m_differential_rays(differential_rays) {}

    /// Access the x-directional differential ray (const).
    constexpr const Ray<T, N>& x() const requires (N > 0) { return m_differential_rays[0]; }
    /// Access the x-directional differential ray (mutable).
    constexpr Ray<T, N>& x() requires (N > 0) { return m_differential_rays[0]; }

    /// Access the y-directional differential ray (const, only for N > 1).
    constexpr const Ray<T, N>& y() const requires (N > 1) { return m_differential_rays[1]; }
    /// Access the y-directional differential ray (mutable, only for N > 1).
    constexpr Ray<T, N>& y() requires (N > 1) { return m_differential_rays[1]; }

    /// Access a differential ray by index (mutable), with bounds checking.
    constexpr Ray<T, N>& differential_ray(int index) {
        assert_if([&]() { return index < 0 || index >= N - 1; }, "RayDifferential index out of range");
        return m_differential_rays[index];
    }

    /// Access a differential ray by index (const), with bounds checking.
    constexpr const Ray<T, N>& differential_ray(int index) const {
        assert_if([&]() { return index < 0 || index >= N - 1; }, "RayDifferential index out of range");
        return m_differential_rays[index];
    }

    /// Get the array of differential rays (mutable).
    constexpr std::array<Ray<T, N>, N - 1>& differential_rays() { return m_differential_rays; }
    /// Get the array of differential rays (const).
    constexpr const std::array<Ray<T, N>, N - 1>& differential_rays() const { return m_differential_rays; }

    /// Access the main ray (const).
    constexpr const Ray<T, N>& main_ray() const { return *this; }
    /// Access the main ray (mutable).
    constexpr Ray<T, N>& main_ray() { return *this; }

    /**
     * @brief Uniformly scale the distance between main and differential rays.
     *
     * Both origins and directions of the differential rays are moved
     * closer to or further from the main ray by @p scale. A scale of 0
     * collapses all differentials back onto the main ray.
     *
     * @tparam U     Scalar type.
     * @param scale  Scale factor applied to differentials.
     * @return Reference to this differential ray.
     */
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

    /**
     * @brief Non-uniformly scale the differentials using per-ray factors.
     *
     * Each differential ray i is scaled by @p scale[i].
     *
     * @tparam U Scalar type of the scale array.
     * @param scale Array of scale factors, one per differential ray.
     * @return Reference to this differential ray.
     */
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

    /**
     * @brief Return a copy with uniformly scaled differentials.
     *
     * @tparam U    Scalar type.
     * @param scale Uniform scale factor.
     * @return New @c RayDifferential with scaled differentials.
     */
    template<typename U>
    requires std::is_arithmetic_v<U>
    auto scaled(U scale) const {
        auto rd = *this;
        rd.scale(scale);
        return rd;
    }

    /**
     * @brief Return a copy with per-ray scaled differentials.
     *
     * @tparam U Scalar type of the scale array.
     * @param scale Array of scale factors, one per differential ray.
     * @return New @c RayDifferential with scaled differentials.
     */
    template<typename U>
    auto scaled(const std::array<U, N - 1>& scale) const {
        auto rd = *this;
        rd.scale(scale);
        return rd;
    }
    
};

/**
 * @brief Compute a conservative t_max for a ray of length @p dist.
 *
 * The returned value is slightly smaller than @p dist by an amount
 * proportional to a small gamma factor, which helps avoid "overshooting"
 * due to floating-point error when tracing to a known distance.
 */
template <typename T>
inline T safe_ray_tmax(T dist) {
    return dist * (T(1) - gamma<T>(3));
}

/// Double-precision 2D ray differential.
using RayDiff2 = RayDifferential<double, 2>;
/// Double-precision 3D ray differential.
using RayDiff3 = RayDifferential<double, 3>;

}  // namespace pbpt::geometry
