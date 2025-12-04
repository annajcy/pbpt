/**
 * @file
 * @brief Directional cones in 3D used for bounding sets of directions.
 */
#pragma once

#include <concepts>

#include "math/function.hpp"
#include "math/operator.hpp"
#include "math/vector.hpp"

#include "transform.hpp"

using namespace pbpt::math;

namespace pbpt::geometry {

/**
 * @brief Solid angle cone around a unit direction.
 *
 * A directional cone is described by a unit axis vector and a maximum
 * opening angle. It is primarily used to bound sets of directions for
 * visibility or importance sampling.
 *
 * @tparam T Scalar type.
 */
template<typename T>
class DirectionalCone {
private:
    Vector<T, 3> m_direction;
    T m_cosine_theta;

public:

    /**
     * @brief Construct a cone that covers a hemisphere.
     *
     * The opening angle is 90 degrees around @p direction.
     */
    constexpr static DirectionalCone<T> hemisphere(const Vector<T, 3>& direction = Vector<T, 3>(0, 0, 1)) {
        return DirectionalCone<T>(direction, pi_v<T> / T(2));
    }

    /**
     * @brief Construct a cone that covers the whole sphere.
     *
     * The opening angle is 180 degrees; all directions are inside.
     */
    constexpr static DirectionalCone<T> entire_sphere(const Vector<T, 3>& direction = Vector<T, 3>(0, 0, 1)) {
        return DirectionalCone<T>(direction, pi_v<T>);
    }

    /**
     * @brief Construct a cone from an axis direction and opening angle.
     *
     * @param direction Unit axis of the cone (will be normalized by caller).
     * @param angle     Half-angle of the cone in radians.
     */
    constexpr DirectionalCone(const Vector<T, 3>& direction, T angle)
        : m_direction(direction), m_cosine_theta(std::cos(angle)) {}

    /**
     * @brief Test whether a direction lies inside the cone.
     */
    constexpr bool contains(const Vector<T, 3>& v) const {
        T cos_theta = m_direction.dot(v.normalized());
        return cos_theta >= m_cosine_theta;
    }

    /**
     * @brief Test whether this cone overlaps another cone.
     *
     * Two cones overlap if their axes are close enough that their
     * opening angles intersect on the sphere.
     */
    constexpr bool is_overlapping(const DirectionalCone<T>& other) const {
        if (other.is_degenerate() || this->is_degenerate()) return false;

        T theta_this = safe_acos(m_cosine_theta);
        T theta_other = safe_acos(other.m_cosine_theta);
        T theta_delta = angle_between(m_direction, other.m_direction);

        return is_less_equal(theta_delta, std::min(theta_this + theta_other, pi_v<T>));
    }

    /**
     * @brief Return the minimal cone that contains both this and @p other.
     *
     * If one cone fully contains the other, it is returned as-is. If the
     * union covers the entire sphere, an @c entire_sphere cone is returned.
     */
    constexpr DirectionalCone<T> united(const DirectionalCone<T>& other) {
        if (other.is_degenerate()) return *this;
        if (this->is_degenerate()) return other;

        T theta_delta = angle_between(m_direction, other.m_direction);
        T theta_this = safe_acos(m_cosine_theta);
        T theta_other = safe_acos(other.m_cosine_theta);

        if (is_less_equal(std::min(theta_delta + theta_other, pi_v<T>), theta_this)) {
            return *this;
        } else if (is_less_equal(std::min(theta_delta + theta_this, pi_v<T>), theta_other)) {
            return other;
        } else {
            T theta_o = (theta_this + theta_other + theta_delta) / 2;
            if (is_greater_equal(theta_o, pi_v<T>)) {
                return DirectionalCone<T>::entire_sphere(Vector<T, 3>(0, 0, 1));
            }

            auto rotate = cross(this->m_direction, other.m_direction);
            if (is_zero(rotate.length())) {
                if (this->m_direction.dot(other.m_direction) > 0) {
                    return DirectionalCone<T>(this->m_direction, theta_o);
                } else {
                    return DirectionalCone<T>::entire_sphere(Vector<T, 3>(0, 0, 1));
                }
            }
            auto theta_r = theta_o - theta_this;
            auto res = Transform<T>::rotate(theta_r, rotate.normalized()).transform_vector(this->m_direction);
            return DirectionalCone<T>(res, theta_o);
        }
    }

    /// Check whether the cone is degenerate (infinite cosine, no valid angle).
    constexpr bool is_degenerate() const {
        return m_cosine_theta == std::numeric_limits<T>::infinity();
    }

    /// Get the opening angle in radians.
    constexpr T angle() const {
        return safe_acos(m_cosine_theta);
    }

    /// Get the cosine of the opening angle (const).
    constexpr const T& cosine_theta() const {
        return m_cosine_theta;
    }

    /// Get the cosine of the opening angle (mutable).
    constexpr T& cosine_theta() {
        return m_cosine_theta;
    }

    /// Get the cone axis direction.
    constexpr Vector<T, 3> direction() const {
        return m_direction;
    }

    /**
     * @brief Bound all directions from a point to an axis-aligned box.
     *
     * Constructs the smallest cone that contains all directions from
     * point @p p to any point inside @p bounds. If @p p lies inside
     * the box, the entire sphere is returned.
     */
    static DirectionalCone<T> bound_subtended_directions(const Point<T, 3>& p, const Bounds<T, 3>& bounds) {
        if (bounds.contains(p)) {
            return DirectionalCone<T>::entire_sphere(Vector<T, 3>(0, 0, 1));
        }

        Point<T, 3> sphere_center = bounds.center();
        Vector<T, 3> diagonal = bounds.diagonal();
        T sphere_radius = diagonal.length() / T(2);
        Vector<T, 3> direction = (sphere_center - p).normalized();
        T dist_to_center = p.distance(sphere_center);
        
        T angle;
        if (dist_to_center <= sphere_radius) {
            return DirectionalCone<T>::entire_sphere(direction);
        } else {
            T sin_theta = sphere_radius / dist_to_center;
            angle = safe_asin(sin_theta);
        }
        
        return DirectionalCone<T>(direction, angle);
    }



};

};
