#pragma once

#include <concepts>

#include "math/function.hpp"
#include "math/operator.hpp"
#include "math/vector.hpp"

#include "transform.hpp"

using namespace pbpt::math;

namespace pbpt::geometry {

template<typename T>
class DirectionalCone {
private:
    Vector<T, 3> m_direction;
    T m_cosine_theta;

public:

    constexpr static DirectionalCone<T> hemisphere(const Vector<T, 3>& direction = Vector<T, 3>(0, 0, 1)) {
        return DirectionalCone<T>(direction, pi_v<T> / T(2));
    }

    constexpr static DirectionalCone<T> entire_sphere(const Vector<T, 3>& direction = Vector<T, 3>(0, 0, 1)) {
        return DirectionalCone<T>(direction, pi_v<T>);
    }

    constexpr DirectionalCone(const Vector<T, 3>& direction, T angle)
        : m_direction(direction), m_cosine_theta(std::cos(angle)) {}

    constexpr bool contains(const Vector<T, 3>& v) const {
        T cos_theta = m_direction.dot(v.normalized());
        return cos_theta >= m_cosine_theta;
    }

    constexpr bool is_overlapping(const DirectionalCone<T>& other) const {
        if (other.is_degenerate() || this->is_degenerate()) return false;

        T theta_this = safe_acos(m_cosine_theta);
        T theta_other = safe_acos(other.m_cosine_theta);
        T theta_delta = angle_between(m_direction, other.m_direction);

        return is_less_equal(theta_delta, std::min(theta_this + theta_other, pi_v<T>));
    }

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

    constexpr bool is_degenerate() const {
        return m_cosine_theta == std::numeric_limits<T>::infinity();
    }

    constexpr T angle() const {
        return safe_acos(m_cosine_theta);
    }

    constexpr const T& cosine_theta() const {
        return m_cosine_theta;
    }

    constexpr T& cosine_theta() {
        return m_cosine_theta;
    }

    constexpr Vector<T, 3> direction() const {
        return m_direction;
    }

    // Compute the directional cone that bounds all directions from point p to any point in the bounding box
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