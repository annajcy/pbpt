#pragma once
#include <cmath>
#include <concepts>
#include <limits>
#include "math/geometry/transform.hpp"
#include "math/global/function.hpp"
#include "math/global/operator.hpp"
#include "math/global/type_alias.hpp"
#include "vector.hpp"

namespace pbpt::math {

template<std::floating_point T>
class DirectionalCone {
private:
    Vector<T, 3> m_direction;
    T m_cosine_theta;

public:

    constexpr static DirectionalCone<T> hemisphere(const Vector<T, 3>& direction) {
        return DirectionalCone<T>(direction, T(-1));
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
                return DirectionalCone<T>::hemisphere(Vector<T, 3>(0, 0, 1));
            }

            auto rotate = cross(this->m_direction, other.m_direction);
            if (is_zero(rotate.length())) {
                if (this->m_direction.dot(other.m_direction) > 0) {
                    return DirectionalCone<T>(this->m_direction, theta_o);
                } else {
                    return DirectionalCone<T>::hemisphere(Vector<T, 3>(0, 0, 1));
                }
            }
            auto theta_r = theta_o - theta_this;
            auto res = math::Transform<T>::rotate(theta_r, rotate.normalized()) * this->m_direction;
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

};

};