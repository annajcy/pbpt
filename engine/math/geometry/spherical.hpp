#pragma once

#include <algorithm>
#include <cmath>
#include <concepts>
#include <cstdlib>

#include "../global/type_alias.hpp"
#include "math/global/function.hpp"
#include "math/global/operator.hpp"
#include "point.hpp"
#include "vector.hpp"

namespace pbpt::math {

template<typename T>
inline constexpr T wrap_angle_2pi(T phi) {
    if (is_less(phi, T(0))) phi += T(2) * pi_v<T>;
    else if (is_greater_equal(phi, T(2) * pi_v<T>)) phi -= T(2) * pi_v<T>;
    else return phi;
    return wrap_angle_2pi(phi);
}

/**
 * @class SphericalPoint
 * @brief N维球坐标点类
 * @details 球坐标系表示：
 * - 2D: (r, φ) - 半径和方位角
 * - 3D: (r, θ, φ) - 半径、极角、方位角
 * - ND: (r, θ₁, θ₂, ..., θₙ₋₂, φ) - 半径、N-2个极角和方位角
 *
 * 角度约定：
 * - θᵢ ∈ [0, π] for i = 1, ..., N-2 (极角)
 * - φ ∈ [0, 2π) (方位角，角度向量的最后一个元素)
 */
template <typename T, int N>
class SphericalPoint {
public:
    Vector<T, N - 1> m_spherical;  // N-1个角度
    T                m_radius;     // 半径

    static constexpr auto from_cartesian(const Point<T, N>& cartesian) {
        return SphericalPoint(cartesian);
    }

    constexpr SphericalPoint(const Vector<T, N - 1>& spherical, T radius) : m_spherical(spherical), m_radius(radius) {}
    constexpr SphericalPoint(const Point<T, N>& cartesian) {
        m_radius = cartesian.to_vector().length();

        if (is_zero(m_radius)) {
            m_spherical = Vector<T, N - 1>::zeros();
            return;
        }

        if constexpr (N == 2) {
            // 2D情况: φ = atan2(y, x) (方位角)
            m_spherical[0] = wrap_angle_2pi(atan2(cartesian.y(), cartesian.x()));
        } else if constexpr (N == 3) {
            // 3D情况: θ = acos(z/r), φ = atan2(y, x)
            m_spherical[0] = wrap_angle_2pi(acos(cartesian.z() / m_radius));  // 极角
            m_spherical[1] = wrap_angle_2pi(atan2(cartesian.y(), cartesian.x()));  // 方位角
        } else {
            // N维情况的递归计算
            cartesian_to_spherical_nd(cartesian);
        }
    }

    constexpr Point<T, N> to_cartesian() const {
        if (m_radius == 0) {
            return Point<T, N>::zeros();
        }

        Vector<T, N> result;

        if constexpr (N == 2) {
            // 2D: x = r*cos(φ), y = r*sin(φ)
            result[0] = m_radius * cos(m_spherical[0]);
            result[1] = m_radius * sin(m_spherical[0]);
        } else if constexpr (N == 3) {
            // 3D: x = r*sin(θ)*cos(φ), y = r*sin(θ)*sin(φ), z = r*cos(θ)
            T sin_theta = sin(m_spherical[0]);
            result[0]   = m_radius * sin_theta * cos(m_spherical[1]);
            result[1]   = m_radius * sin_theta * sin(m_spherical[1]);
            result[2]   = m_radius * cos(m_spherical[0]);
        } else {
            // N维情况
            spherical_to_cartesian_nd(result);
        }

        return Point<T, N>::from_vector(result);
    }

    constexpr T radius() const { return m_radius; }
    constexpr T angle(int i) const { return m_spherical[i]; }
    constexpr const Vector<T, N - 1>& angles() const { return m_spherical; }

    constexpr T azimuth() const {
        static_assert(N >= 2, "Azimuth angle only exists for N >= 2");
        return m_spherical[N - 2];
    }

private:

    constexpr void cartesian_to_spherical_nd(const Point<T, N>& cartesian) {
        // N维球坐标的一般公式（方位角在最后）
        // θᵢ = acos(xᵢ₊₂ / sqrt(x₁² + x₂² + ... + xᵢ₊₂²)) for i = 0, ..., N-3
        // (极角) φ = atan2(x₂, x₁) (方位角)

        // 计算极角 (前N-2个角度)
        for (int i = 0; i < N - 2; ++i) {
            T sum_squares = 0;
            for (int j = 0; j <= i + 2; ++j) {
                sum_squares += cartesian[j] * cartesian[j];
            }
            T partial_radius = sqrt(sum_squares);

            if (is_zero(partial_radius)) {
                m_spherical[i] = 0;
            } else {
                m_spherical[i] = acos(cartesian[i + 2] / partial_radius);
            }
        }

        // 最后一个角度 (方位角)
        m_spherical[N - 2] = wrap_angle_2pi(atan2(cartesian[1], cartesian[0]));
    }

    constexpr void spherical_to_cartesian_nd(Vector<T, N>& result) const {
        // N维球坐标到笛卡尔的一般公式（方位角在最后）
        // x₁ = r * cos(φ) * ∏ᵢ₌₀ᴺ⁻³ sin(θᵢ)
        // x₂ = r * sin(φ) * ∏ᵢ₌₀ᴺ⁻³ sin(θᵢ)
        // xₖ = r * cos(θₖ₋₂) * ∏ᵢ₌ₖ₋₂ᴺ⁻³ sin(θᵢ) for k = 3, ..., N-1
        // xₙ = r * cos(θₙ₋₃)

        // 计算所有sin值的累积乘积
        Vector<T, N - 1> sin_products;
        sin_products[N - 2] = T(1);  // 最后一个元素（方位角位置）

        for (int i = N - 3; i >= 0; --i) {
            sin_products[i] = sin_products[i + 1] * sin(m_spherical[i]);
        }

        // 计算笛卡尔坐标
        T azimuth_angle = m_spherical[N - 2];  // 方位角
        result[0]       = m_radius * cos(azimuth_angle) * sin_products[0];
        result[1]       = m_radius * sin(azimuth_angle) * sin_products[0];

        for (int i = 2; i < N; ++i) {
            if (i == N - 1) {
                // 最后一个坐标
                result[i] = m_radius * cos(m_spherical[i - 2]);
            } else {
                result[i] = m_radius * cos(m_spherical[i - 2]) * sin_products[i - 1];
            }
        }
    }
};

template<std::floating_point T>
inline constexpr auto cos_theta(const Vector<T, 3>& v) {
    return v.z();
} 

template <std::floating_point T>
inline constexpr auto cos_theta_sq(const Vector<T, 3>& v) {
    return v.z() * v.z();
}

template<std::floating_point T>
inline constexpr auto sin_theta_sq(const Vector<T, 3>& v) {
    return std::max(T(0), T(1) - cos_theta_sq(v));
}

template<std::floating_point T>
inline constexpr auto sin_theta(const Vector<T, 3>& v) {
    return std::sqrt(sin_theta_sq(v));
}

template <std::floating_point T>
inline constexpr auto tan_theta(const Vector<T, 3>& v) {
    return sin_theta(v) / cos_theta(v);
}

template <std::floating_point T>
inline constexpr auto tan_theta_sq(const Vector<T, 3>& v) {
    return sin_theta_sq(v) / cos_theta_sq(v);
}

template<std::floating_point T>
inline constexpr auto phi(const Vector<T, 3>& v) {
    auto p = std::atan2(v.y(), v.x());
    return wrap_angle_2pi(p);
}

template<std::floating_point T>
inline constexpr auto sin_phi(const Vector<T, 3>& v) {
    auto s_th = sin_theta(v);
    return is_zero(s_th) ? T(0) : std::clamp(v.y() / s_th, T(-1), T(1));
}

template<std::floating_point T>
inline constexpr auto cos_phi(const Vector<T, 3>& v) {
    auto s_th = sin_theta(v);
    return is_zero(s_th) ? T(1) : std::clamp(v.x() / s_th, T(-1), T(1));
}

template<std::floating_point T>
inline constexpr auto cos_delta_phi(const Vector<T, 3>& a, const Vector<T, 3>& b) {
    auto axy = a.x() * a.x() + a.y() * a.y();
    auto bxy = b.x() * b.x() + b.y() * b.y();
    return (is_zero(axy) || is_zero(bxy)) ? T(1) : std::clamp(
        (a.x() * b.x() + a.y() * b.y()) / std::sqrt(axy * bxy), 
        T(-1), T(1)
    );
}

template<std::floating_point T>
inline constexpr Vector<T, 3> equal_area_square_to_sphere(const Vector<T, 2>& p) {
    T u = 2 * p.x() - 1, v = 2 * p.y() - 1;
    T up = std::abs(u), vp = std::abs(v);
    T signed_distance = 1 - (up + vp);
    T d = std::abs(signed_distance);
    T r = 1 - d;

    T phi = (r == 0 ? 1 : (vp - up) / (r + 1)) * (pi_v<T> / 4);
    T z = std::copysign(1 - r * r, signed_distance);

    T cos_phi = std::copysign(std::cos(phi), u);
    T sin_phi = std::copysign(std::sin(phi), v);

    return Vector<T, 3>(
        cos_phi * r * std::sqrt(2 - r * r), 
        sin_phi * r * std::sqrt(2 - r * r), 
        z
    );
}

template<std::floating_point T>
inline constexpr Vector<T, 2> equal_area_sphere_to_square(const Vector<T, 3>& w) {
// TODO
}

template<std::floating_point T>
inline constexpr T spherical_triangle_area(
    const Vector<T, 3>& a,
    const Vector<T, 3>& b,
    const Vector<T, 3>& c
) {
    return std::abs(T(2) * std::atan2(
        a.dot(cross(b, c)),
        T(1) + a.dot(b) + b.dot(c) + c.dot(a)
    ));
}

template<std::floating_point T>
constexpr T spherical_polygon_area(
    const std::vector<Vector<T, 3>>& vertices
) {
    assert_if([&vertices]() { return vertices.size() < 3; }, "At least 3 vertices are required for a polygon");
    const auto& a = vertices[0];
    T area = T(0);
    for (int i = 2; i < vertices.size(); ++i) {
        area += spherical_triangle_area(a, vertices[i - 1], vertices[i]);
    }
    return area;
}

using Sphere2 = SphericalPoint<Float, 2>;
using Sphere3 = SphericalPoint<Float, 3>;

}  // namespace pbpt::math