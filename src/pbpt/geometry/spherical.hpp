/**
 * @file
 * @brief Spherical coordinate helpers and equal-area mappings.
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdlib>

#include "pbpt/math/function.hpp"
#include "pbpt/math/operator.hpp"
#include "pbpt/math/vector.hpp"
#include "pbpt/math/point.hpp"
#include "pbpt/math/type_alias.hpp"

using namespace pbpt::math;

namespace pbpt::geometry {

/**
 * @brief Wrap an angle into the [0, 2π) range.
 *
 * Repeatedly adds or subtracts 2π until the value lies in [0, 2π).
 */
template<typename T>
inline constexpr T wrap_angle_2pi(T phi) {
    if (is_less(phi, T(0))) phi += T(2) * pi_v<T>;
    else if (is_greater_equal(phi, T(2) * pi_v<T>)) phi -= T(2) * pi_v<T>;
    else return phi;
    return wrap_angle_2pi(phi);
}

/**
 * @brief Point expressed in N-dimensional spherical coordinates.
 *
 * Spherical coordinates are stored as:
 * - 2D: (r, phi)         – radius and azimuth angle
 * - 3D: (r, theta, phi)  – radius, polar angle, azimuth angle
 * - ND: (r, theta_1, ..., theta_{N-2}, phi)
 *
 * Angle conventions:
 * - Each polar angle theta_i is in [0, pi].
 * - The last angle phi is the azimuth in [0, 2*pi).
 *
 * @tparam T Scalar type.
 * @tparam N Dimension of the embedding Euclidean space.
 */
template <typename T, int N>
class SphericalPoint {
public:
    /// Angular parameters (N-1 angles: N-2 polar angles and one azimuth).
    Vector<T, N - 1> m_spherical;
    /// Radial distance from the origin.
    T                m_radius;

    /// Construct from a Cartesian point.
    static constexpr auto from_cartesian(const Point<T, N>& cartesian) {
        return SphericalPoint(cartesian);
    }

    /// Construct from angles and radius.
    constexpr SphericalPoint(const Vector<T, N - 1>& spherical, T radius) : m_spherical(spherical), m_radius(radius) {}
    /// Construct from Cartesian coordinates, converting to spherical form.
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

    /// Convert back to Cartesian coordinates.
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

    /// Get the radius.
    constexpr T radius() const { return m_radius; }
    /// Get angle i (0-based).
    constexpr T angle(int i) const { return m_spherical[i]; }
    /// Get the full angle vector.
    constexpr const Vector<T, N - 1>& angles() const { return m_spherical; }

    /// Get the azimuth angle (last angle).
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

/// Cosine of the polar angle for a 3D direction (z component).
template<typename T>
inline constexpr auto cos_theta(const Vector<T, 3>& v) {
    return v.z();
} 

/// Cosine squared of the polar angle.
template <typename T>
inline constexpr auto cos2_theta(const Vector<T, 3>& v) {
    return v.z() * v.z();
}

/// Sine squared of the polar angle.
template<typename T>
inline constexpr auto sin2_theta(const Vector<T, 3>& v) {
    return std::max(T(0), T(1) - cos2_theta(v));
}

/// Sine of the polar angle.
template<typename T>
inline constexpr auto sin_theta(const Vector<T, 3>& v) {
    return std::sqrt(sin2_theta(v));
}

/// Tangent of the polar angle.
template <typename T>
inline constexpr auto tan_theta(const Vector<T, 3>& v) {
    return sin_theta(v) / cos_theta(v);
}

/// Tangent squared of the polar angle.
template <typename T>
inline constexpr auto tan2_theta(const Vector<T, 3>& v) {
    return sin2_theta(v) / cos2_theta(v);
}

/// Azimuth angle of a 3D direction in [0, 2*pi).
template<typename T>
inline constexpr auto phi(const Vector<T, 3>& v) {
    auto p = std::atan2(v.y(), v.x());
    return wrap_angle_2pi(p);
}

/// Sine of the azimuth angle.
template<typename T>
inline constexpr auto sin_phi(const Vector<T, 3>& v) {
    auto s_th = sin_theta(v);
    return is_zero(s_th) ? T(0) : std::clamp(v.y() / s_th, T(-1), T(1));
}

/// Cosine of the azimuth angle.
template<typename T>
inline constexpr auto cos_phi(const Vector<T, 3>& v) {
    auto s_th = sin_theta(v);
    return is_zero(s_th) ? T(1) : std::clamp(v.x() / s_th, T(-1), T(1));
}

/// Cosine of the azimuth difference between two directions.
template<typename T>
inline constexpr auto cos_delta_phi(const Vector<T, 3>& a, const Vector<T, 3>& b) {
    auto axy = a.x() * a.x() + a.y() * a.y();
    auto bxy = b.x() * b.x() + b.y() * b.y();
    return (is_zero(axy) || is_zero(bxy)) ? T(1) : std::clamp(
        (a.x() * b.x() + a.y() * b.y()) / std::sqrt(axy * bxy), 
        T(-1), T(1)
    );
}

/**
 * @brief Warp a 2D point into the unit square with equal-area tiling.
 *
 * Points outside [0,1]^2 are mirrored back into the square so that the
 * mapping preserves area when combined with @c equal_area_square_to_sphere.
 */
template<typename T>
inline constexpr auto warp_equal_area_square(math::Point<T, 2> uv) {
    if (uv.x() < 0) {
        uv.x() = -uv.x();     // mirror across u = 0
        uv.y() = 1 - uv.y();  // mirror across v = 0.5
    } else if (uv.x() > 1) {
        uv.x() = 2 - uv.x();  // mirror across u = 1
        uv.y() = 1 - uv.y();  // mirror across v = 0.5
    }
    if (uv.y() < 0) {
        uv.x() = 1 - uv.x();  // mirror across u = 0.5
        uv.y() = -uv.y();     // mirror across v = 0;
    } else if (uv.y() > 1) {
        uv.x() = 1 - uv.x();  // mirror across u = 0.5
        uv.y() = 2 - uv.y();  // mirror across v = 1
    }
    return uv;
}

/**
 * @brief Map a point in [0,1]^2 to the unit sphere with equal-area property.
 *
 * The mapping is designed so that each region of the unit square has the
 * same area on the unit sphere, which is useful for stratified sampling
 * of directions.
 */
template<typename T>
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

/**
 * @brief Signed area of a spherical triangle on the unit sphere.
 *
 * Uses a robust formula based on the vertices' dot and cross products.
 */
template<typename T>
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

/**
 * @brief Area of a spherical polygon on the unit sphere.
 *
 * The polygon is assumed to be defined by vertices on the unit sphere
 * in counter-clockwise order. It is triangulated fan-wise from vertex 0.
 */
template<typename T>
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

/// 2D spherical point (radius and azimuth) using the default scalar type.
using Sphere2 = SphericalPoint<Float, 2>;
/// 3D spherical point (radius, polar and azimuth) using the default scalar type.
using Sphere3 = SphericalPoint<Float, 3>;

}  // namespace pbpt::geometry
