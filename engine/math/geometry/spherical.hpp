#pragma once

#include "../global/function.hpp"
#include "../global/type_alias.hpp"
#include "point.hpp"
#include "vector.hpp"

namespace pbpt::math {

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

    constexpr SphericalPoint(const Vector<T, N - 1>& spherical, T radius) : m_spherical(spherical), m_radius(radius) {}

    /**
     * @brief 从笛卡尔坐标构造球坐标
     */
    constexpr SphericalPoint(const Point<T, N>& cartesian) {
        m_radius = cartesian.to_vector().length();

        if (m_radius == 0) {
            m_spherical = Vector<T, N - 1>::zeros();
            return;
        }

        if constexpr (N == 2) {
            // 2D情况: φ = atan2(y, x) (方位角)
            m_spherical[0] = atan2(cartesian.y(), cartesian.x());
            if (m_spherical[0] < 0) {
                m_spherical[0] += T(2 * M_PI);
            }
        } else if constexpr (N == 3) {
            // 3D情况: θ = acos(z/r), φ = atan2(y, x)
            m_spherical[0] = acos(cartesian.z() / m_radius);       // 极角
            m_spherical[1] = atan2(cartesian.y(), cartesian.x());  // 方位角
            if (m_spherical[1] < 0) {
                m_spherical[1] += T(2 * M_PI);
            }
        } else {
            // N维情况的递归计算
            cartesian_to_spherical_nd(cartesian);
        }
    }

    /**
     * @brief 转换为笛卡尔坐标
     */
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

    /**
     * @brief 获取半径
     */
    constexpr T radius() const { return m_radius; }

    /**
     * @brief 获取第i个角度
     */
    constexpr T angle(int i) const { return m_spherical[i]; }

    /**
     * @brief 获取所有角度
     */
    constexpr const Vector<T, N - 1>& angles() const { return m_spherical; }

    /**
     * @brief 获取方位角（角度向量的最后一个元素）
     */
    constexpr T azimuth() const {
        static_assert(N >= 2, "Azimuth angle only exists for N >= 2");
        return m_spherical[N - 2];
    }

private:
    /**
     * @brief N维笛卡尔到球坐标的转换
     */
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

            if (partial_radius == 0) {
                m_spherical[i] = 0;
            } else {
                m_spherical[i] = acos(cartesian[i + 2] / partial_radius);
            }
        }

        // 最后一个角度 (方位角)
        m_spherical[N - 2] = atan2(cartesian[1], cartesian[0]);
        if (m_spherical[N - 2] < 0) {
            m_spherical[N - 2] += T(2 * M_PI);
        }
    }

    /**
     * @brief N维球坐标到笛卡尔的转换
     */
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

using Sphere2 = SphericalPoint<Float, 2>;
using Sphere3 = SphericalPoint<Float, 3>;

}  // namespace pbpt::math