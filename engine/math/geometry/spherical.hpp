#pragma once

#include "../global/type_alias.hpp"
#include "../global/function.hpp"

#include "math/geometry/vector.hpp"
#include "point.hpp"

namespace pbpt::math {

/**
 * @class SphericalPoint
 * @brief N维球坐标点类
 * @details 球坐标系表示：
 * - 2D: (r, θ) - 半径和角度
 * - 3D: (r, θ, φ) - 半径、方位角、极角
 * - ND: (r, θ₁, θ₂, ..., θₙ₋₁) - 半径和N-1个角度
 * 
 * 角度约定：
 * - θ₁ ∈ [0, 2π) (方位角)
 * - θᵢ ∈ [0, π] for i > 1 (极角)
 */
template<typename T, int N>
class SphericalPoint {
public:
    Vector<T, N - 1> m_spherical;  // N-1个角度
    T m_radius;                    // 半径

    constexpr SphericalPoint(const Vector<T, N - 1>& spherical, T radius) 
        : m_spherical(spherical), m_radius(radius) {}
    
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
            // 2D情况: θ = atan2(y, x)
            m_spherical[0] = atan2(cartesian.y(), cartesian.x());
            if (m_spherical[0] < 0) {
                m_spherical[0] += T(2 * M_PI);
            }
        } else if constexpr (N == 3) {
            // 3D情况: θ = atan2(y, x), φ = acos(z/r)
            m_spherical[0] = atan2(cartesian.y(), cartesian.x());
            if (m_spherical[0] < 0) {
                m_spherical[0] += T(2 * M_PI);
            }
            m_spherical[1] = acos(cartesian.z() / m_radius);
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
            // 2D: x = r*cos(θ), y = r*sin(θ)
            result[0] = m_radius * cos(m_spherical[0]);
            result[1] = m_radius * sin(m_spherical[0]);
        } else if constexpr (N == 3) {
            // 3D: x = r*sin(φ)*cos(θ), y = r*sin(φ)*sin(θ), z = r*cos(φ)
            T sin_phi = sin(m_spherical[1]);
            result[0] = m_radius * sin_phi * cos(m_spherical[0]);
            result[1] = m_radius * sin_phi * sin(m_spherical[0]);
            result[2] = m_radius * cos(m_spherical[1]);
        } else {
            // N维情况
            spherical_to_cartesian_nd(result);
        }
        
        return Point<T, N>(result);
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

private:
    /**
     * @brief N维笛卡尔到球坐标的转换
     */
    constexpr void cartesian_to_spherical_nd(const Point<T, N>& cartesian) {
        // N维球坐标的一般公式
        // θ₁ = atan2(x₂, x₁)
        // θᵢ = acos(xᵢ₊₁ / sqrt(x₁² + x₂² + ... + xᵢ₊₁²)) for i = 2, ..., N-1
        
        // 第一个角度 (方位角)
        m_spherical[0] = atan2(cartesian[1], cartesian[0]);
        if (m_spherical[0] < 0) {
            m_spherical[0] += T(2 * M_PI);
        }
        
        // 其余角度 (极角)
        for (int i = 1; i < N - 1; ++i) {
            T sum_squares = 0;
            for (int j = 0; j <= i; ++j) {
                sum_squares += cartesian[j] * cartesian[j];
            }
            sum_squares += cartesian[i + 1] * cartesian[i + 1];
            T partial_radius = sqrt(sum_squares);
            
            if (partial_radius == 0) {
                m_spherical[i] = 0;
            } else {
                m_spherical[i] = acos(cartesian[i + 1] / partial_radius);
            }
        }
    }
    
    /**
     * @brief N维球坐标到笛卡尔的转换
     */
    constexpr void spherical_to_cartesian_nd(Vector<T, N>& result) const {
        // N维球坐标到笛卡尔的一般公式
        // x₁ = r * cos(θ₁) * ∏ᵢ₌₂ᴺ⁻¹ sin(θᵢ)
        // x₂ = r * sin(θ₁) * ∏ᵢ₌₂ᴺ⁻¹ sin(θᵢ)
        // xₖ = r * cos(θₖ₋₁) * ∏ᵢ₌ₖᴺ⁻¹ sin(θᵢ) for k = 3, ..., N-1
        // xₙ = r * cos(θₙ₋₁)
        
        // 计算所有sin值的累积乘积
        Vector<T, N - 1> sin_products;
        sin_products[N - 2] = T(1); // 最后一个元素
        
        for (int i = N - 3; i >= 0; --i) {
            sin_products[i] = sin_products[i + 1] * sin(m_spherical[i + 1]);
        }
        
        // 计算笛卡尔坐标
        result[0] = m_radius * cos(m_spherical[0]) * sin_products[0];
        result[1] = m_radius * sin(m_spherical[0]) * sin_products[0];
        
        for (int i = 2; i < N - 1; ++i) {
            result[i] = m_radius * cos(m_spherical[i - 1]) * sin_products[i - 1];
        }
        
        result[N - 1] = m_radius * cos(m_spherical[N - 2]);
    }
};

using Sphere2 = SphericalPoint<Float, 2>;
using Sphere3 = SphericalPoint<Float, 3>;

} // namespace pbpt::math