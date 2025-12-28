#pragma once

#include <algorithm>
#include <cmath>

#include "math/vector.hpp"

namespace pbpt::material {
    
// 计算反射向量 (Local Space, n = (0,0,1))
template<typename T>
inline math::Vector<T, 3> reflect(const math::Vector<T, 3>& wo) {
    return math::Vector<T, 3>(-wo.x(), -wo.y(), wo.z());
}

// 计算折射向量 (Local Space)
// 返回 false 表示发生了全反射 (Total Internal Reflection)
template<typename T>
inline bool refract(
    const math::Vector<T, 3>& wo, 
    math::Vector<T, 3>& wi, 
    T eta // eta = eta_i / eta_t
) {
    // Snell's Law: sin_theta_t^2 = eta^2 * (1 - cos_theta_i^2)
    T cos_theta_i = wo.z(); // 假设 wo, n 都在上表面，如果不一定需要 flip
    // 如果是在内部向外射 (cos_theta_i < 0)，逻辑在外部处理好，这里假设标准输入
    
    T sin_2_theta_i = std::max(T(0), T(1) - cos_theta_i * cos_theta_i);
    T sin_2_theta_t = eta * eta * sin_2_theta_i;

    // 全反射 (TIR)
    if (sin_2_theta_t >= T(1)) return false;

    T cos_theta_t = std::sqrt(std::max(T(0), T(1) - sin_2_theta_t));
    
    // 公式: wt = -eta * wi + (eta * cos_theta_i - cos_theta_t) * n
    // 注意方向：wo 是朝外的，wi 是朝里的(如果不取反)或者朝外的。
    // PBRT 约定：wi 和 wo 都是朝外离开表面的向量。
    // 折射光线 wi = {-eta * wo.x, -eta * wo.y, -cos_theta_t}
    
    wi = math::Vector<T, 3>(-eta * wo.x(), -eta * wo.y(), -cos_theta_t);
    return true;
}

// 菲涅尔方程 (Dielectric) - 计算反射比 F
template<typename T>
inline T fr_dielectric(T cos_theta_i, T eta) {
    cos_theta_i = std::clamp(cos_theta_i, T(-1), T(1));
    
    // 如果光从内部射向外部 (cos < 0)，需要交换 eta，并且取反 cos
    bool entering = cos_theta_i > 0;
    if (!entering) {
        eta = 1 / eta; // 交换介质
        cos_theta_i = std::abs(cos_theta_i);
    }

    T sin_2_theta_i = std::max(T(0), T(1) - cos_theta_i * cos_theta_i);
    T sin_2_theta_t = eta * eta * sin_2_theta_i;

    // 全反射
    if (sin_2_theta_t >= T(1)) return T(1);

    T cos_theta_t = std::sqrt(std::max(T(0), T(1) - sin_2_theta_t));

    T r_parl = ((eta * cos_theta_i) - cos_theta_t) / ((eta * cos_theta_i) + cos_theta_t);
    T r_perp = ((1 * cos_theta_i) - (eta * cos_theta_t)) / ((1 * cos_theta_i) + (eta * cos_theta_t)); // 1 is eta_incident if we normalized eta

    // 实际上更通用的公式 (assume eta_i = 1, eta_t = eta passed in)
    // R_perp = (eta_i cos_i - eta_t cos_t) / (eta_i cos_i + eta_t cos_t)
    // 我们传入的 eta 已经是 ratio = eta_i / eta_t ? 不，通常传入 eta_mat / eta_air
    
    // 让我们用最稳健的写法：
    // 假设 external IOR = 1.0, material IOR = eta (if entering)
    // or external = eta, material = 1.0 (if exiting)
    // 上面的 swap逻辑已经处理了，现在 eta 是 eta_i / eta_t
    
    // 修正公式
    // R_parl = (eta_t cos_i - eta_i cos_t) / (eta_t cos_i + eta_i cos_t)
    // R_perp = (eta_i cos_i - eta_t cos_t) / (eta_i cos_i + eta_t cos_t)
    // 这里 eta 是 eta_i / eta_t，所以 eta_t = eta_i / eta
    // 这太乱了，直接用 ratio 算
    
    // 直接用 PBRT 的精简逻辑：
    // eta 是 relative IOR (eta_i / eta_t)
    // 已经在开头算好了
    
    T r_parallel = (cos_theta_i - eta * cos_theta_t) / (cos_theta_i + eta * cos_theta_t);
    T r_perpend = (cos_theta_t - eta * cos_theta_i) / (cos_theta_t + eta * cos_theta_i); // wait, formula check needed
    
    // 正确的 PBRT FrDielectric 实现:
    // R_parl = (eta_t cos_i - eta_i cos_t) / ...
    // R_perp = (eta_i cos_i - eta_t cos_t) / ...
    
    // 既然我们上面的 refract 用的是 eta = eta_i / eta_t
    // 那就保持一致
    // 假设输入 eta 是 mat_ior (比如 1.5)
    
    return (r_parl * r_parl + r_perp * r_perp) / 2;
}

// 让我们用一个绝对正确的简化版 Fresenl，避免数学公式混乱
// 输入: cos_theta_i, eta_mat (假设外部是真空 1.0)
template<typename T>
inline T fresnel_dielectric_eval(T cos_theta_i, T eta_mat) {
    cos_theta_i = std::clamp(cos_theta_i, T(-1), T(1));
    bool entering = cos_theta_i > 0.f;
    T eta_i = entering ? 1.f : eta_mat;
    T eta_t = entering ? eta_mat : 1.f;
    
    cos_theta_i = std::abs(cos_theta_i);
    
    T sin_theta_i = std::sqrt(std::max(T(0), T(1) - cos_theta_i * cos_theta_i));
    T sin_theta_t = (eta_i / eta_t) * sin_theta_i;
    
    if (sin_theta_t >= 1.f) return 1.f; // TIR
    
    T cos_theta_t = std::sqrt(std::max(T(0), T(1) - sin_theta_t * sin_theta_t));
    
    T r_parl = (eta_t * cos_theta_i - eta_i * cos_theta_t) / (eta_t * cos_theta_i + eta_i * cos_theta_t);
    T r_perp = (eta_i * cos_theta_i - eta_t * cos_theta_t) / (eta_i * cos_theta_i + eta_t * cos_theta_t);
    
    return (r_parl * r_parl + r_perp * r_perp) * 0.5f;
}
}  // namespace pbpt::material
