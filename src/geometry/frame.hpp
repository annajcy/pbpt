#pragma once

#include <concepts>

#include "math/vector.hpp"

#include "transform.hpp"

using namespace pbpt::math;

namespace pbpt::geometry {

/// @brief 正交局部坐标系 Frame，用于 BSDF、Shading、Scattering 等局部空间操作
/// 保证 [t, b, n] 是单位正交基，默认构造为右手系（t × b = n）
template <typename T>
class Frame {
private:
    Vector<T, 3> m_t, m_b, m_n;  
    Transform<T> m_local_to_world;
    Transform<T> m_world_to_local;

public:
    constexpr Frame() = default;

    /// @brief 以法线 n 构造局部坐标系（右手系）
    /// @param n 局部空间中的 n 轴，通常是法线方向
    /// @param flip_handedness 若为 true，则构造左手系（t × b = -n）
    constexpr explicit Frame(const Vector<T, 3>& n, bool flip_to_left_handedness = false) {
        m_n = n.normalized();
        const Vector<T, 3> up = (std::abs(m_n.x()) > T(0.99)) ? Vector<T, 3>(T(0), T(1), T(0))
                                                             : Vector<T, 3>(T(1), T(0), T(0));
        m_t = cross(up, m_n).normalized();
        m_b = cross(m_n, m_t);
        if (flip_to_left_handedness) m_b = -m_b;
        update_tranform();
    }

    /// @brief 基于已知的 tangent 和 normal 构造局部坐标系
    /// 默认会自动修正手性使其为右手系（若需左手系，可设 flip_handedness = true）
    constexpr Frame(const Vector<T, 3>& tangent, const Vector<T, 3>& normal, bool flip_to_left_handedness = false) {
        m_t = tangent.normalized();
        m_n = normal.normalized();
        m_b = cross(m_n, m_t);
        if (flip_to_left_handedness) m_b = -m_b;
        update_tranform();
    }

    constexpr const Vector<T, 3>& t() const { return m_t; }
    constexpr const Vector<T, 3>& b() const { return m_b; }
    constexpr const Vector<T, 3>& n() const { return m_n; }
    constexpr const Transform<T>& local_to_world() const { return m_local_to_world; }
    constexpr const Transform<T>& world_to_local() const { return m_world_to_local; }

private:

    constexpr void update_tranform() {
        m_local_to_world = Transform<T>::from_mat3x3(Matrix<T, 3, 3>(m_t, m_b, m_n));
        m_world_to_local = m_local_to_world.inversed();
    }
};

using Fra = Frame<Float>;

}  // namespace pbpt::geometry
