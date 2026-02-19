#pragma once

#include <cmath>
#include <concepts>
#include "pbpt/math/vector.hpp"
#include "transform.hpp"

namespace pbpt::geometry {

/**
 * @brief Orthonormal local coordinate frame.
 * * Efficiently stores a shading frame (tangent, bitangent, normal).
 * Designed for high-performance BSDF evaluation.
 */
template <typename T>
class Frame {
private:
    math::Vector<T, 3> m_t;  // x
    math::Vector<T, 3> m_b;  // y
    math::Vector<T, 3> m_n;  // z

public:
    constexpr Frame() = default;

    /**
     * @brief Build a local frame from a normal.
     * Uses the Duff's method (or similar) to robustly generate an orthonormal basis.
     */
    constexpr Frame(const math::Vector<T, 3>& n) {
        m_n = n.normalized();

        // Robust method to generate basis (Frisvad / Duff)
        // Checks if n.z is close to -1 or 1 to avoid singularity
        if (m_n.z() < T(-0.9999999)) {
            m_t = math::Vector<T, 3>(T(0), T(-1), T(0));
            m_b = math::Vector<T, 3>(T(-1), T(0), T(0));
        } else {
            const T a = T(1) / (T(1) + m_n.z());
            const T b = -m_n.x() * m_n.y() * a;
            m_t = math::Vector<T, 3>(T(1) - m_n.x() * m_n.x() * a, b, -m_n.x());
            m_b = math::Vector<T, 3>(b, T(1) - m_n.y() * m_n.y() * a, -m_n.y());
        }
        // 注：如果你原来的 up-vector 方法跑得通，用原来的也可以，这个方法更鲁棒且无分支(大部分情况)
    }

    constexpr Frame(const math::Vector<T, 3>& tangent, const math::Vector<T, 3>& normal) {
        m_n = normal.normalized();
        m_t = tangent.normalized();
        // Gram-Schmidt orthogonalization to ensure t is perpendicular to n
        m_t = (m_t - m_n * m_n.dot(m_t)).normalized();  // Optional but safer
        m_b = math::cross(m_n, m_t);
    }

    // --- Data Access ---
    constexpr const math::Vector<T, 3>& t() const { return m_t; }
    constexpr const math::Vector<T, 3>& b() const { return m_b; }
    constexpr const math::Vector<T, 3>& n() const { return m_n; }

    // --- Fast Vector Transforms (Use these in BSDFs!) ---

    /// World(Render) -> Local
    /// Projects v onto the basis vectors.
    constexpr math::Vector<T, 3> to_local(const math::Vector<T, 3>& v) const {
        return math::Vector<T, 3>{v.dot(m_t), v.dot(m_b), v.dot(m_n)};
    }

    /// Local -> World(Render)
    /// Linear combination of basis vectors.
    constexpr math::Vector<T, 3> to_render(const math::Vector<T, 3>& v) const {
        return m_t * v.x() + m_b * v.y() + m_n * v.z();
    }

    // --- BSDF Helpers (Local Space) ---
    // These operate on vectors ALREADY in local space!

    static constexpr T cos_theta(const math::Vector<T, 3>& w) { return w.z(); }
    static constexpr T cos2_theta(const math::Vector<T, 3>& w) { return w.z() * w.z(); }
    static constexpr T abs_cos_theta(const math::Vector<T, 3>& w) { return std::abs(w.z()); }

    static constexpr T sin2_theta(const math::Vector<T, 3>& w) { return std::max(T(0), T(1) - cos2_theta(w)); }

    static T sin_theta(const math::Vector<T, 3>& w) {  // std::sqrt usually not constexpr
        return std::sqrt(sin2_theta(w));
    }

    static T tan_theta(const math::Vector<T, 3>& w) { return sin_theta(w) / cos_theta(w); }

    // --- Heavy Transforms (Avoid in inner loops) ---
    // Keep these if you need to export the frame to a matrix for other systems

    constexpr Transform<T> local_to_render_transform() const {
        return Transform<T>::from_mat3x3(Matrix<T, 3, 3>::from_cols(m_t, m_b, m_n));
    }

    constexpr Transform<T> render_to_local_transform() const {
        // For orthonormal basis, Inverse == Transpose
        return local_to_render_transform().inversed();
    }
};

}  // namespace pbpt::geometry