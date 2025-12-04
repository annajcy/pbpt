/**
 * @file
 * @brief Local orthonormal frames used for shading and scattering.
 */
#pragma once

#include <concepts>

#include "math/vector.hpp"

#include "transform.hpp"

using namespace pbpt::math;

namespace pbpt::geometry {

/**
 * @brief Orthonormal local coordinate frame.
 *
 * Represents a right-handed or left-handed basis [t, b, n] where t is
 * tangent, b is bitangent and n is the normal. The frame also stores
 * transforms between local and world coordinates, which are useful for
 * converting directions in BRDF/BSDF evaluations.
 *
 * @tparam T Scalar type.
 */
template <typename T>
class Frame {
private:
    Vector<T, 3> m_t, m_b, m_n;  
    Transform<T> m_local_to_world;
    Transform<T> m_world_to_local;

public:
    constexpr Frame() = default;

    /**
     * @brief Build a local frame from a normal.
     *
     * By default a right-handed frame is constructed such that
     * t × b = n. Passing @p flip_to_left_handedness = true yields a
     * left-handed frame instead (t × b = -n).
     *
     * @param n Local-space normal direction.
     * @param flip_to_left_handedness Whether to construct a left-handed frame.
     */
    constexpr explicit Frame(const Vector<T, 3>& n, bool flip_to_left_handedness = false) {
        m_n = n.normalized();
        const Vector<T, 3> up = (std::abs(m_n.x()) > T(0.99)) ? Vector<T, 3>(T(0), T(1), T(0))
                                                             : Vector<T, 3>(T(1), T(0), T(0));
        m_t = cross(up, m_n).normalized();
        m_b = cross(m_n, m_t);
        if (flip_to_left_handedness) m_b = -m_b;
        update_tranform();
    }

    /**
     * @brief Build a frame from a given tangent and normal.
     *
     * The bitangent is computed as n × t, and handedness can be flipped
     * by setting @p flip_to_left_handedness to true.
     */
    constexpr Frame(const Vector<T, 3>& tangent, const Vector<T, 3>& normal, bool flip_to_left_handedness = false) {
        m_t = tangent.normalized();
        m_n = normal.normalized();
        m_b = cross(m_n, m_t);
        if (flip_to_left_handedness) m_b = -m_b;
        update_tranform();
    }

    /// Tangent vector.
    constexpr const Vector<T, 3>& t() const { return m_t; }
    /// Bitangent vector.
    constexpr const Vector<T, 3>& b() const { return m_b; }
    /// Normal vector.
    constexpr const Vector<T, 3>& n() const { return m_n; }
    /// Transform from local frame coordinates to world space.
    constexpr const Transform<T>& local_to_world() const { return m_local_to_world; }
    /// Transform from world space to local frame coordinates.
    constexpr const Transform<T>& world_to_local() const { return m_world_to_local; }

private:

    constexpr void update_tranform() {
        m_local_to_world = Transform<T>::from_mat3x3(Matrix<T, 3, 3>(m_t, m_b, m_n));
        m_world_to_local = m_local_to_world.inversed();
    }
};

/// Frame alias using the library's default floating-point type.
using Fra = Frame<Float>;

}  // namespace pbpt::geometry
