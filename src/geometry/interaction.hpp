/**
 * @file
 * @brief Interaction records for points where rays hit geometry.
 */
#pragma once

#include "math/point.hpp"
#include "math/vector.hpp"
#include "math/normal.hpp"
#include "geometry/ray.hpp"

namespace pbpt::geometry {

/**
 * @brief Offset a ray origin to avoid self-intersection.
 *
 * The offset is computed from a lower/upper error bound on the hit
 * position and the surface normal. The resulting origin is slightly
 * shifted along the normal so that new rays do not immediately intersect
 * the surface from which they originate.
 *
 * @tparam T Scalar type.
 * @param p_lower Lower bound of the hit position.
 * @param p_upper Upper bound of the hit position.
 * @param dir     Direction of the new ray.
 * @param n       Geometric surface normal at the hit.
 * @return Offset origin for the new ray.
 */
template<typename T>
inline math::Point<T, 3> offset_ray_origin(
    const math::Point<T, 3>& p_lower,
    const math::Point<T, 3>& p_upper,
    const Vector<T, 3>& dir, 
    const Normal<T, 3>& n
) {
    // Shift the ray start slightly along the geometric normal so that new rays
    // do not immediately self‐intersect the triangle they originate from.
    // The offset magnitude combines (1) how “thick” the triangle is when
    // projected onto the normal and (2) a gamma-corrected term that compensates
    // floating-point error proportional to the triangle midpoint coordinates.

    auto d = abs(n.x()) * (p_upper.x() - p_lower.x()) + 
            abs(n.y()) * (p_upper.y() - p_lower.y()) + 
            abs(n.z()) * (p_upper.z() - p_lower.z());

    auto p_mid = p_lower.mid(p_upper);

    constexpr auto gamma7 = gamma<T>(7);

    d += 2 * gamma7 * (
            abs(p_mid.x() * abs(n.x())) + 
            abs(p_mid.y() * abs(n.y())) + 
            abs(p_mid.z() * abs(n.z()))
        );

    auto nv = n.to_vector();
    Vector<T, 3> offset = d * nv;
    if (is_less(nv.dot(dir), 0)) {
        offset = -offset;
    }

    return p_mid + offset;
}

/**
 * @brief Base interaction record storing a point interval and outgoing direction.
 *
 * An interaction represents where a ray hits a surface or medium. Instead
 * of a single point, it stores a small bounding segment [p_lower, p_upper]
 * that accounts for numerical uncertainty, plus an outgoing direction
 * relative to the incoming ray.
 *
 * Concrete interaction types derive from this template using CRTP and
 * provide implementations for ray spawning.
 *
 * @tparam T       Scalar type.
 * @tparam Derived Concrete interaction type.
 */
template<typename T, typename Derived>
class Interaction {
protected:
    /// Lower bound of the hit position accounting for numerical error.
    math::Point<T, 3> m_p_lower;
    /// Upper bound of the hit position accounting for numerical error.
    math::Point<T, 3> m_p_upper;
    /// Outgoing direction with respect to the local frame of the incoming ray.
    math::Vector<T, 3> m_dir;

public:
    /**
     * @brief Construct from a lower/upper bound and outgoing direction.
     */
    Interaction(
        const math::Point<T, 3>& p_lower, 
        const math::Point<T, 3>& p_upper, 
        const math::Vector<T, 3>& dir
    ) : m_p_lower(p_lower), m_p_upper(p_upper), m_dir(dir) {}

    /**
     * @brief Construct from a central point, outgoing direction and error margin.
     *
     * The lower/upper bounds are obtained by subtracting/adding
     * @p error_margin from the central point.
     */
    Interaction(
        const math::Point<T, 3>& p, 
        const math::Vector<T, 3>& dir, 
        const math::Vector<T, 3>& error_margin = math::Vector<T, 3>{0, 0, 0}
    ) : m_p_lower(p - error_margin), m_p_upper(p + error_margin), m_dir(dir) {}

    /// Outgoing direction (const).
    const math::Vector<T, 3>& dir() const { return m_dir; }
    /// Lower bound of the hit position (const).
    const math::Point<T, 3>& p_lower() const { return m_p_lower; }
    /// Upper bound of the hit position (const).
    const math::Point<T, 3>& p_upper() const { return m_p_upper; }

    /// Lower bound of the hit position (mutable).
    math::Point<T, 3>& p_lower() { return m_p_lower; }
    /// Upper bound of the hit position (mutable).
    math::Point<T, 3>& p_upper() { return m_p_upper; }
    /// Outgoing direction (mutable).
    math::Vector<T, 3>& dir() { return m_dir; }

    /// Midpoint of [p_lower, p_upper].
    math::Point<T, 3> point() const { return m_p_lower.mid(m_p_upper); }

    /**
     * @brief Spawn a ray in the given direction from this interaction.
     *
     * The concrete interaction type decides how to offset the origin
     * to avoid self-intersection.
     */
    geometry::Ray<T, 3> spawn_ray(const math::Vector<T, 3>& direction) const {
        return as_derived().spawn_ray_impl(direction);
    }

    /**
     * @brief Spawn a ray from this interaction toward a target point.
     *
     * The returned ray typically has its t_max set so that it terminates
     * just before reaching @p point.
     */
    geometry::Ray<T, 3> spawn_ray_to(const math::Point<T, 3>& point) const {
        return as_derived().spawn_ray_to_impl(point);
    }

    /// Access the derived interaction (const).
    const Derived& as_derived() const {
        return static_cast<const Derived&>(*this);
    }

    /// Access the derived interaction (mutable).
    Derived& as_derived() {
        return static_cast<Derived&>(*this);
    }
};

/**
 * @brief Interaction with a surface, including geometry and texture coordinates.
 *
 * Extends @c Interaction with:
 * - geometric normal,
 * - texture coordinates (u, v),
 * - partial derivatives of the position with respect to u and v,
 * - partial derivatives of the normal with respect to u and v.
 *
 * This information is used for shading, texture mapping, and computing
 * ray offsets that account for the local surface geometry.
 *
 * @tparam T Scalar type.
 */
template<typename T>
class SurfaceInteraction : public Interaction<T, SurfaceInteraction<T>> {
    friend class Interaction<T, SurfaceInteraction<T>>;

    using base = Interaction<T, SurfaceInteraction<T>>;
    using base::m_p_lower;
    using base::m_p_upper;
    using base::m_dir;

protected:
    /// Geometric surface normal at the interaction point.
    Normal<T, 3> m_n;
    /// Texture coordinates at the interaction point.
    Point<T, 2> m_uv;
    /// Partial derivative of position with respect to u.
    Vector<T, 3> m_dpdu;
    /// Partial derivative of position with respect to v.
    Vector<T, 3> m_dpdv;
    /// Partial derivative of the normal with respect to u.
    Normal<T, 3> m_dndu;
    /// Partial derivative of the normal with respect to v.
    Normal<T, 3> m_dndv;

public:
    /**
     * @brief Construct from full error bounds and surface differential data.
     *
     * @param p_lower Lower bound of the hit position.
     * @param p_upper Upper bound of the hit position.
     * @param wo      Outgoing direction with respect to the incoming ray.
     * @param n       Geometric normal at the hit.
     * @param uv      Texture coordinates.
     * @param dpdu    Partial derivative of position with respect to u.
     * @param dpdv    Partial derivative of position with respect to v.
     * @param dndu    Partial derivative of normal with respect to u.
     * @param dndv    Partial derivative of normal with respect to v.
     */
    SurfaceInteraction(
        const math::Point<T, 3>& p_lower,
        const math::Point<T, 3>& p_upper,
        const math::Vector<T, 3>& wo,
        const Normal<T, 3>& n,
        const Point<T, 2>& uv,
        const Vector<T, 3>& dpdu,
        const Vector<T, 3>& dpdv,
        const Normal<T, 3>& dndu,
        const Normal<T, 3>& dndv
    ) : Interaction<T, SurfaceInteraction<T>>(p_lower, p_upper, wo),
          m_n(n),
          m_uv(uv),
          m_dpdu(dpdu),
          m_dpdv(dpdv),
          m_dndu(dndu),
          m_dndv(dndv) {}

    /**
     * @brief Construct from a central point, direction and surface data.
     *
     * The error margin is expanded symmetrically around @p p.
     *
     * @param p            Central hit point.
     * @param wo           Outgoing direction with respect to the incoming ray.
     * @param n            Geometric normal at the hit.
     * @param uv           Texture coordinates.
     * @param dpdu         Partial derivative of position with respect to u.
     * @param dpdv         Partial derivative of position with respect to v.
     * @param dndu         Partial derivative of normal with respect to u.
     * @param dndv         Partial derivative of normal with respect to v.
     * @param error_margin Symmetric error margin around p.
     */
    SurfaceInteraction(
        math::Point<T, 3> p,
        const math::Vector<T, 3>& wo,
        const Normal<T, 3>& n,
        const Point<T, 2>& uv,
        const Vector<T, 3>& dpdu,
        const Vector<T, 3>& dpdv,
        const Normal<T, 3>& dndu,
        const Normal<T, 3>& dndv,
        const math::Vector<T, 3>& error_margin = math::Vector<T, 3>{0, 0, 0}
    ) : Interaction<T, SurfaceInteraction<T>>(p, wo, error_margin),
          m_n(n),
          m_uv(uv),
          m_dpdu(dpdu),
          m_dpdv(dpdv),
          m_dndu(dndu),
          m_dndv(dndv) {}

    /// Get the geometric surface normal (const).
    const Normal<T, 3>& n() const { return m_n; }
    /// Get the texture coordinates (const).
    const Point<T, 2>& uv() const { return m_uv; }
    /// Get the partial derivative of position with respect to u (const).
    const Vector<T, 3>& dpdu() const { return m_dpdu; }
    /// Get the partial derivative of position with respect to v (const).
    const Vector<T, 3>& dpdv() const { return m_dpdv; }
    /// Get the partial derivative of the normal with respect to u (const).
    const Normal<T, 3>& dndu() const { return m_dndu; }
    /// Get the partial derivative of the normal with respect to v (const).
    const Normal<T, 3>& dndv() const { return m_dndv; }

    /// Get the geometric surface normal (mutable).
    Normal<T, 3>& n() { return m_n; }
    /// Get the texture coordinates (mutable).
    Point<T, 2>& uv() { return m_uv; }
    /// Get the partial derivative of position with respect to u (mutable).
    Vector<T, 3>& dpdu() { return m_dpdu; }
    /// Get the partial derivative of position with respect to v (mutable).
    Vector<T, 3>& dpdv() { return m_dpdv; }
    /// Get the partial derivative of the normal with respect to u (mutable).
    Normal<T, 3>& dndu() { return m_dndu; }
    /// Get the partial derivative of the normal with respect to v (mutable).
    Normal<T, 3>& dndv() { return m_dndv; }

    /// Spawn a ray in direction @p wi, offset away from the surface.
    Ray<T, 3> spawn_ray_impl(const Vector<T, 3>& wi) const {
        auto o = offset_ray_origin(m_p_lower, m_p_upper, wi, m_n);
        return Ray<T, 3>(o, wi);
    }

    /// Spawn a ray from this interaction toward point @p p_to.
    Ray<T, 3> spawn_ray_to_impl(const Point<T, 3>& p_to) const {
        auto p_from = this->point();
        auto dir = p_to - p_from;
        auto o = offset_ray_origin(m_p_lower, m_p_upper, dir, m_n);
        auto dist = dir.length();
        dir = dir / dist;
        return Ray<T, 3>(o, dir, safe_ray_tmax(dist));
    }
};

}
