/**
 * @file interaction.hpp
 * @brief Interaction records for points where rays hit geometry or lights.
 * SurfaceInteraction stores geometric data; shading data is provided separately.
 */
#pragma once

#include <cmath>
#include <optional>

#include "math/point.hpp"
#include "math/vector.hpp"
#include "math/normal.hpp"
#include "geometry/ray.hpp"

namespace pbpt::geometry {

// -----------------------------------------------------------------------------
// Helper Functions
// -----------------------------------------------------------------------------

/**
 * @brief Compute a conservative t_max for a ray of length @p dist.
 *
 * The returned value is slightly smaller than @p dist by an amount
 * proportional to a small gamma factor, which helps avoid "overshooting"
 * due to floating-point error when tracing to a known distance.
 */
template<typename T>
inline T safe_ray_tmax(T dist, T factor = static_cast<T>(3.0)) {
    // 缩短一点点 t_max 以防止精度误差导致的穿透
    return dist * (static_cast<T>(1) - math::epsilon_v<T> * factor);
}

/**
 * @brief Offset a ray origin to avoid self-intersection (Shadow Acne).
 */
template<typename T>
inline math::Point<T, 3> offset_ray_origin(
    const math::Point<T, 3>& p_lower,
    const math::Point<T, 3>& p_upper,
    const math::Vector<T, 3>& dir, // Direction of the NEW ray
    const math::Normal<T, 3>& n
) {
    // Determine the offset magnitude based on the error box projection onto the normal
    auto d = std::abs(n.x()) * (p_upper.x() - p_lower.x()) + 
             std::abs(n.y()) * (p_upper.y() - p_lower.y()) + 
             std::abs(n.z()) * (p_upper.z() - p_lower.z());

    auto p_mid = p_lower.mid(p_upper);

    // Scaling factor for robustness (gamma(7) is standard in PBRT)
    // Assuming math::gamma<T>(7) exists. If not, use (7 * machine_epsilon).
    constexpr T gamma7 = math::gamma<T>(7); 

    d += static_cast<T>(2) * gamma7 * (
            std::abs(p_mid.x() * std::abs(n.x())) + 
            std::abs(p_mid.y() * std::abs(n.y())) + 
            std::abs(p_mid.z() * std::abs(n.z()))
        );

    // Minimal offset to handle cases where calculation yields zero (e.g. at origin)
    if (d < math::epsilon_v<T> * static_cast<T>(1024)) {
        d = math::epsilon_v<T> * static_cast<T>(1024);
    }

    auto nv = n.to_vector();
    math::Vector<T, 3> offset = d * nv;

    // Flip offset if ray direction is opposite to normal (transmissive/refractive case)
    if (nv.dot(dir) < static_cast<T>(0)) {
        offset = -offset;
    }

    return p_mid + offset;
}

// -----------------------------------------------------------------------------
// Shading Info (separate from geometric interaction)
// -----------------------------------------------------------------------------

template<typename T>
struct ShadingInfo {
    math::Normal<T, 3> n{};
};

// -----------------------------------------------------------------------------
// Surface Interaction
// -----------------------------------------------------------------------------

/**
 * @brief Full surface interaction with geometric frame only.
 */
template<typename T>
class SurfaceInteraction {
private:
    /// Lower bound of the hit position (Render Space).
    math::Point<T, 3> m_p_lower;
    /// Upper bound of the hit position (Render Space).
    math::Point<T, 3> m_p_upper;
    /// Outgoing direction towards the camera/previous path vertex (Render Space).
    math::Vector<T, 3> m_wo;
    /// Geometric surface normal (Render Space).
    math::Normal<T, 3> m_n;

    math::Point<T, 2> m_uv;
    math::Vector<T, 3> m_dpdu;
    math::Vector<T, 3> m_dpdv;

public:
    SurfaceInteraction() = default;

    SurfaceInteraction(
        const math::Point<T, 3>& p_lower,
        const math::Point<T, 3>& p_upper,
        const math::Vector<T, 3>& wo,
        const math::Normal<T, 3>& n,
        const math::Point<T, 2>& uv,
        const math::Vector<T, 3>& dpdu,
        const math::Vector<T, 3>& dpdv
    ) : m_p_lower(p_lower), m_p_upper(p_upper), m_wo(wo), m_n(n), m_uv(uv), m_dpdu(dpdu), m_dpdv(dpdv) {}

    SurfaceInteraction(
        const math::Point<T, 3>& p,
        const math::Vector<T, 3>& wo,
        const math::Normal<T, 3>& n,
        const math::Point<T, 2>& uv,
        const math::Vector<T, 3>& dpdu,
        const math::Vector<T, 3>& dpdv,
        const math::Vector<T, 3>& error_margin = math::Vector<T, 3>{0, 0, 0}
    ) : m_p_lower(p - error_margin), m_p_upper(p + error_margin), m_wo(wo), m_n(n), m_uv(uv), m_dpdu(dpdu), m_dpdv(dpdv) {}

    // --- Accessors ---
    const math::Vector<T, 3>& wo() const { return m_wo; }
    const math::Point<T, 3>& p_lower() const { return m_p_lower; }
    const math::Point<T, 3>& p_upper() const { return m_p_upper; }
    math::Point<T, 3> point() const { return m_p_lower.mid(m_p_upper); }

    const math::Normal<T, 3>& n() const { return m_n; }
    const math::Point<T, 2>& uv() const { return m_uv; }
    const math::Vector<T, 3>& dpdu() const { return m_dpdu; }
    const math::Vector<T, 3>& dpdv() const { return m_dpdv; }

    // --- Ray Spawning ---
    Ray<T, 3> spawn_ray(const math::Vector<T, 3>& wi) const {
        auto o = offset_ray_origin(m_p_lower, m_p_upper, wi, m_n);
        return Ray<T, 3>(o, wi);
    }

    Ray<T, 3> spawn_ray_to(const math::Point<T, 3>& p_to) const {
        auto p_from = point();
        auto dir = p_to - p_from;

        auto o = offset_ray_origin(m_p_lower, m_p_upper, dir, m_n);

        auto dist = dir.length();
        dir = dir / dist; // Normalize

        return Ray<T, 3>(o, dir, safe_ray_tmax(dist));
    }

    void flip_normal() {
        m_n = -m_n;
    }
};

// -----------------------------------------------------------------------------
// Surface Differentials
// -----------------------------------------------------------------------------

template<typename T>
struct SurfaceDifferentials {
    math::Vector<T, 3> dpdx{};
    math::Vector<T, 3> dpdy{};
    T dudx{0};
    T dvdx{0};
    T dudy{0};
    T dvdy{0};
};

template<typename T>
inline std::optional<SurfaceDifferentials<T>> compute_surface_differentials(
    const SurfaceInteraction<T>& si,
    const geometry::RayDifferential<T, 3>& ray
) {
    auto n_vec = si.n().to_vector();
    auto p = si.point();

    T tx_denom = n_vec.dot(ray.x().direction());
    T ty_denom = n_vec.dot(ray.y().direction());
    if (std::abs(tx_denom) < math::epsilon_v<T> || std::abs(ty_denom) < math::epsilon_v<T>) {
        return std::nullopt;
    }

    T tx = n_vec.dot(p - ray.x().origin()) / tx_denom;
    T ty = n_vec.dot(p - ray.y().origin()) / ty_denom;
    if (!std::isfinite(tx) || !std::isfinite(ty)) {
        return std::nullopt;
    }

    auto px = ray.x().origin() + ray.x().direction() * tx;
    auto py = ray.y().origin() + ray.y().direction() * ty;

    SurfaceDifferentials<T> diffs;
    diffs.dpdx = px - p;
    diffs.dpdy = py - p;
    T eps_sq = math::epsilon_v<T> * math::epsilon_v<T>;
    if (diffs.dpdx.length_squared() <= eps_sq && diffs.dpdy.length_squared() <= eps_sq) {
        return std::nullopt;
    }

    int dim0 = 0;
    int dim1 = 1;
    T abs_nx = std::abs(n_vec.x());
    T abs_ny = std::abs(n_vec.y());
    T abs_nz = std::abs(n_vec.z());
    if (abs_nx > abs_ny && abs_nx > abs_nz) {
        dim0 = 1;
        dim1 = 2;
    } else if (abs_ny > abs_nz) {
        dim0 = 0;
        dim1 = 2;
    }

    const auto& dpdu = si.dpdu();
    const auto& dpdv = si.dpdv();
    T a00 = dpdu[dim0];
    T a01 = dpdv[dim0];
    T a10 = dpdu[dim1];
    T a11 = dpdv[dim1];
    T det = a00 * a11 - a01 * a10;
    if (std::abs(det) > math::epsilon_v<T>) {
        T inv_det = T(1) / det;
        T bx0 = diffs.dpdx[dim0];
        T bx1 = diffs.dpdx[dim1];
        T by0 = diffs.dpdy[dim0];
        T by1 = diffs.dpdy[dim1];

        diffs.dudx = (bx0 * a11 - bx1 * a01) * inv_det;
        diffs.dvdx = (-bx0 * a10 + bx1 * a00) * inv_det;
        diffs.dudy = (by0 * a11 - by1 * a01) * inv_det;
        diffs.dvdy = (-by0 * a10 + by1 * a00) * inv_det;
    }

    return diffs;
}

} // namespace pbpt::geometry
