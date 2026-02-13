/**
 * @file interaction.hpp
 * @brief Interaction records for points where rays hit geometry or lights.
 * SurfaceInteraction stores geometric data; shading data is provided separately.
 */
#pragma once

#include <cmath>
#include <optional>
#include <algorithm>

#include "pbpt/math/point.hpp"
#include "pbpt/math/vector.hpp"
#include "pbpt/math/normal.hpp"
#include "pbpt/math/matrix.hpp"
#include "pbpt/geometry/ray.hpp"

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
    math::Normal<T, 3> dndu{};
    math::Normal<T, 3> dndv{};
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
    math::Normal<T, 3> m_dndu;
    math::Normal<T, 3> m_dndv;

public:
    SurfaceInteraction() = default;

    SurfaceInteraction(
        const math::Point<T, 3>& p_lower,
        const math::Point<T, 3>& p_upper,
        const math::Vector<T, 3>& wo,
        const math::Normal<T, 3>& n,
        const math::Point<T, 2>& uv,
        const math::Vector<T, 3>& dpdu,
        const math::Vector<T, 3>& dpdv,
        const math::Normal<T, 3>& dndu = math::Normal<T, 3>(0, 0, 0),
        const math::Normal<T, 3>& dndv = math::Normal<T, 3>(0, 0, 0)
    ) : m_p_lower(p_lower), m_p_upper(p_upper), m_wo(wo), m_n(n), m_uv(uv), m_dpdu(dpdu), m_dpdv(dpdv), m_dndu(dndu), m_dndv(dndv) {}

    SurfaceInteraction(
        const math::Point<T, 3>& p,
        const math::Vector<T, 3>& wo,
        const math::Normal<T, 3>& n,
        const math::Point<T, 2>& uv,
        const math::Vector<T, 3>& dpdu,
        const math::Vector<T, 3>& dpdv,
        const math::Vector<T, 3>& error_margin = math::Vector<T, 3>{0, 0, 0},
        const math::Normal<T, 3>& dndu = math::Normal<T, 3>(0, 0, 0),
        const math::Normal<T, 3>& dndv = math::Normal<T, 3>(0, 0, 0)
    ) : m_p_lower(p - error_margin), m_p_upper(p + error_margin), m_wo(wo), m_n(n), m_uv(uv), m_dpdu(dpdu), m_dpdv(dpdv), m_dndu(dndu), m_dndv(dndv) {}

    // --- Accessors ---
    const math::Vector<T, 3>& wo() const { return m_wo; }
    const math::Point<T, 3>& p_lower() const { return m_p_lower; }
    const math::Point<T, 3>& p_upper() const { return m_p_upper; }
    math::Point<T, 3> point() const { return m_p_lower.mid(m_p_upper); }

    const math::Normal<T, 3>& n() const { return m_n; }
    const math::Point<T, 2>& uv() const { return m_uv; }
    const math::Vector<T, 3>& dpdu() const { return m_dpdu; }
    const math::Vector<T, 3>& dpdv() const { return m_dpdv; }
    const math::Normal<T, 3>& dndu() const { return m_dndu; }
    const math::Normal<T, 3>& dndv() const { return m_dndv; }

    // --- Ray Spawning ---
    Ray<T, 3> spawn_ray(const math::Vector<T, 3>& wi) const {
        auto o = offset_ray_origin(m_p_lower, m_p_upper, wi, m_n);
        return Ray<T, 3>(o, wi);
    }

    RayDifferential<T, 3> spawn_ray_differential(
        const math::Vector<T, 3>& wi,
        const RayDifferentialOffset<T>& diffs_offset
    ) const {
        // 1. Compute main ray origin with Shadow Acne avoidance
        auto o = offset_ray_origin(m_p_lower, m_p_upper, wi, m_n);
        Ray<T, 3> main_ray(o, wi);

        // 2. Apply position deltas to the SAFE origin
        // This ensures differential rays maintain a similar distance from the surface
        auto ox = o + diffs_offset.dpdx;
        auto oy = o + diffs_offset.dpdy;

        // 3. Apply direction deltas to the main direction
        auto dir_x = wi + diffs_offset.dwdx;
        auto dir_y = wi + diffs_offset.dwdy;

        Ray<T, 3> ray_x(ox, dir_x);
        Ray<T, 3> ray_y(oy, dir_y);

        return RayDifferential<T, 3>(main_ray, {ray_x, ray_y});
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
        m_dndu = -m_dndu;
        m_dndv = -m_dndv;
    }

    std::optional<SurfaceDifferentials<T>> compute_differentials(
        const geometry::RayDifferential<T, 3>& ray_diff
    ) const {
        // Treat "no perturbation" differential rays as invalid differentials.
        const auto& main_ray = ray_diff.main_ray();
        auto is_same_ray = [&](const geometry::Ray<T, 3>& ray) {
            auto origin_delta = ray.origin() - main_ray.origin();
            auto direction_delta = ray.direction() - main_ray.direction();
            return origin_delta.length_squared() <= math::epsilon_v<T> &&
                   direction_delta.length_squared() <= math::epsilon_v<T>;
        };
        if (is_same_ray(ray_diff.x()) && is_same_ray(ray_diff.y())) {
            return std::nullopt;
        }

        auto n_vec = m_n.to_vector();
        auto p = point();

        T tx_denom = n_vec.dot(ray_diff.x().direction());
        T ty_denom = n_vec.dot(ray_diff.y().direction());
        if (std::abs(tx_denom) < math::epsilon_v<T> || std::abs(ty_denom) < math::epsilon_v<T>) {
            return std::nullopt;
        }

        T x_d = -n_vec.dot(ray_diff.x().origin() - p);
        T y_d = -n_vec.dot(ray_diff.y().origin() - p);
        
        T tx = x_d / tx_denom;
        T ty = y_d / ty_denom;

        if (!std::isfinite(tx) || !std::isfinite(ty)) {
            return std::nullopt;
        }

        auto px = ray_diff.x().origin() + ray_diff.x().direction() * tx;
        auto py = ray_diff.y().origin() + ray_diff.y().direction() * ty;

        SurfaceDifferentials<T> diffs;
        diffs.dpdx = px - p;
        diffs.dpdy = py - p;

        math::Matrix<T, 2, 3> A;
        for (int i = 0; i < 3; ++i) {
            A.at(0, i) = m_dpdu[i];
            A.at(1, i) = m_dpdv[i];
        }

        math::Matrix<T, 2, 3> B;
        for (int i = 0; i < 3; ++i) {
            B.at(0, i) = diffs.dpdx[i];
            B.at(1, i) = diffs.dpdy[i];
        }

        auto solution = math::solve_LMS(A, B);

        T limit = static_cast<T>(1e8);
        auto clamp_val = [&](T val) {
            if (!std::isfinite(val)) return T(0);
            return std::clamp(val, -limit, limit);
        };

        diffs.dudx = clamp_val(solution.at(0, 0));
        diffs.dvdx = clamp_val(solution.at(0, 1));
        diffs.dudy = clamp_val(solution.at(1, 0));
        diffs.dvdy = clamp_val(solution.at(1, 1));

        return diffs;
    }
};

} // namespace pbpt::geometry
