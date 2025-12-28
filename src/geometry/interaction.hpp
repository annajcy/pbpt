/**
 * @file interaction.hpp
 * @brief Interaction records for points where rays hit geometry or lights.
 * * Hierarchy:
 * 1. Interaction (Base): Position error bounds, outgoing direction (wo).
 * 2. NormalInteractionBase (Middle): Adds Normal (n), implements robust ray spawning.
 * 3. NormalInteraction (Leaf): Lightweight, used for NEE/Light sampling.
 * 4. SurfaceInteraction (Leaf): Heavyweight, adds UV, dpdu, dpdv, etc.
 */
#pragma once

#include <cmath>

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

    auto nv = n.to_vector();
    math::Vector<T, 3> offset = d * nv;

    // Flip offset if ray direction is opposite to normal (transmissive/refractive case)
    if (nv.dot(dir) < static_cast<T>(0)) {
        offset = -offset;
    }

    return p_mid + offset;
}

// -----------------------------------------------------------------------------
// Level 1: Interaction (Base)
// -----------------------------------------------------------------------------

/**
 * @brief Base interaction record.
 * Handles position error bounds (p_lower/p_upper) and outgoing direction (wo).
 */
template<typename T, typename Derived>
class Interaction {
protected:
    /// Lower bound of the hit position (Render Space).
    math::Point<T, 3> m_p_lower;
    /// Upper bound of the hit position (Render Space).
    math::Point<T, 3> m_p_upper;
    /// Outgoing direction towards the camera/previous path vertex (Render Space).
    math::Vector<T, 3> m_wo;

public:
    Interaction() = default;
    Interaction(
        const math::Point<T, 3>& p_lower, 
        const math::Point<T, 3>& p_upper, 
        const math::Vector<T, 3>& wo
    ) : m_p_lower(p_lower), m_p_upper(p_upper), m_wo(wo) {}

    Interaction(
        const math::Point<T, 3>& p, 
        const math::Vector<T, 3>& wo, 
        const math::Vector<T, 3>& error_margin = math::Vector<T, 3>{0, 0, 0}
    ) : m_p_lower(p - error_margin), m_p_upper(p + error_margin), m_wo(wo) {}

    // --- Accessors ---
    const math::Vector<T, 3>& wo() const { return m_wo; }
    const math::Point<T, 3>& p_lower() const { return m_p_lower; }
    const math::Point<T, 3>& p_upper() const { return m_p_upper; }
    math::Point<T, 3> point() const { return m_p_lower.mid(m_p_upper); }

    // --- CRTP Interface ---
    
    // Delegate to derived implementation
    geometry::Ray<T, 3> spawn_ray(const math::Vector<T, 3>& wi) const {
        return as_derived().spawn_ray_impl(wi);
    }

    geometry::Ray<T, 3> spawn_ray_to(const math::Point<T, 3>& p_to) const {
        return as_derived().spawn_ray_to_impl(p_to);
    }

    // Cast helpers
    const Derived& as_derived() const { return static_cast<const Derived&>(*this); }
    Derived& as_derived() { return static_cast<Derived&>(*this); }
};

// -----------------------------------------------------------------------------
// Level 2: NormalInteractionBase (Intermediate)
// -----------------------------------------------------------------------------

/**
 * @brief Intermediate class that adds Normal and robust ray spawning logic.
 * * Crucially, it keeps the `Derived` template parameter to pass it down to leaf classes.
 * This allows both SurfaceInteraction and NormalInteraction to share the 
 * spawn_ray_impl logic while remaining distinct types in the CRTP hierarchy.
 */
template<typename T, typename Derived>
class NormalInteractionBase : public Interaction<T, Derived> {
protected:
    /// Geometric surface normal (Render Space).
    math::Normal<T, 3> m_n;
    math::Normal<T, 3> m_shading_n;

public:
    NormalInteractionBase() = default;
    NormalInteractionBase(
        const math::Point<T, 3>& p_lower,
        const math::Point<T, 3>& p_upper,
        const math::Vector<T, 3>& wo,
        const math::Normal<T, 3>& n,
        const math::Normal<T, 3>& shading_n
    ) : Interaction<T, Derived>(p_lower, p_upper, wo), m_n(n), m_shading_n(shading_n) {}

    NormalInteractionBase(
        const math::Point<T, 3>& p,
        const math::Vector<T, 3>& wo,
        const math::Normal<T, 3>& n,
        const math::Normal<T, 3>& shading_n,
        const math::Vector<T, 3>& error_margin = math::Vector<T, 3>{0, 0, 0}
    ) : Interaction<T, Derived>(p, wo, error_margin), m_n(n), m_shading_n(shading_n) {}

    const math::Normal<T, 3>& n() const { return m_n; }
    const math::Normal<T, 3>& shading_n() const { return m_shading_n; }

    // --- Shared Implementation for Ray Spawning ---

    /// Spawn a ray in direction @p wi, offset away from the surface based on Normal.
    Ray<T, 3> spawn_ray_impl(const math::Vector<T, 3>& wi) const {
        // Access m_p_lower via this-> because it's a dependent name in template inheritance
        auto o = offset_ray_origin(this->m_p_lower, this->m_p_upper, wi, m_n);
        return Ray<T, 3>(o, wi);
    }

    /// Spawn a ray toward a specific point @p p_to.
    Ray<T, 3> spawn_ray_to_impl(const math::Point<T, 3>& p_to) const {
        auto p_from = this->point();
        auto dir = p_to - p_from;
        
        // Use the direction to p_to to decide offset direction
        auto o = offset_ray_origin(this->m_p_lower, this->m_p_upper, dir, m_n);
        
        auto dist = dir.length();
        dir = dir / dist; // Normalize
        
        // Set t_max slightly less than distance to avoid hitting the target itself 
        // (if the target is another surface)
        return Ray<T, 3>(o, dir, safe_ray_tmax(dist));
    }

    void filp_normals() {
        m_n = -m_n;
        m_shading_n = -m_shading_n;
    }
};

// -----------------------------------------------------------------------------
// Level 3: Concrete Leaf Classes
// -----------------------------------------------------------------------------

/**
 * @brief Lightweight interaction with just Position, Normal, and Wo.
 * Used for Light Sampling (NEE) where UVs and differentials are unnecessary.
 */
template<typename T>
class NormalInteraction : public NormalInteractionBase<T, NormalInteraction<T>> {
public:
    NormalInteraction(
        const math::Point<T, 3>& p_lower,
        const math::Point<T, 3>& p_upper,
        const math::Vector<T, 3>& wo,
        const math::Normal<T, 3>& n,
        const math::Normal<T, 3>& shading_n 
    ) : NormalInteractionBase<T, NormalInteraction<T>>(p_lower, p_upper, wo, n, shading_n) {}

    NormalInteraction(
        const math::Point<T, 3>& p,
        const math::Vector<T, 3>& wo,
        const math::Normal<T, 3>& n,
        const math::Normal<T, 3>& shading_n,
        const math::Vector<T, 3>& error_margin = math::Vector<T, 3>{0, 0, 0}
    ) : NormalInteractionBase<T, NormalInteraction<T>>(p, wo, n, shading_n, error_margin) {}
};

/**
 * @brief Full surface interaction with UVs and partial derivatives.
 * Used for geometry intersection and material shading.
 */
template<typename T>
class SurfaceInteraction : public NormalInteractionBase<T, SurfaceInteraction<T>> {
    // Friend base to allow access to spawn_ray_impl if it were protected (it's public here)
    // friend class NormalInteractionBase<T, SurfaceInteraction<T>>;

protected:
    math::Point<T, 2> m_uv;
    math::Vector<T, 3> m_dpdu;
    math::Vector<T, 3> m_dpdv;
    math::Normal<T, 3> m_dndu;
    math::Normal<T, 3> m_dndv;

    math::Vector<T, 3> m_shading_dpdu;
    math::Vector<T, 3> m_shading_dpdv;
    math::Normal<T, 3> m_shading_dndu;
    math::Normal<T, 3> m_shading_dndv;

public:
    SurfaceInteraction() = default;
    // Constructor 1: Full details
    SurfaceInteraction(
        const math::Point<T, 3>& p_lower,
        const math::Point<T, 3>& p_upper,
        const math::Vector<T, 3>& wo,
        const math::Normal<T, 3>& n,
        const math::Normal<T, 3>& shading_n,
        const math::Point<T, 2>& uv,
        const math::Vector<T, 3>& dpdu,
        const math::Vector<T, 3>& dpdv,
        const math::Normal<T, 3>& dndu,
        const math::Normal<T, 3>& dndv,
        const math::Vector<T, 3>& shading_dpdu,
        const math::Vector<T, 3>& shading_dpdv,
        const math::Normal<T, 3>& shading_dndu,
        const math::Normal<T, 3>& shading_dndv
    ) : NormalInteractionBase<T, SurfaceInteraction<T>>(p_lower, p_upper, wo, n, shading_n),
        m_uv(uv), m_dpdu(dpdu), m_dpdv(dpdv), m_dndu(dndu), m_dndv(dndv), m_shading_dpdu(shading_dpdu), m_shading_dpdv(shading_dpdv), m_shading_dndu(shading_dndu), m_shading_dndv(shading_dndv) {}

    // Constructor 2: Central point + margin
    SurfaceInteraction(
        const math::Point<T, 3>& p,
        const math::Vector<T, 3>& wo,
        const math::Normal<T, 3>& n,
        const math::Normal<T, 3>& shading_n,
        const math::Point<T, 2>& uv,
        const math::Vector<T, 3>& dpdu,
        const math::Vector<T, 3>& dpdv,
        const math::Normal<T, 3>& dndu,
        const math::Normal<T, 3>& dndv,
        const math::Vector<T, 3>& shading_dpdu,
        const math::Vector<T, 3>& shading_dpdv,
        const math::Normal<T, 3>& shading_dndu,
        const math::Normal<T, 3>& shading_dndv,
        const math::Vector<T, 3>& error_margin = math::Vector<T, 3>{0, 0, 0}
    ) : NormalInteractionBase<T, SurfaceInteraction<T>>(p, wo, n, shading_n, error_margin),
        m_uv(uv), m_dpdu(dpdu), m_dpdv(dpdv), m_dndu(dndu), m_dndv(dndv), m_shading_dpdu(shading_dpdu), m_shading_dpdv(shading_dpdv), m_shading_dndu(shading_dndu), m_shading_dndv(shading_dndv) {}

    // Constructor 3: Simplified for light sources (NEE) 
    // Constructs a full SurfaceInteraction but with zero derivatives.
    // Useful if you have a uniform area light but the interface expects a SurfaceInteraction.
    SurfaceInteraction(
        const math::Point<T, 3>& p,
        const math::Vector<T, 3>& error_margin,
        const math::Point<T, 2>& uv,
        const math::Vector<T, 3>& wo,
        const math::Normal<T, 3>& n
    ) : NormalInteractionBase<T, SurfaceInteraction<T>>(p, wo, n, error_margin),
        m_uv(uv), 
        m_dpdu({0,0,0}), m_dpdv({0,0,0}), 
        m_dndu({0,0,0}), m_dndv({0,0,0})
    {}

    // --- Accessors ---
    const math::Point<T, 2>& uv() const { return m_uv; }
    const math::Vector<T, 3>& dpdu() const { return m_dpdu; }
    const math::Vector<T, 3>& dpdv() const { return m_dpdv; }
    const math::Normal<T, 3>& dndu() const { return m_dndu; }
    const math::Normal<T, 3>& dndv() const { return m_dndv; }
    const math::Vector<T, 3>& shading_dpdu() const { return m_shading_dpdu; }
    const math::Vector<T, 3>& shading_dpdv() const { return m_shading_dpdv; }
    const math::Normal<T, 3>& shading_dndu() const { return m_shading_dndu; }
    const math::Normal<T, 3>& shading_dndv() const { return m_shading_dndv; }
};

} // namespace pbpt::geometry
