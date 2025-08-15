#pragma once

#include "math/geometry/normal.hpp"
#include "math/geometry/point.hpp"
#include "math/geometry/ray.hpp"
#include "math/geometry/vector.hpp"
#include "math/global/operator.hpp"
#include "math/global/type_alias.hpp"

using namespace pbpt::math;

namespace pbpt::core {

inline Pt3 offset_ray_origin(
    const Pt3Interv& pi, 
    const Vec3& wi, 
    const Normal3& n
) {
    const auto p = to_point(pi);

    auto d = abs(n.x()) * pi.x().width() + 
            abs(n.y()) * pi.y().width() + 
            abs(n.z()) * pi.z().width();

    constexpr auto gamma7 = gamma<Float>(7);

    d += 2 * gamma7 * (
            abs(p.x() * abs(n.x())) + 
            abs(p.y() * abs(n.y())) + 
            abs(p.z() * abs(n.z()))
        );

    auto nv = n.to_vector();
    Vec3 offset = d * nv;
    if (is_less(nv.dot(wi), 0)) {
        offset = -offset;
    }

    return p + offset;
}

template<typename Derived>
class Interaction {
private:
    Pt3Interv m_pi; // Interaction point with intervals
    Vec3 m_wo;

public:
    Interaction(const Pt3Interv& pi, const Vec3& wo)
        : m_pi(pi), m_wo(wo) {}

    const Pt3Interv& pi() const { return m_pi; }
    const Pt3 p() const { return to_point(m_pi); }
    const Vec3& wo() const { return m_wo; }

    Ray3 spawn_ray(const Vec3& wi) const {
        return static_cast<const Derived*>(this)->spawn_ray_impl(wi);
    }

    Ray3 spawn_ray_to(const Pt3& p) const {
        return static_cast<const Derived*>(this)->spawn_ray_to_impl(p);
    }

    template<typename OtherDerived>
    Ray3 spawn_ray_to(const Interaction<OtherDerived>& target) const {
        return static_cast<const Derived*>(this)->spawn_ray_to_impl(static_cast<const OtherDerived&>(target));
    }


};

class SurfaceInteraction : public Interaction<SurfaceInteraction> {
private:
    Normal3 m_n;
    Pt2 m_uv;
    Vec3 m_dpdu, m_dpdv;
    Normal3 m_dndu, m_dndv;

public:

    SurfaceInteraction(const Pt3Interv& pi, const Vec3& wo, const Normal3& n, const Pt2& uv,
                      const Vec3& dpdu, const Vec3& dpdv, const Normal3& dndu, const Normal3& dndv)
        : Interaction(pi, wo), m_n(n), m_uv(uv), m_dpdu(dpdu), m_dpdv(dpdv), m_dndu(dndu), m_dndv(dndv) {}

    const Normal3& n() const { return m_n; }
    const Pt2& uv() const { return m_uv; }
    const Vec3& dpdu() const { return m_dpdu; }
    const Vec3& dpdv() const { return m_dpdv; }
    const Normal3& dndu() const { return m_dndu; }
    const Normal3& dndv() const { return m_dndv; }

    Ray3 spawn_ray_impl(const Vec3& wi) const {
        auto o = offset_ray_origin(pi(), wi, m_n);
        return Ray3(o, wi);
    }

    Ray3 spawn_ray_to_impl(const Pt3& p) const {
        auto p_from = to_point(pi());
        auto dir = p - p_from;
        auto o = offset_ray_origin(pi(), dir, m_n);
        auto dist = dir.length();
        dir = dir / dist;
        return Ray3(o, dir, safe_ray_tmax(dist));
    }

    Ray3 spawn_ray_to_impl(const SurfaceInteraction& target) const {
        const Pt3 p_from = to_point(this->pi());
        const Pt3 p_to   = to_point(target.pi());

        const Vec3 dir = p_to - p_from;
        const Float dist = dir.length();
        const Vec3 dir_norm = dir / dist;

        const Pt3 o1 = offset_ray_origin(this->pi(), dir_norm, this->n());
        const Pt3 o2 = offset_ray_origin(target.pi(), -dir_norm, target.n());

        const Vec3 final_dir = o2 - o1;
        const Float final_dist = final_dir.length();

        return Ray3(o1, final_dir, safe_ray_tmax(final_dist));
    }


};

}