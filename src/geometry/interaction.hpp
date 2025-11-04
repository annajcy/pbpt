#pragma once

#include <utility>
#include "math/point.hpp"
#include "math/vector.hpp"
#include "math/normal.hpp"
#include "geometry/ray.hpp"

namespace pbpt::geometry {

template<typename T>
inline math::Point<T, 3> offset_ray_origin(
    const math::Point<T, 3>& p_lower,
    const math::Point<T, 3>& p_upper,
    const Vector<T, 3>& wi, 
    const Normal<T, 3>& n
) {

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
    if (is_less(nv.dot(wi), 0)) {
        offset = -offset;
    }

    return p_mid + offset;
}

template<typename T, typename Derived>
class Interaction {
protected:
    math::Point<T, 3> m_p_lower;
    math::Point<T, 3> m_p_upper;
    math::Vector<T, 3> m_wo;

public:
    Interaction(const math::Point<T, 3>& p_lower, const math::Point<T, 3>& p_upper, const math::Vector<T, 3>& wo)
        : m_p_lower(p_lower), m_p_upper(p_upper), m_wo(wo) {}

    Interaction(const math::Point<T, 3>& p, const math::Vector<T, 3>& wo)
        : m_p_lower(p), m_p_upper(p), m_wo(wo) {}

    const math::Vector<T, 3>& wo() const { return m_wo; }
    const math::Point<T, 3>& p_lower() const { return m_p_lower; }
    const math::Point<T, 3>& p_upper() const { return m_p_upper; }

    math::Point<T, 3> p() const { return m_p_lower.mid(m_p_upper); }

    geometry::Ray<T, 3> spawn_ray(const math::Vector<T, 3>& wi) const {
        return as_derived().spawn_ray_impl(wi);
    }

    geometry::Ray<T, 3> spawn_ray_to(const math::Point<T, 3>& p) const {
        return as_derived().spawn_ray_to_impl(p);
    }

    const Derived& as_derived() const {
        return static_cast<const Derived&>(*this);
    }

    Derived& as_derived() {
        return static_cast<Derived&>(*this);
    }
};

template<typename T>
class SurfaceInteraction : public Interaction<T, SurfaceInteraction<T>> {
    friend class Interaction<T, SurfaceInteraction<T>>;

    using base = Interaction<T, SurfaceInteraction<T>>;
    using base::m_p_lower;
    using base::m_p_upper;
    using base::m_wo;

protected:
    Normal<T, 3> m_n;
    Point<T, 2> m_uv;
    Vector<T, 3> m_dpdu, m_dpdv;
    Normal<T, 3> m_dndu, m_dndv;

public:
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

    SurfaceInteraction(
        math::Point<T, 3> p,
        const math::Vector<T, 3>& wo,
        const Normal<T, 3>& n,
        const Point<T, 2>& uv,
        const Vector<T, 3>& dpdu,
        const Vector<T, 3>& dpdv,
        const Normal<T, 3>& dndu,
        const Normal<T, 3>& dndv
    ) : Interaction<T, SurfaceInteraction<T>>(p, wo),
          m_n(n),
          m_uv(uv),
          m_dpdu(dpdu),
          m_dpdv(dpdv),
          m_dndu(dndu),
          m_dndv(dndv) {}

    const Normal<T, 3>& n() const { return m_n; }
    const Point<T, 2>& uv() const { return m_uv; }
    const Vector<T, 3>& dpdu() const { return m_dpdu; }
    const Vector<T, 3>& dpdv() const { return m_dpdv; }
    const Normal<T, 3>& dndu() const { return m_dndu; }
    const Normal<T, 3>& dndv() const { return m_dndv; }

    Ray<T, 3> spawn_ray_impl(const Vector<T, 3>& wi) const {
        auto o = offset_ray_origin(m_p_lower, m_p_upper, wi, m_n);
        return Ray<T, 3>(o, wi);
    }

    Ray<T, 3> spawn_ray_to_impl(const Point<T, 3>& p_to) const {
        auto p_from = this->p();
        auto dir = p_to - p_from;
        auto o = offset_ray_origin(m_p_lower, m_p_upper, dir, m_n);
        auto dist = dir.length();
        dir = dir / dist;
        return Ray<T, 3>(o, dir, safe_ray_tmax(dist));
    }
};

}