#pragma once

#include "math/point.hpp"
#include "math/vector.hpp"
#include "math/normal.hpp"
#include "geometry/ray.hpp"

using namespace pbpt::math;
using namespace pbpt::geometry;

namespace pbpt::core {

template<typename T>
inline Point<T, 3> offset_ray_origin(
    const PointInterval<T, 3>& pi, 
    const Vector<T, 3>& wi, 
    const Normal<T, 3>& n
) {
    const auto p = to_point(pi);

    auto d = abs(n.x()) * pi.x().width() + 
            abs(n.y()) * pi.y().width() + 
            abs(n.z()) * pi.z().width();

    constexpr auto gamma7 = gamma<T>(7);

    d += 2 * gamma7 * (
            abs(p.x() * abs(n.x())) + 
            abs(p.y() * abs(n.y())) + 
            abs(p.z() * abs(n.z()))
        );

    auto nv = n.to_vector();
    Vector<T, 3> offset = d * nv;
    if (is_less(nv.dot(wi), 0)) {
        offset = -offset;
    }

    return p + offset;
}

template<typename T, typename Derived>
class Interaction {
private:
    PointInterval<T, 3> m_pi; // Interaction point with intervals
    Vector<T, 3> m_wo;

public:
    Interaction(const PointInterval<T, 3>& pi, const Vector<T, 3>& wo)
        : m_pi(pi), m_wo(wo) {}

    const PointInterval<T, 3>& pi() const { return m_pi; }
    const Point<T, 3> p() const { return to_point(m_pi); }
    const Vector<T, 3>& wo() const { return m_wo; }

    Ray<T, 3> spawn_ray(const Vector<T, 3>& wi) const {
        return as_derived().spawn_ray_impl(wi);
    }

    Ray<T, 3> spawn_ray_to(const Point<T, 3>& p) const {
        return as_derived().spawn_ray_to_impl(p);
    }

    template<typename OtherDerived>
    Ray<T, 3> spawn_ray_to(const Interaction<T, OtherDerived>& target) const {
        return as_derived().spawn_ray_to_impl(static_cast<const OtherDerived&>(target));
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
private:
    Normal<T, 3> m_n;
    Point<T, 2> m_uv;
    Vector<T, 3> m_dpdu, m_dpdv;
    Normal<T, 3> m_dndu, m_dndv;

public:
    using base = Interaction<T, SurfaceInteraction<T>>;
    using base::pi;
    using base::wo;

    SurfaceInteraction(const PointInterval<T, 3>& pi, const Vector<T, 3>& wo, const Normal<T, 3>& n, const Point<T, 2>& uv,
                      const Vector<T, 3>& dpdu, const Vector<T, 3>& dpdv, const Normal<T, 3>& dndu, const Normal<T, 3>& dndv)
        : Interaction<T, SurfaceInteraction<T>>(pi, wo), m_n(n), m_uv(uv), m_dpdu(dpdu), m_dpdv(dpdv), m_dndu(dndu), m_dndv(dndv) {}

    const Normal<T, 3>& n() const { return m_n; }
    const Point<T, 2>& uv() const { return m_uv; }
    const Vector<T, 3>& dpdu() const { return m_dpdu; }
    const Vector<T, 3>& dpdv() const { return m_dpdv; }
    const Normal<T, 3>& dndu() const { return m_dndu; }
    const Normal<T, 3>& dndv() const { return m_dndv; }

    Ray<T, 3> spawn_ray_impl(const Vector<T, 3>& wi) const {
        auto o = offset_ray_origin(pi(), wi, m_n);
        return Ray<T, 3>(o, wi);
    }

    Ray<T, 3> spawn_ray_to_impl(const Point<T, 3>& p) const {
        auto p_from = to_point(pi());
        auto dir = p - p_from;
        auto o = offset_ray_origin(pi(), dir, m_n);
        auto dist = dir.length();
        dir = dir / dist;
        return Ray<T, 3>(o, dir, safe_ray_tmax(dist));
    }

    Ray<T, 3> spawn_ray_to_impl(const SurfaceInteraction& target) const {
        const Point<T, 3> p_from = to_point(this->pi());
        const Point<T, 3> p_to   = to_point(target.pi());

        const Vector<T, 3> dir = p_to - p_from;
        const T dist = dir.length();
        const Vector<T, 3> dir_norm = dir / dist;

        const Point<T, 3> o1 = offset_ray_origin(this->pi(), dir_norm, this->n());
        const Point<T, 3> o2 = offset_ray_origin(target.pi(), -dir_norm, target.n());

        const Vector<T, 3> final_dir = o2 - o1;
        const T final_dist = final_dir.length();

        return Ray<T, 3>(o1, final_dir / final_dist, safe_ray_tmax(final_dist));
    }
};

}