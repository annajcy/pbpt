#pragma once

#include <algorithm>
#include <cmath>
#include <optional>
#include <utility>

#include "geometry/bounds.hpp"
#include "geometry/directional_cone.hpp"
#include "geometry/interaction.hpp"
#include "geometry/ray.hpp"
#include "math/function.hpp"
#include "math/normal.hpp"
#include "math/point.hpp"
#include "math/sampling.hpp"

#include "math/vector.hpp"
#include "shape/shape.hpp"

namespace pbpt::shape {

/**
 * @brief Sphere shape supporting partial spheres and analytic sampling.
 *
 * The sphere can be restricted in z and azimuth (phi) to represent
 * spherical caps or wedges. It implements all `Shape` interface
 * methods such as area, bounds, ray intersection and sampling.
 *
 * @tparam T Scalar type.
 */
template<typename T>
class Sphere : public Shape<Sphere<T>, T> {
    friend class Shape<Sphere<T>, T>;

public:
    /**
     * @brief Detailed information about a ray–sphere intersection.
     *
     * Stores the hit point, the parametric distance along the ray and
     * the azimuth angle phi in [0, 2*pi].
     */
    template<typename U>
    struct IntersectionResult {
        /// Hit point in render space.
        math::Point<U, 3> p_hit;
        /// Parametric distance along the ray where the hit occurs.
        U t_hit;
        /// Azimuthal angle of the hit point around the z-axis.
        U phi;
    };

private:
    T m_radius{};
    T m_z_min{};
    T m_z_max{};
    T m_phi_max{};

public:
    /**
     * @brief Constructs a full sphere of the given radius.
     */
    Sphere(T radius) : m_radius(radius) {
        m_z_min = -radius;
        m_z_max = radius;
        m_phi_max = static_cast<T>(2 * math::pi_v<T>);
    }

    /**
     * @brief Constructs a partial sphere.
     *
     * The sphere is truncated in z to [z_min, z_max] and in azimuth
     * to [0, phi_max]. Input z-limits are clamped to [-radius, radius]
     * and swapped if needed so that z_min <= z_max.
     */
    Sphere(T radius, T z_min, T z_max, T phi_max)
        : m_radius(radius),
          m_z_min(std::clamp(z_min, -radius, radius)),
          m_z_max(std::clamp(z_max, -radius, radius)),
          m_phi_max(std::clamp(phi_max, static_cast<T>(0), static_cast<T>(2 * math::pi_v<T>))
    ) {
        if (m_z_min > m_z_max) {
            std::swap(m_z_min, m_z_max);
        }
    }

    /// Minimum z of the spherical segment.
    T z_min() const { return m_z_min; }
    /// Maximum z of the spherical segment.
    T z_max() const { return m_z_max; }
    /// Maximum azimuth angle in radians (<= 2*pi).
    T phi_max() const { return m_phi_max; }
    /// Sphere radius.
    T radius() const { return m_radius; }

    /// Polar angle corresponding to z_min.
    T z_min_theta() const {
        return std::acos(std::clamp(m_z_min / m_radius, static_cast<T>(-1), static_cast<T>(1)));
    }

    /// Polar angle corresponding to z_max.
    T z_max_theta() const {
        return std::acos(std::clamp(m_z_max / m_radius, static_cast<T>(-1), static_cast<T>(1)));
    }

private:
    geometry::Bounds<T, 3> bounding_box_impl() const {
        return geometry::Bounds<T, 3>(
            math::Point<T, 3>(-m_radius, -m_radius, m_z_min),
            math::Point<T, 3>(m_radius, m_radius, m_z_max)
        );
    }

    geometry::DirectionalCone<T> normal_bounding_cone_impl() const {
        return geometry::DirectionalCone<T>::entire_sphere();
    }

    std::pair<std::optional<T>, std::optional<T>> basic_intersect(const geometry::Ray<T, 3>& ray) const {
        auto a = ray.direction().length_squared();
        auto origin_vec = ray.origin().to_vector();
        auto b = static_cast<T>(2) * ray.direction().dot(origin_vec);
        auto c = origin_vec.length_squared() - m_radius * m_radius;

        auto discriminant = b * b - static_cast<T>(4) * a * c;
        if (discriminant < static_cast<T>(0)) {
            return {std::nullopt, std::nullopt};
        }

        auto sqrt_discriminant = std::sqrt(discriminant);
        auto t0 = (-b - sqrt_discriminant) / (static_cast<T>(2) * a);
        auto t1 = (-b + sqrt_discriminant) / (static_cast<T>(2) * a);
        return {t0, t1};
    }

    std::optional<IntersectionResult<T>> is_intersected(const geometry::Ray<T, 3>& ray) const {
        auto [t0_opt, t1_opt] = basic_intersect(ray);
        if (!t0_opt.has_value()) {
            return std::nullopt;
        }

        auto t0 = t0_opt.value();
        auto t1 = t1_opt.value();

        if (t0 > ray.t_max() || t1 < ray.t_min()) {
            return std::nullopt;
        }

        auto t_hit = t0;
        if (t_hit < ray.t_min()) {
            t_hit = t1;
            if (t_hit > ray.t_max()) {
                return std::nullopt;
            }
        }
        
        auto p_hit = ray.at(t_hit);
        auto phi = std::atan2(p_hit.y(), p_hit.x());
        if (phi < static_cast<T>(0)) {
            phi += static_cast<T>(2 * math::pi_v<T>);
        }

        if (p_hit.z() < m_z_min || p_hit.z() > m_z_max || phi > m_phi_max) {
            return std::nullopt;
        }

        return std::make_optional(IntersectionResult<T>{
            p_hit, t_hit, phi
        });
    }

    std::optional<T> is_intersected_impl(const geometry::Ray<T, 3>& ray) const {
        auto result_opt = is_intersected(ray);
        if (!result_opt.has_value()) {
            return std::nullopt;
        }
        return std::make_optional(result_opt->t_hit);
    }

    std::optional<std::pair<geometry::SurfaceInteraction<T>, T>> intersect_impl(
        const geometry::Ray<T, 3>& ray
    ) const {
        auto result_opt = is_intersected(ray);
        if (!result_opt.has_value()) {
            return std::nullopt;
        }

        auto intersection = get_surface_interaction(ray, result_opt.value());
        return std::make_optional(std::make_pair(intersection, result_opt->t_hit));
    }

    geometry::SurfaceInteraction<T> get_surface_interaction(
        const geometry::Ray<T, 3>& ray,
        const IntersectionResult<T>& intersection
    ) const {
        T u = intersection.phi / m_phi_max;
        T cos_theta = intersection.p_hit.z() / m_radius;
        T theta = std::acos(std::clamp(cos_theta, static_cast<T>(-1), static_cast<T>(1)));
        T v = (theta - z_min_theta()) / (z_max_theta() - z_min_theta());

        // Compute sphere dpdu and dpdv
        math::Vector<T, 3> dpdu(
            -m_phi_max * intersection.p_hit.y(),
            m_phi_max * intersection.p_hit.x(),
            static_cast<T>(0)
        );
        T sin_theta = std::sqrt(1.0 - cos_theta * cos_theta);
        T z_radius = std::sqrt(
            intersection.p_hit.x() * intersection.p_hit.x() + 
            intersection.p_hit.y() * intersection.p_hit.y()
        );
        T cos_phi{}, sin_phi{};
        bool is_polar{};
        if (std::abs(z_radius) <= epsilon_v<T>) {
            cos_phi = 1;
            sin_phi = 0;
            is_polar = true;
        } else {
            cos_phi = intersection.p_hit.x() / z_radius;
            sin_phi = intersection.p_hit.y() / z_radius;
            is_polar = false;
        }

        math::Vector<T, 3> dpdv(
            intersection.p_hit.z() * cos_phi,
            intersection.p_hit.z() * sin_phi,
            -m_radius * sin_theta
        );
        dpdv = dpdv * (z_max_theta() - z_min_theta());

        // Compute sphere d2pduu, d2pdudv, d2pdvv
        auto d2pduu = -m_phi_max * m_phi_max * math::Vector<T, 3>(
            intersection.p_hit.x(),
            intersection.p_hit.y(),
            static_cast<T>(0)
        );

        auto d2pdudv = (z_max_theta() - z_min_theta()) * intersection.p_hit.z() * m_phi_max * math::Vector<T, 3>(
            -sin_phi,
            cos_phi,
            static_cast<T>(0)
        );
        auto d2pdvv = -std::pow(z_max_theta() - z_min_theta(), 2) * math::Vector<T, 3>(
            intersection.p_hit.x(),
            intersection.p_hit.y(),
            intersection.p_hit.z()
        );

        math::Vector<T, 3> N{};

        // Compute fundamental forms
        T E = dpdu.dot(dpdu);
        T F = dpdu.dot(dpdv);
        T G = dpdv.dot(dpdv);
        if (!is_polar) {
            N = cross(dpdu, dpdv).normalized();
        } else {
            auto n = (intersection.p_hit - math::Point<T, 3>(0, 0, 0));
            N = n.normalized();
        }
        
        T e = N.dot(d2pduu);
        T f = N.dot(d2pdudv);
        T g = N.dot(d2pdvv);

        T EGF2 = E * G - F * F;
        T inv_EGF2 = (EGF2 == static_cast<T>(0)) ? static_cast<T>(0) : static_cast<T>(1) / EGF2;
        math::Vector<T, 3> dndu = ((f * F - e * G) * inv_EGF2) * dpdu + ((e * F - f * E) * inv_EGF2) * dpdv;
        math::Vector<T, 3> dndv = ((g * F - f * G) * inv_EGF2) * dpdu + ((f * F - g * E) * inv_EGF2) * dpdv;
        
        math::Vector<T, 3> p_error = math::gamma<T>(5) * math::Vector<T, 3>(
            std::abs(intersection.p_hit.x()),
            std::abs(intersection.p_hit.y()),
            std::abs(intersection.p_hit.z())
        );

        return geometry::SurfaceInteraction<T>(
            intersection.p_hit - p_error,
            intersection.p_hit + p_error,
            -ray.direction().normalized(),
            math::Normal<T, 3>(N),
            math::Point<T, 2>(u, v),
            dpdu,
            dpdv,
            math::Normal<T, 3>(dndu),
            math::Normal<T, 3>(dndv)
        ); 
    }

    /// Surface area of the (partial) sphere.
    T area_impl() const {
        return m_phi_max * m_radius * (m_z_max - m_z_min);  
    }

    /// Samples a point uniformly on the sphere surface.
    std::optional<ShapeSample<T>> sample_on_shape_impl(
        const math::Point<T, 2>& u_sample
    ) const {
        math::Point<T, 3> p = math::sample_uniform_sphere(u_sample, m_radius);
        if (p.z() < m_z_min || p.z() > m_z_max) {
            return std::nullopt;
        }
        math::Vector<T, 3> p_error = math::gamma<T>(5) * math::Vector<T, 3>(
            std::abs(p.x()),
            std::abs(p.y()),
            std::abs(p.z())
        );

        math::Normal<T, 3> n = math::Normal<T, 3>::from_vector(
            (p - math::Point<T, 3>(0, 0, 0)).normalized()
        );

        T theta = std::acos(std::clamp(p.z() / m_radius, static_cast<T>(-1), static_cast<T>(1)));
        T phi = std::atan2(p.y(), p.x());
        if (phi < static_cast<T>(0)) {
            phi += static_cast<T>(2 * math::pi_v<T>);
        }

        if (phi > m_phi_max) {
            return std::nullopt;
        }

        T u = phi / m_phi_max;
        T v = (theta - z_min_theta()) / (z_max_theta() - z_min_theta());
        ShapeSample<T> sample{};
        sample.point = p;
        sample.normal = n;
        sample.uv = math::Point<T, 2>(u, v);
        sample.pdf = static_cast<T>(1) / area_impl();
        return std::make_optional(sample);
    }

    

};

};
