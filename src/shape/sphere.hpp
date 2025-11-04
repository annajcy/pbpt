#pragma once

#include <algorithm>
#include <optional>
#include <utility>

#include "geometry/bounds.hpp"
#include "geometry/directional_cone.hpp"
#include "geometry/interaction.hpp"
#include "geometry/ray.hpp"
#include "geometry/transform.hpp"
#include "math/function.hpp"
#include "math/point.hpp"

#include "shape/shape.hpp"

namespace pbpt::shape {

template<typename T>
class Sphere : public Shape<Sphere<T>, T> {
    friend class Shape<Sphere<T>, T>;

public:
    template<typename U>
    struct IntersectionResult {
        math::Point<U, 3> p_hit;
        U t_hit;
        U phi;
    };

private:
    T m_radius{};
    T m_z_min{};
    T m_z_max{};
    T m_phi_max{};

public:
    Sphere(T radius) : m_radius(radius) {
        m_z_min = -radius;
        m_z_max = radius;
        m_phi_max = static_cast<T>(2 * math::pi_v<T>);
    }

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

};

};
