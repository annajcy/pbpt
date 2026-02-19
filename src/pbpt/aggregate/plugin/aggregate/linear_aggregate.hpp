#pragma once

#include <limits>
#include <optional>
#include <vector>

#include "pbpt/aggregate/aggregate.hpp"

namespace pbpt::aggregate {

template <typename T>
class LinearAggregate : public Aggregate<LinearAggregate<T>, T> {
    friend class Aggregate<LinearAggregate<T>, T>;

private:
    std::vector<shape::Primitive<T>> m_primitives;

public:
    LinearAggregate() = default;
    LinearAggregate(const std::vector<shape::Primitive<T>>& primitives) : m_primitives(primitives) {}

private:
    std::optional<shape::PrimitiveIntersectionRecord<T>> intersect_ray_impl(const geometry::Ray<T, 3>& ray) const {
        std::optional<shape::PrimitiveIntersectionRecord<T>> closest_hit;
        T closest_t = std::numeric_limits<T>::infinity();

        for (const auto& primitive : m_primitives) {
            auto hit = primitive.intersect_ray(ray);
            if (hit && hit->intersection.t < closest_t) {
                closest_t = hit->intersection.t;
                closest_hit = hit;
            }
        }

        return closest_hit;
    }

    std::optional<shape::PrimitiveIntersectionRecord<T>> intersect_ray_differential_impl(
        const geometry::RayDifferential<T, 3>& ray_diff) const {
        std::optional<shape::PrimitiveIntersectionRecord<T>> closest_hit;
        T closest_t = std::numeric_limits<T>::infinity();

        for (const auto& primitive : m_primitives) {
            auto hit = primitive.intersect_ray_differential(ray_diff);
            if (hit && hit->intersection.t < closest_t) {
                closest_t = hit->intersection.t;
                closest_hit = hit;
            }
        }

        return closest_hit;
    }

    std::optional<T> is_intersected_ray_impl(const geometry::Ray<T, 3>& ray) const {
        T closest_t = std::numeric_limits<T>::infinity();
        bool hit_found = false;

        for (const auto& primitive : m_primitives) {
            auto hit_t = primitive.is_intersected_ray(ray);
            if (hit_t && *hit_t < closest_t) {
                closest_t = *hit_t;
                hit_found = true;
            }
        }

        if (hit_found) {
            return std::make_optional(closest_t);
        } else {
            return std::nullopt;
        }
    }
};

}  // namespace pbpt::aggregate
