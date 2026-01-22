#pragma once

#include <limits>
#include <optional>
#include <vector>

#include "shape/primitive.hpp"

namespace pbpt::aggregate {

template<typename Derived, typename T>
class Aggregate {
public:
    std::optional<shape::PrimitiveIntersectionRecord<T>> intersect(const geometry::Ray<T, 3>& ray) const {
        return as_derived().intersect_impl(ray);
    }

    std::optional<shape::PrimitiveIntersectionRecord<T>> intersect(const geometry::RayDifferential<T, 3>& ray) const {
        return as_derived().intersect_impl(ray);
    }

    std::optional<T> is_intersected(const geometry::Ray<T, 3>& ray) const {
        return as_derived().is_intersected_impl(ray);
    }

    Derived& as_derived() {
        return static_cast<Derived&>(*this);
    }

    const Derived& as_derived() const {
        return static_cast<const Derived&>(*this);
    }
};

template<typename T>
class LinearAggregate : public Aggregate<LinearAggregate<T>, T> {
    friend class Aggregate<LinearAggregate<T>, T>;
private:
    std::vector<shape::Primitive<T>> m_primitives;

public:
    LinearAggregate() = default;
    LinearAggregate(const std::vector<shape::Primitive<T>>& primitives): m_primitives(primitives) {}

private:
    std::optional<shape::PrimitiveIntersectionRecord<T>> intersect_impl(const geometry::Ray<T, 3>& ray) const {
        std::optional<shape::PrimitiveIntersectionRecord<T>> closest_hit;
        T closest_t = std::numeric_limits<T>::infinity();

        for (const auto& primitive : m_primitives) {
            auto hit = primitive.intersect(ray);
            if (hit && hit->intersection.t < closest_t) {
                closest_t = hit->intersection.t;
                closest_hit = hit;
            }
        }

        return closest_hit;
    }

    std::optional<shape::PrimitiveIntersectionRecord<T>> intersect_impl(const geometry::RayDifferential<T, 3>& ray) const {
        std::optional<shape::PrimitiveIntersectionRecord<T>> closest_hit;
        T closest_t = std::numeric_limits<T>::infinity();

        for (const auto& primitive : m_primitives) {
            auto hit = primitive.intersect(ray);
            if (hit && hit->intersection.t < closest_t) {
                closest_t = hit->intersection.t;
                closest_hit = hit;
            }
        }

        return closest_hit;
    }

    std::optional<T> is_intersected_impl(const geometry::Ray<T, 3>& ray) const {
        T closest_t = std::numeric_limits<T>::infinity();
        bool hit_found = false;

        for (const auto& primitive : m_primitives) {
            auto hit_t = primitive.is_intersected(ray);
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
