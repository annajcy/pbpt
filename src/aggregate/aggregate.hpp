#pragma once

#include <optional>

#include "pbpt/shape/primitive.hpp"

namespace pbpt::aggregate {

template<typename Derived, typename T>
class Aggregate {
public:
    std::optional<shape::PrimitiveIntersectionRecord<T>> intersect_ray(const geometry::Ray<T, 3>& ray) const {
        return as_derived().intersect_ray_impl(ray);
    }

    std::optional<shape::PrimitiveIntersectionRecord<T>> intersect_ray_differential(const geometry::RayDifferential<T, 3>& ray_diff) const {
        return as_derived().intersect_ray_differential_impl(ray_diff);
    }

    std::optional<T> is_intersected_ray(const geometry::Ray<T, 3>& ray) const {
        return as_derived().is_intersected_ray_impl(ray);
    }

    Derived& as_derived() {
        return static_cast<Derived&>(*this);
    }

    const Derived& as_derived() const {
        return static_cast<const Derived&>(*this);
    }
};

}  // namespace pbpt::aggregate
