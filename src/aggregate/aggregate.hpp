#pragma once

#include <optional>

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

}  // namespace pbpt::aggregate
