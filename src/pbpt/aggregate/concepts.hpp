#pragma once

#include "pbpt/geometry/ray.hpp"

namespace pbpt::aggregate {

template <typename AggregateT, typename T>
concept AggregateIntersectConcept = requires(const AggregateT& aggregate, const geometry::Ray<T, 3>& ray,
                                            const geometry::RayDifferential<T, 3>& ray_diff) {
    aggregate.intersect_ray(ray);
    aggregate.intersect_ray_differential(ray_diff);
};

}  // namespace pbpt::aggregate
