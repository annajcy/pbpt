#pragma once

#include "geometry/bounds.hpp"

namespace pbpt::shape {

template<typename Derived, typename T>
class Shape {
public:
    constexpr Derived& as_derived() noexcept {
        return static_cast<Derived&>(*this);
    }

    constexpr const Derived& as_derived() const noexcept {
        return static_cast<const Derived&>(*this);
    }

    T area() const {
        return as_derived().area_impl();
    }

    geometry::Bounds<T, 3> bounding_box() const {
        return as_derived().bounding_box_impl();
    }
};

};