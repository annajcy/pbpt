#pragma once

#include "geometry/bounds.hpp"
#include "geometry/directional_cone.hpp"

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

    geometry::DirectionalCone<T> normal_bounding_cone() const {
        return as_derived().normal_bounding_cone_impl();
    }

    bool is_intersected(const geometry::Ray<T, 3>& ray, T* t_hit) const {
        return as_derived().is_intersected_impl(ray, t_hit);
    }
};

template<typename T>
class Sphere : public Shape<Sphere<T>, T> {
    friend class Shape<Sphere<T>, T>;

private:
    math::Point<T, 3> m_center;
    T m_radius;

public:
    Sphere(const math::Point<T, 3>& center, T radius) : m_center(center), m_radius(radius) {}
    
};

};