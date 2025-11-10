#pragma once

#include <optional>
#include <utility>

#include "geometry/bounds.hpp"
#include "geometry/directional_cone.hpp"
#include "geometry/interaction.hpp"
#include "geometry/ray.hpp"
#include "geometry/transform.hpp"

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

    std::optional<T> is_intersected(const geometry::Ray<T, 3>& ray) const {
        return as_derived().is_intersected_impl(ray);
    }

    std::optional<std::pair<geometry::SurfaceInteraction<T>, T>> intersect(
        const geometry::Ray<T, 3>& ray
    ) const {
        return as_derived().intersect_impl(ray);
    }
};

template<typename T, template<typename> class ShapeType>
class TransformedShape : public Shape<TransformedShape<T, ShapeType>, T> {
    friend class Shape<TransformedShape<T, ShapeType>, T>;

private:
    ShapeType<T> m_shape;
    geometry::Transform<T> m_render_to_object{};
    geometry::Transform<T> m_object_to_render{};

public:
    TransformedShape(
        const ShapeType<T>& shape,
        const geometry::Transform<T>& render_to_object
    ) : m_shape(shape), m_render_to_object(render_to_object) {
        m_object_to_render = render_to_object.inversed();
    }

    const geometry::Transform<T>& render_to_object_transform() const {
        return m_render_to_object;
    }

    const geometry::Transform<T>& object_to_render_transform() const {
        return m_object_to_render;
    }

    void update_transform(const geometry::Transform<T>& new_render_to_object) {
        m_render_to_object = new_render_to_object;
        m_object_to_render = new_render_to_object.inversed();
    }

private:
    geometry::Bounds<T, 3> bounding_box_impl() const {
        auto bbox = m_shape.bounding_box();
        return m_object_to_render.transform_bounds(bbox);
    }

    geometry::DirectionalCone<T> normal_bounding_cone_impl() const {
        return m_shape.normal_bounding_cone();
    }

    std::optional<T> is_intersected_impl(const geometry::Ray<T, 3>& ray) const {
        auto ray_object = m_render_to_object.transform_ray(ray);
        const Shape<ShapeType<T>, T>& shape_iface = m_shape;
        return shape_iface.is_intersected(ray_object);
    }

    std::optional<std::pair<geometry::SurfaceInteraction<T>, T>> intersect_impl(const geometry::Ray<T, 3>& ray) const {
        auto ray_object = m_render_to_object.transform_ray(ray);

        const Shape<ShapeType<T>, T>& shape_iface = m_shape;
        auto result = shape_iface.intersect(ray_object);
        if (!result.has_value())
            return std::nullopt;

        auto [si_object, t_hit] = result.value();
        auto si_render = m_object_to_render.transform_surface_interaction(si_object);
        return std::make_optional(std::make_pair(si_render, t_hit));
    }

};

};
