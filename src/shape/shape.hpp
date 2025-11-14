#pragma once

#include <optional>
#include <utility>

#include "geometry/bounds.hpp"
#include "geometry/directional_cone.hpp"
#include "geometry/interaction.hpp"
#include "geometry/ray.hpp"
#include "geometry/transform.hpp"
#include "math/normal.hpp"
#include "math/point.hpp"

namespace pbpt::shape {

template<typename T>
struct ShapeSample {
    math::Point<T, 3> point;
    math::Normal<T, 3> normal;
    math::Point<T, 2> uv;
    T pdf{};
};

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

    std::optional<ShapeSample<T>> sample_on_shape(
        const math::Point<T, 2>& u_sample
    ) const {
        return as_derived().sample_on_shape_impl(u_sample);
    }

    std::optional<ShapeSample<T>> sample_on_solid_angle(
        const math::Point<T, 3>& reference,
        const math::Point<T, 2>& u_sample
    ) const {
        return as_derived().sample_on_solid_angle_impl(reference, u_sample);
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
        const geometry::Transform<T>& object_to_render
    ) : m_shape(shape), m_object_to_render(object_to_render), m_render_to_object(object_to_render.inversed()) {}

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

    T area_impl() const {
        return m_shape.area();
    }

    std::optional<ShapeSample<T>> sample_on_shape_impl(
        const math::Point<T, 2>& u_sample
    ) const {
        const Shape<ShapeType<T>, T>& shape_iface = m_shape;
        auto sample_opt = shape_iface.sample_on_shape(u_sample);
        if (!sample_opt.has_value())
            return std::nullopt;

        auto sample_object = sample_opt.value();
        auto point_render = m_object_to_render.transform_point(sample_object.point);
        auto normal_render = m_object_to_render.transform_normal(sample_object.normal).normalized();

        return std::make_optional(ShapeSample<T>{
            point_render,
            normal_render,
            sample_object.uv,
            sample_object.pdf
        });
    }

    std::optional<ShapeSample<T>> sample_on_solid_angle_impl(
        const math::Point<T, 3>& reference,
        const math::Point<T, 2>& u_sample
    ) const {
        const Shape<ShapeType<T>, T>& shape_iface = m_shape;
        auto sample_opt = shape_iface.sample_on_solid_angle(
            m_render_to_object.transform_point(reference),
            u_sample
        );
        if (!sample_opt.has_value())
            return std::nullopt;

        auto sample_object = sample_opt.value();
        auto point_render = m_object_to_render.transform_point(sample_object.point);
        auto normal_render = m_object_to_render.transform_normal(sample_object.normal).normalized();

        return std::make_optional(ShapeSample<T>{
            point_render,
            normal_render,
            sample_object.uv,
            sample_object.pdf
        });
    }
};

};
