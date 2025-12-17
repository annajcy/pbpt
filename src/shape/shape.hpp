/**
 * @file shape.hpp
 * @brief Geometric shape interface and transformed shape wrapper.
 * 
 */

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
#include "camera/render_transform.hpp"

namespace pbpt::shape {

/**
 * @brief Result of sampling a point on a shape surface.
 *
 * Stores the sampled position, shading normal, texture coordinates
 * and the PDF with respect to the sampling domain (area or solid angle).
 *
 * @tparam T Scalar type.
 */
template<typename T>
struct ShapeSample {
    /// Sampled point on the surface (render space).
    math::Point<T, 3> point;
    /// Outward-facing surface normal at the sampled point.
    math::Normal<T, 3> normal;
    /// Parametric texture coordinates associated with the sample.
    math::Point<T, 2> uv;
    /// Probability density of the sample.
    T pdf{};
};

/**
 * @brief CRTP base class for geometric shapes.
 *
 * A `Shape` exposes a unified interface for area, bounding boxes,
 * normal cones, ray intersection and surface sampling while letting
 * derived shapes implement the actual geometry logic via `*_impl`
 * methods.
 *
 * @tparam Derived Concrete shape type (CRTP).
 * @tparam T       Scalar type.
 */
template<typename Derived, typename T>
class Shape {
public:
    /// Returns a reference to the derived shape (non-const).
    constexpr Derived& as_derived() noexcept {
        return static_cast<Derived&>(*this);
    }

    /// Returns a reference to the derived shape (const).
    constexpr const Derived& as_derived() const noexcept {
        return static_cast<const Derived&>(*this);
    }

    /// Total surface area of the shape.
    T area() const {
        return as_derived().area_impl();
    }

    /// Axis-aligned bounding box of the shape in render space.
    geometry::Bounds<T, 3> bounding_box() const {
        return as_derived().bounding_box_impl();
    }

    /// Bounding cone that contains all surface normals of the shape.
    geometry::DirectionalCone<T> normal_bounding_cone() const {
        return as_derived().normal_bounding_cone_impl();
    }

    /**
     * @brief Tests whether a ray intersects the shape.
     *
     * @return Optional parametric distance t in [t_min, t_max] if an
     *         intersection exists, or std::nullopt otherwise.
     */
    std::optional<T> is_intersected(const geometry::Ray<T, 3>& ray) const {
        return as_derived().is_intersected_impl(ray);
    }

    /**
     * @brief Computes a full surface interaction with the shape.
     *
     * When an intersection exists, returns the surface interaction
     * (position, normal, UV, etc.) together with the hit distance t.
     */
    std::optional<std::pair<geometry::SurfaceInteraction<T>, T>> intersect(
        const geometry::Ray<T, 3>& ray
    ) const {
        return as_derived().intersect_impl(ray);
    }

    /**
     * @brief Samples a point uniformly (or according to some strategy) on the shape.
     *
     * @param u_sample 2D sample in [0,1]^2.
     * @return Optional shape sample and its area-domain PDF.
     */
    std::optional<ShapeSample<T>> sample_on_shape(
        const math::Point<T, 2>& u_sample
    ) const {
        return as_derived().sample_on_shape_impl(u_sample);
    }

    /**
     * @brief Samples the shape with respect to solid angle from a reference point.
     *
     * @param reference Reference point in render space (e.g. shading point).
     * @param u_sample  2D sample in [0,1]^2.
     * @return Optional shape sample and its solid-angle PDF.
     */
    std::optional<ShapeSample<T>> sample_on_solid_angle(
        const math::Point<T, 3>& reference,
        const math::Point<T, 2>& u_sample
    ) const {
        return as_derived().sample_on_solid_angle_impl(reference, u_sample);
    }
};

/**
 * @brief Shape wrapper that applies a rigid transform around another shape.
 *
 * The underlying `ShapeType<T>` is defined in its own object space.
 * `TransformedShape` exposes the same interface but in render space,
 * forwarding all queries and sampling through the stored transforms.
 *
 * @tparam T         Scalar type.
 * @tparam ShapeType Concrete shape template to wrap (e.g. Sphere).
 */
template<typename T, template<typename> class ShapeType>
class TransformedShape : public Shape<TransformedShape<T, ShapeType>, T> {
    friend class Shape<TransformedShape<T, ShapeType>, T>;

private:
    /// Underlying shape defined in object space.
    ShapeType<T> m_shape;
    /// Transform from render space to object space.
    geometry::Transform<T> m_render_to_object{};
    /// Transform from object space to render space.
    geometry::Transform<T> m_object_to_render{};

public:
    /**
     * @brief Wraps an object-space shape with a render-to-object transform.
     *
     * @param shape           Underlying shape in its own object space.
     * @param object_to_render Transform from object space to render space.
     */
    TransformedShape(
        const ShapeType<T>& shape,
        const geometry::Transform<T>& object_to_render
    ) : m_shape(shape), m_object_to_render(object_to_render), m_render_to_object(object_to_render.inversed()) {}

    /**
     * @brief Wraps an object-space shape with a render transform.
     *
     * @param shape            Underlying shape in its own object space.
     * @param object_to_world  Transform from object space to world space.
     * @param render_transform RenderTransform defining render-to-world.
     */
    TransformedShape(
        const ShapeType<T>& shape,
        const geometry::Transform<T>& object_to_world,
        const camera::RenderTransform<T>& render_transform
    ) : m_shape(shape) {
        m_object_to_render = render_transform.world_to_render() * object_to_world;
        m_render_to_object = m_object_to_render.inversed();
    }

    /// Returns the transform from render space to object space.
    const geometry::Transform<T>& render_to_object_transform() const {
        return m_render_to_object;
    }

    /// Returns the transform from object space to render space.
    const geometry::Transform<T>& object_to_render_transform() const {
        return m_object_to_render;
    }

    /**
     * @brief Updates the transforms when a new render-to-object transform is provided.
     *
     * The inverse is recomputed so that both directions stay consistent.
     */
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
