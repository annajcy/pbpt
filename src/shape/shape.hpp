/**
 * @file shape.hpp
 * @brief Geometric shape interface and transformed shape wrapper.
 * 
 */

#pragma once

#include <optional>

#include "geometry/bounds.hpp"
#include "geometry/directional_cone.hpp"
#include "geometry/interaction.hpp"
#include "geometry/ray.hpp"
#include "geometry/transform.hpp"
#include "math/normal.hpp"
#include "math/point.hpp"
#include "camera/render_transform.hpp"

namespace pbpt::shape {

template<typename T>
struct IntersectionRecord {
    geometry::SurfaceInteraction<T> interaction;
    T t{};
};

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

    geometry::Transform<T> object_to_render_transform() const {
        return as_derived().object_to_render_transform_impl();
    }

    geometry::Transform<T> render_to_object_transform() const {
        return as_derived().render_to_object_transform_impl();
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
    std::optional<IntersectionRecord<T>> intersect(
        const geometry::Ray<T, 3>& ray
    ) const {
        return as_derived().intersect_impl(ray);
    }

    /**
     * @brief Samples a point uniformly (or according to some strategy) on the shape.
     *
     * @param u_sample 2D sample in [0,1]^2.
     * @return Shape sample and its area-domain PDF.
     */
    ShapeSample<T> sample_on_shape(
        const math::Point<T, 2>& u_sample
    ) const {
        return as_derived().sample_on_shape_impl(u_sample);
    }

    /**
     * @brief Computes the PDF for sampling a given surface point on the shape.
     *
     * @param p_surface Surface point in object space.
     * @return PDF value with respect to area measure.
     */
    T sample_on_shape_pdf(
        const math::Point<T, 3>& p_surface
    ) const {
        return as_derived().sample_on_shape_pdf_impl(p_surface);
    }

    /**
     * @brief Samples the shape with respect to solid angle from a reference point.
     *
     * @param reference Reference point in object space (e.g. shading point).
     * @param u_sample  2D sample in [0,1]^2.
     * @return Shape sample and its solid-angle PDF.
     */
    ShapeSample<T> sample_on_solid_angle(
        const math::Point<T, 3>& reference,
        const math::Point<T, 2>& u_sample
    ) const {
        return as_derived().sample_on_solid_angle_impl(reference, u_sample);
    }

    /**
     * @brief Computes the PDF for sampling a given surface point with respect to solid angle.
     *
     * @param reference Reference point in object space (e.g. shading point).
     * @param p_surface Surface point in object space.
     * @return PDF value with respect to solid angle measure.
     */
    T sample_on_solid_angle_pdf(
        const math::Point<T, 3>& reference,
        const math::Point<T, 3>& p_surface
    ) const {
        return as_derived().sample_on_solid_angle_pdf_impl(reference, p_surface);
    }
};

};
