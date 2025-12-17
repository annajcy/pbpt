/**
 * @file
 * @brief Core camera interface and common sampling structures.
 */
#pragma once

#include "geometry/ray.hpp"

namespace pbpt::camera {

/**
 * @brief Sample on the film plane and (optionally) the lens.
 *
 * This structure encodes where a ray should intersect the virtual film
 * (p_film) in raster coordinates and, for thin-lens cameras, where on
 * the lens aperture the ray should originate (p_lens).
 *
 * @tparam T Scalar type (e.g. float or double).
 */
template<typename T>
struct CameraSample{
    /// Film sample position in raster or viewport space.
    math::Point<T, 2> p_film{};
    /// Lens sample position for thin-lens cameras (ignored for pinhole).
    ///
    /// Typically this is a point in a 2D unit disk (later scaled by the
    /// lens radius) used to simulate finite aperture and depth of field.
    math::Point<T, 2> p_lens{};

    /**
     * @brief Create a camera sample for a pinhole camera.
     *
     * The lens point is fixed at the origin and is ignored by pinhole
     * camera models.
     *
     * @param p_film Film sample position.
     * @return CameraSample with `p_lens` set to (0, 0).
     */
    static CameraSample<T> create_pinhole_sample(const math::Point<T, 2>& p_film) {
        return CameraSample<T>{p_film, math::Point<T, 2>(0, 0)};
    }

    /**
     * @brief Create a camera sample for a thin-lens camera.
     *
     * Both film and lens sample positions are specified, allowing thin-lens
     * camera models to compute rays that correctly simulate depth of field.
     *
     * @param p_film Film sample position.
     * @param p_lens Lens sample position.
     * @return CameraSample with both film and lens positions set.
     */
    static CameraSample<T> create_thinlens_sample(const math::Point<T, 2>& p_film, const math::Point<T, 2>& p_lens) {
        return CameraSample<T>{p_film, p_lens};
    }
};

/**
 * @brief CRTP base class for camera models.
 *
 * Concrete camera implementations derive from this template and provide
 * three implementation methods:
 * - `generate_ray_impl` to generate a primary ray.
 * - `generate_differential_ray_impl` to generate ray differentials for
 *   texture filtering and derivatives.
 * - `film_resolution_impl` to report the film resolution in pixels.
 *
 * The base class forwards calls to these methods using CRTP.
 *
 * @tparam Derived Concrete camera type.
 * @tparam T       Scalar type (e.g. float or double).
 */
template<typename Derived, typename T>
class Camera {
public:
    /**
     * @brief Generate a primary camera ray from a sample.
     *
     * @param sample Film and lens sample positions.
     * @return Primary ray in camera space.
     */
    geometry::Ray<T, 3> generate_ray(const CameraSample<T>& sample) const {
        return as_derived().generate_ray_impl(sample);
    }

    /**
     * @brief Generate a ray with associated differentials.
     *
     * Ray differentials are used to approximate how rays change with
     * small perturbations in the film plane, which is important for
     * texture filtering and anti-aliasing.
     *
     * @param sample Film and lens sample positions.
     * @return Ray with differentials in camera space.
     */
    geometry::RayDifferential<T, 3> generate_differential_ray(const CameraSample<T>& sample) const {
        return as_derived().generate_differential_ray_impl(sample);
    }

    /**
     * @brief Get the film resolution in pixels.
     *
     * @return 2D vector (width, height) in pixels.
     */
    math::Vector<int, 2> film_resolution() const {
        return as_derived().film_resolution_impl();
    }

private:
    /// Access the derived implementation (non-const).
    constexpr Derived& as_derived() noexcept {
        return static_cast<Derived&>(*this);
    }

    /// Access the derived implementation (const).
    constexpr const Derived& as_derived() const noexcept {
        return static_cast<const Derived&>(*this);
    }
};

} // namespace pbpt::camera
