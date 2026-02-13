/**
 * @file
 * @brief Core camera interface and common sampling structures.
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <string>

#include "pbpt/geometry/ray.hpp"

#include "pbpt/geometry/ray.hpp"
#include "pbpt/geometry/transform.hpp"
#include "pbpt/geometry/spherical.hpp"

#include "pbpt/math/point.hpp"
#include "pbpt/math/vector.hpp"

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



/**
 * @brief Camera-space projection utilities.
 *
 * Encapsulates the standard graphics pipeline transforms:
 * camera space -> clip space -> viewport (raster) space. It also provides
 * helper factories for orthographic and perspective projections.
 *
 * @tparam T Scalar type (e.g. float or double).
 */
template<typename T>
class CameraProjection {
private:
    /// Transform from clip space to viewport (raster) coordinates.
    geometry::Transform<T> m_clip_to_viewport{};
    /// Transform from camera space to clip space.
    geometry::Transform<T> m_camera_to_clip{};
    /// Precomputed transform from camera space directly to viewport space.
    geometry::Transform<T> m_camera_to_viewport{};

public:
    CameraProjection() = default;
    
    /**
     * @brief Construct a projection from camera-to-clip and clip-to-viewport.
     *
     * The camera-to-viewport transform is computed as
     * `clip_to_viewport * camera_to_clip`.
     *
     * @param camera_to_clip   Transform from camera space to clip space.
     * @param clip_to_viewport Transform from clip space to viewport space.
     */
    CameraProjection(
        const geometry::Transform<T>& camera_to_clip, 
        const geometry::Transform<T>& clip_to_viewport
    ) : m_clip_to_viewport(clip_to_viewport), m_camera_to_clip(camera_to_clip) {
        m_camera_to_viewport = m_clip_to_viewport * m_camera_to_clip;
    }

    /// Get the camera-to-clip transform.
    geometry::Transform<T> camera_to_clip() const {
        return m_camera_to_clip;
    }

    /// Get the clip-to-viewport transform.
    geometry::Transform<T> clip_to_viewport() const {
        return m_clip_to_viewport;
    }

    /// Get the camera-to-viewport transform.
    geometry::Transform<T> camera_to_viewport() const {
        return m_camera_to_viewport;
    }

    /// Get the inverse transform: viewport-to-clip.
    geometry::Transform<T> viewport_to_clip() const {
        return m_clip_to_viewport.inversed();
    }

    /// Get the inverse transform: clip-to-camera.
    geometry::Transform<T> clip_to_camera() const {
        return m_camera_to_clip.inversed();
    }

    /// Get the inverse transform: viewport-to-camera.
    geometry::Transform<T> viewport_to_camera() const {
        return m_camera_to_viewport.inversed();
    }

    /**
     * @brief Map a 2D viewport coordinate into camera space at z = 0.
     *
     * @param p 2D viewport coordinate.
     * @return 3D point in camera space on the z = 0 plane.
     */
    math::Point<T, 3> apply_viewport_to_camera(const math::Point<T, 2>& p, T viewport_depth = T{0.0}) const {
        math::Homogeneous<T, 4> hp = math::Homogeneous<T, 4>::from_point(math::Point<T, 3>(p.x(), p.y(), viewport_depth));
        auto hc = viewport_to_camera().transform_homogeneous(hp);
        return hc.to_point();
    }

    /**
     * @brief Build an orthographic projection.
     *
     * The view volume is defined by left/right, bottom/top and near/far
     * planes in camera space, and the viewport transform maps the clip
     * cube to a raster of given width and height.
     *
     * @param left   Left plane in camera x.
     * @param right  Right plane in camera x.
     * @param bottom Bottom plane in camera y.
     * @param top    Top plane in camera y.
     * @param near   Near plane in camera z.
     * @param far    Far plane in camera z.
     * @param width  Viewport width in pixels.
     * @param height Viewport height in pixels.
     * @return CameraProjection representing this orthographic setup.
     */
    static CameraProjection<T> orthographic(
        T left, T right, T bottom, T top, T near, T far, 
        T width, T height
    ) {
        geometry::Transform<T> camera_to_clip = geometry::Transform<T>::orthographic(left, right, bottom, top, near, far);
        geometry::Transform<T> clip_to_viewport = geometry::Transform<T>::viewport(width, height);
        return CameraProjection<T>(camera_to_clip, clip_to_viewport);
    }

    /**
     * @brief Build a perspective projection.
     *
     * This uses a vertical field of view `fov_y_rad` (in radians),
     * aspect ratio, and near/far clipping planes, followed by a viewport
     * transform to raster coordinates.
     *
     * @param fov_y_rad Vertical field of view in radians.
     * @param aspect_xy Aspect ratio (width / height).
     * @param near      Near plane in camera z.
     * @param far       Far plane in camera z.
     * @param width     Viewport width in pixels.
     * @param height    Viewport height in pixels.
     * @return CameraProjection representing this perspective setup.
     */
    static CameraProjection<T> perspective(
        T fov_y_rad, T aspect_xy, T near, T far, 
        T width, T height
    ) {
        geometry::Transform<T> camera_to_clip = geometry::Transform<T>::perspective(fov_y_rad, aspect_xy, near, far);
        geometry::Transform<T> clip_to_viewport = geometry::Transform<T>::viewport(width, height);
        return CameraProjection<T>(camera_to_clip, clip_to_viewport);
    }

    /**
    * @brief Calculate physical film size from FOV, resolution, and near plane.
    *
    * This computes the physical dimensions of the film based on the field of view,
    * which axis it applies to, and the clipping plane distance.
    *
    * @tparam T Scalar type
    * @param fov_degrees Field of view in degrees
    * @param fov_axis Which axis FOV applies to: "x", "y", "smaller", "larger"
    * @param near_clip Near clipping plane distance
    * @param width Film width in pixels
    * @param height Film height in pixels
    * @return Physical film size (width, height)
    */
    static math::Vector<T, 2> calculate_physical_film_size(
        T fov_degrees, const std::string& fov_axis,
        T near_clip, int width, int height
    ) {
        T fov_rad = math::deg2rad(fov_degrees);
        T aspect = T(width) / T(height);
        const T near_distance = std::abs(near_clip);
        
        // Calculate the physical size of the dimension to which FOV applies
        // near_clip may be negative in a right-handed -Z-forward camera setup.
        // Film dimensions must stay positive; using signed near flips the film
        // and mirrors the rendered image horizontally/vertically.
        T physical_size_for_fov = 2 * near_distance * std::tan(fov_rad / 2);
        
        T physical_width, physical_height;
        
        if (fov_axis == "smaller") {
            // FOV applies to the smaller dimension
            if (width <= height) {
                // Width is smaller or equal
                physical_width = physical_size_for_fov;
                physical_height = physical_size_for_fov / aspect;
            } else {
                // Height is smaller
                physical_height = physical_size_for_fov;
                physical_width = physical_size_for_fov * aspect;
            }
        } else if (fov_axis == "larger") {
            // FOV applies to the larger dimension
            if (width >= height) {
                // Width is larger or equal
                physical_width = physical_size_for_fov;
                physical_height = physical_size_for_fov / aspect;
            } else {
                // Height is larger
                physical_height = physical_size_for_fov;
                physical_width = physical_size_for_fov * aspect;
            }
        } else if (fov_axis == "x") {
            // FOV applies to horizontal (width)
            physical_width = physical_size_for_fov;
            physical_height = physical_size_for_fov / aspect;
        } else if (fov_axis == "y") {
            // FOV applies to vertical (height)
            physical_height = physical_size_for_fov;
            physical_width = physical_size_for_fov * aspect;
        } else {
            // Default to "y" axis (most common in computer graphics)
            physical_height = physical_size_for_fov;
            physical_width = physical_size_for_fov * aspect;
        }
        
        return math::Vector<T, 2>(physical_width, physical_height);
    }

    /**
    * @brief Helper for creating an orthographic projection from film parameters.
    *
    * The physical film size defines the extents in x and y; near/far specify
    * depth range, and `film_resolution` gives the raster size.
    */
  
    static CameraProjection<T> create_orthographic_projection(
        const math::Vector<int, 2>& film_resolution,
        const math::Vector<T, 2>& film_physical_size,
        T near, T far
    ) {
        return CameraProjection<T>::orthographic(
            -film_physical_size.x() / 2,
            film_physical_size.x() / 2,
            -film_physical_size.y() / 2,
            film_physical_size.y() / 2,
            near,
            far,
            film_resolution.x(),
            film_resolution.y()
        );
    }

    /**
    * @brief Helper for creating a perspective projection from film parameters.
    *
    * The physical film size defines the frustum at the near plane; near/far
    * specify depth range, and `film_resolution` gives the raster size.
    */
    static CameraProjection<T> create_orthographic_projection_by_fov(
        const math::Vector<int, 2>& film_resolution,
        T fov, const std::string& fov_axis,
        T near, T far
    ) {
        auto film_size = calculate_physical_film_size(
            fov, fov_axis, near, 
            film_resolution.x(), film_resolution.y()
        );
        return create_orthographic_projection(
            film_resolution,
            film_size,
            near, far
        );
    }

    /**
    * @brief Helper for creating a perspective projection from film parameters.
    *
    * The vertical field of view is computed from the film height and
    * near plane distance so that the film edges align with the frustum
    * at the near plane.
    */

    static CameraProjection<T> create_perspective_projection(
        const math::Vector<int, 2>& film_resolution,
        const math::Vector<T, 2>& film_physical_size,
        T near, T far
    ) {
        T aspect = film_physical_size.x() / film_physical_size.y();
        T fov_y = 2 * std::atan2(film_physical_size.y() / 2, near);
        return CameraProjection<T>::perspective(
            fov_y, aspect,
            near, far,
            film_resolution.x(), film_resolution.y()
        );
    }

    /**
    * @brief Helper for creating a perspective projection from FOV and film parameters.
    *
    * The physical film size is computed from the given field of view,
    * aspect ratio, and near plane distance.
    */
    static CameraProjection<T> create_perspective_projection_by_fov(
        const math::Vector<int, 2>& film_resolution,
        T fov_degrees, const std::string& fov_axis,
        T near, T far
    ) {
        auto film_size = calculate_physical_film_size(
            fov_degrees, fov_axis, near, 
            film_resolution.x(), film_resolution.y()
        );
        return create_perspective_projection(
            film_resolution,
            film_size,
            near, far
        );
    }
};


/**
 * @brief Base class for projective cameras (orthographic, perspective).
 *
 * This class combines the generic `Camera` interface with a
 * `CameraProjection` object that defines how camera space maps to
 * clip space and viewport space.
 *
 * @tparam Derived Concrete camera type.
 * @tparam T       Scalar type (e.g. float or double).
 */
template<typename Derived, typename T>
class ProjectiveCamera : public Camera<Derived, T> {
    friend class Camera<Derived, T>;
protected:
    /// Projection used by this camera.
    CameraProjection<T> m_projection{};

public:
    ProjectiveCamera() = default;
    
    /**
     * @brief Construct a projective camera with the given projection.
     *
     * @param projection CameraProjection defining the projection.
     */
    ProjectiveCamera(const CameraProjection<T>& projection) : m_projection(projection) {}

    /// Get the camera projection.
    const CameraProjection<T>& projection() const {
        return m_projection;
    }

protected:
    /**
     * @brief Implementation of film resolution query.
     *
     * The resolution is derived from the viewport transform, assuming
     * that the viewport matrix scales the clip space cube to pixel
     * coordinates. The width and height are inferred from the diagonal
     * elements of the viewport transform.
     *
     * @return 2D vector (width, height) in pixels.
     */
    math::Vector<int, 2> film_resolution_impl() const {
        const auto viewport = m_projection.clip_to_viewport();
        const auto& mat = viewport.matrix();
        auto width_value = mat.at(0, 0) * T(2);
        auto height_value = mat.at(1, 1) * T(2);
        const int width = std::max(1, static_cast<int>(std::lround(static_cast<double>(width_value))));
        const int height = std::max(1, static_cast<int>(std::lround(static_cast<double>(height_value))));
        return math::Vector<int, 2>(width, height);
    }
};

} // namespace pbpt::camera
