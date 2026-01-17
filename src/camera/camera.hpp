/**
 * @file
 * @brief Core camera interface and common sampling structures.
 */
#pragma once

#include "geometry/ray.hpp"

#include "geometry/ray.hpp"
#include "geometry/transform.hpp"
#include "geometry/spherical.hpp"

#include "sampler/2d.hpp"
#include "math/point.hpp"
#include "math/vector.hpp"
#include <type_traits>

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
        
        // Calculate the physical size of the dimension to which FOV applies
        T physical_size_for_fov = 2 * near_clip * std::tan(fov_rad / 2);
        
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
        T fov, const std::string& fov_axis,
        T near, T far
    ) {
        auto film_size = calculate_physical_film_size(
            fov, fov_axis, near, 
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

/**
 * @brief Ideal orthographic (pinhole) camera.
 *
 * Rays are cast with a constant direction along +z and origins located
 * at the back-projected film positions. There is no perspective foreshortening.
 *
 * @tparam T Scalar type (e.g. float or double).
 */
template<typename T>
class OrthographicCamera : public ProjectiveCamera<OrthographicCamera<T>, T> {
    friend class Camera<OrthographicCamera<T>, T>;
    friend class ProjectiveCamera<OrthographicCamera<T>, T>;

public:
    OrthographicCamera() = default;
    OrthographicCamera(const CameraProjection<T>& projection)
        : ProjectiveCamera<OrthographicCamera<T>, T>(projection) {}

    /**
     * @brief Construct an orthographic camera from explicit bounds and resolution.
     *
     * @param left         Left plane in camera x.
     * @param right        Right plane in camera x.
     * @param bottom       Bottom plane in camera y.
     * @param top          Top plane in camera y.
     * @param near         Near plane in camera z.
     * @param far          Far plane in camera z.
     * @param resolution_x Film resolution in x (pixels).
     * @param resolution_y Film resolution in y (pixels).
     */
    OrthographicCamera(
        T left, T right, T bottom, T top, T near, T far, 
        T resolution_x, T resolution_y
    ) : ProjectiveCamera<OrthographicCamera<T>, T>(CameraProjection<T>::orthographic(
        left, right, bottom, top, near, far, 
        resolution_x, resolution_y)
    ) { }

    /**
     * @brief Construct an orthographic camera from film parameters.
     *
     * @param film_resolution   Film resolution (width, height) in pixels.
     * @param film_physical_size Physical film size in x and y.
     * @param near              Near plane in camera z.
     * @param far               Far plane in camera z.
     */
    OrthographicCamera(
        const math::Vector<int, 2>& film_resolution, 
        const math::Vector<T, 2>& film_physical_size,
        T near, T far
    ) : ProjectiveCamera<OrthographicCamera<T>, T>(
        CameraProjection<T>::create_orthographic_projection(
            film_resolution, 
            film_physical_size, 
            near, far
        )
    ) {}

private:
    /// Generate a primary ray for an orthographic camera.
    geometry::Ray<T, 3> generate_ray_impl(const CameraSample<T>& sample) const {
        auto origin = this->m_projection.apply_viewport_to_camera(sample.p_film);
        auto direction = math::Vector<T, 3>(0, 0, -1);
        return geometry::Ray<T, 3>(origin, direction);
    }

    /// Generate ray differentials by perturbing the film sample position.
    geometry::RayDifferential<T, 3> generate_differential_ray_impl(const CameraSample<T>& sample) const {
        geometry::Ray<T, 3> main_ray = this->generate_ray_impl(sample);

        T eps = static_cast<T>(1e-3);
        auto p_film_x = sample.p_film + math::Vector<T, 2>(eps, 0); 
        auto p_film_y = sample.p_film + math::Vector<T, 2>(0, eps);

        CameraSample<T> sample_x = { p_film_x, sample.p_lens };
        CameraSample<T> sample_y = { p_film_y, sample.p_lens };

        geometry::Ray<T, 3> ray_x = this->generate_ray_impl(sample_x);
        geometry::Ray<T, 3> ray_y = this->generate_ray_impl(sample_y);

        return geometry::RayDifferential<T, 3>(main_ray, {ray_x, ray_y});
    }
};

/**
 * @brief Ideal perspective (pinhole) camera.
 *
 * Rays originate at the camera origin and pass through film points
 * back-projected into camera space using the perspective projection.
 *
 * @tparam T Scalar type (e.g. float or double).
 */
template<typename T>
class PerspectiveCamera : public ProjectiveCamera<PerspectiveCamera<T>, T> {
    friend class Camera<PerspectiveCamera<T>, T>;
    friend class ProjectiveCamera<PerspectiveCamera<T>, T>;

public:
    PerspectiveCamera() = default;
    PerspectiveCamera(const CameraProjection<T>& projection)
        : ProjectiveCamera<PerspectiveCamera<T>, T>(projection) {}

    /**
     * @brief Construct a perspective camera from explicit frustum and resolution.
     *
     * @param fov_y_rad    Vertical field of view in radians.
     * @param aspect_xy    Aspect ratio (width / height).
     * @param near         Near plane in camera z.
     * @param far          Far plane in camera z.
     * @param resolution_x Film resolution in x (pixels).
     * @param resolution_y Film resolution in y (pixels).
     */
    PerspectiveCamera(
        const T fov_y_rad,
        const T aspect_xy,
        const T near, const T far,
        const T resolution_x, const T resolution_y
    ) : ProjectiveCamera<PerspectiveCamera<T>, T>(CameraProjection<T>::perspective(
        fov_y_rad, 
        aspect_xy, 
        near, far, 
        resolution_x, resolution_y)
    ) { }

    /**
     * @brief Construct a perspective camera from film parameters.
     *
     * The field of view is derived from the film height and near plane
     * distance so that the film edges align with the frustum at the
     * near plane.
     *
     * @param film_resolution   Film resolution (width, height) in pixels.
     * @param film_physical_size Physical film size in x and y.
     * @param near              Near plane in camera z.
     * @param far               Far plane in camera z.
     */
    PerspectiveCamera(
        const math::Vector<int, 2>& film_resolution, 
        const math::Vector<T, 2>& film_physical_size,
        T near, T far
    ) : ProjectiveCamera<PerspectiveCamera<T>, T>(
        CameraProjection<T>::create_perspective_projection(film_resolution, film_physical_size, near, far)
    ) {}

private:
    /// Generate a primary ray for a perspective camera.
    geometry::Ray<T, 3> generate_ray_impl(const CameraSample<T>& sample) const {
        auto origin = math::Point<T, 3>(0, 0, 0);
        auto p_camera = this->m_projection.apply_viewport_to_camera(sample.p_film);
        return geometry::Ray<T, 3>(origin, p_camera);
    }

    /// Generate ray differentials by perturbing the film sample position.
    geometry::RayDifferential<T, 3> generate_differential_ray_impl(const CameraSample<T>& sample) const {
        
        geometry::Ray<T, 3> main_ray = this->generate_ray_impl(sample);

        T eps = static_cast<T>(1e-3);
        auto p_film_x = sample.p_film + math::Vector<T, 2>(eps, 0); 
        auto p_film_y = sample.p_film + math::Vector<T, 2>(0, eps);

        CameraSample<T> sample_x = { p_film_x, sample.p_lens };
        CameraSample<T> sample_y = { p_film_y, sample.p_lens };

        geometry::Ray<T, 3> ray_x = this->generate_ray_impl(sample_x);
        geometry::Ray<T, 3> ray_y = this->generate_ray_impl(sample_y);

        return geometry::RayDifferential<T, 3>(main_ray, {ray_x, ray_y});
    }
};

// Thin Lens Cameras

/**
 * @brief Orthographic camera with a finite circular aperture (thin lens).
 *
 * Instead of originating rays from a fixed point behind the film, this
 * camera simulates depth of field by sampling points on a circular lens
 * and focusing rays on a plane at a given focal distance.
 *
 * @tparam T Scalar type (e.g. float or double).
 */
template<typename T>
class ThinLensOrthographicCamera : public ProjectiveCamera<ThinLensOrthographicCamera<T>, T> {
    friend class Camera<ThinLensOrthographicCamera<T>, T>;
    friend class ProjectiveCamera<ThinLensOrthographicCamera<T>, T>;

private:
    /// Distance from the lens to the focal plane along +z.
    T m_focal_distance{1};

public:
    ThinLensOrthographicCamera() = default;
    ThinLensOrthographicCamera(const CameraProjection<T>& projection, T lens_radius, T focal_distance)
        : ProjectiveCamera<ThinLensOrthographicCamera<T>, T>(projection), m_focal_distance(focal_distance) {}

    /**
     * @brief Construct a thin-lens orthographic camera from explicit bounds.
     *
     * @param left          Left plane in camera x.
     * @param right         Right plane in camera x.
     * @param bottom        Bottom plane in camera y.
     * @param top           Top plane in camera y.
     * @param near          Near plane in camera z.
     * @param far           Far plane in camera z.
     * @param resolution_x  Film resolution in x (pixels).
     * @param resolution_y  Film resolution in y (pixels).
     * @param lens_radius   Radius of the circular lens aperture.
     * @param focal_distance Distance from lens to focal plane along +z.
     */
    ThinLensOrthographicCamera(
        T left, T right, T bottom, T top, T near, T far, 
        T resolution_x, T resolution_y, 
        T focal_distance
    ) : ProjectiveCamera<ThinLensOrthographicCamera<T>, T>(CameraProjection<T>::orthographic(
        left, right, bottom, top, near, far, 
        resolution_x, resolution_y)
    ), m_focal_distance(focal_distance) { }

    /**
     * @brief Construct a thin-lens orthographic camera from film parameters.
     *
     * @param film_resolution    Film resolution (width, height) in pixels.
     * @param film_physical_size Physical film size in x and y.
     * @param near               Near plane in camera z.
     * @param far                Far plane in camera z.
     * @param lens_radius        Radius of the circular lens aperture.
     * @param focal_distance     Distance from lens to focal plane along +z.
     */
    ThinLensOrthographicCamera(
        const math::Vector<int, 2>& film_resolution, 
        const math::Vector<T, 2>& film_physical_size,
        T near, T far, T focal_distance
    ) : ProjectiveCamera<ThinLensOrthographicCamera<T>, T>(
            CameraProjection<T>::create_orthographic_projection(
                film_resolution, 
                film_physical_size, 
                near, far
            )
        ), m_focal_distance(focal_distance) {}
    
private:
    /**
     * @brief Generate a ray using a thin-lens orthographic model.
     *
     * The algorithm:
     * 1. Cast a pinhole ray from the film point with direction +z.
     * 2. Intersect this ray with a plane at z = focal_distance to find
     *    the focus point.
     * 3. Sample a point on the lens disk and shoot a ray from the lens
     *    point to the focus point.
     */
    geometry::Ray<T, 3> generate_ray_impl(const CameraSample<T>& sample) const {
        // 1. 获取视口映射点 (位于近平面，但这不重要，我们只需要它的 x 和 y)
        auto p_camera = this->m_projection.apply_viewport_to_camera(sample.p_film);

        // 2. [优化] 直接构造焦点
        // 正交投影特性：焦点产生的 X,Y 与胶片点一致。
        // 焦平面深度：严格位于 Z = -m_focal_distance
        auto p_focus = math::Point<T, 3>(p_camera.x(), p_camera.y(), -m_focal_distance);

        // // 3. 采样透镜 (Lens)
        // auto p_lens = sampler::sample_uniform_disk_concentric(sample.p_lens, m_lens_radius);
        auto origin = math::Point<T, 3>(sample.p_lens.x(), sample.p_lens.y(), 0);

        // 4. 生成射线
        auto direction = (p_focus - origin).normalized();
        return geometry::Ray<T, 3>(origin, direction);
    }

    /// Generate ray differentials by perturbing the film sample position.
    geometry::RayDifferential<T, 3> generate_differential_ray_impl(const CameraSample<T>& sample) const {
       
        geometry::Ray<T, 3> main_ray = this->generate_ray_impl(sample);

        T eps = static_cast<T>(1e-3);
        auto p_film_x = sample.p_film + math::Vector<T, 2>(eps, 0); 
        auto p_film_y = sample.p_film + math::Vector<T, 2>(0, eps);

        CameraSample<T> sample_x = { p_film_x, sample.p_lens };
        CameraSample<T> sample_y = { p_film_y, sample.p_lens };

        geometry::Ray<T, 3> ray_x = this->generate_ray_impl(sample_x);
        geometry::Ray<T, 3> ray_y = this->generate_ray_impl(sample_y);

        return geometry::RayDifferential<T, 3>(main_ray, {ray_x, ray_y});
    }
};

/**
 * @brief Perspective camera with a finite circular aperture (thin lens).
 *
 * This model adds depth of field to a pinhole perspective camera by
 * sampling points on a circular lens aperture and focusing on a plane
 * at `focal_distance`.
 *
 * @tparam T Scalar type (e.g. float or double).
 */
template<typename T>
class ThinLensPerspectiveCamera : public ProjectiveCamera<ThinLensPerspectiveCamera<T>, T>{
    friend class Camera<ThinLensPerspectiveCamera<T>, T>;
    friend class ProjectiveCamera<ThinLensPerspectiveCamera<T>, T>;

private:
    /// Distance from the lens to the focal plane along +z.
    T m_focal_distance{1};
    
public:
    ThinLensPerspectiveCamera() = default;
    
    ThinLensPerspectiveCamera(const CameraProjection<T>& projection, T focal_distance)
        : ProjectiveCamera<ThinLensPerspectiveCamera<T>, T>(projection), m_focal_distance(focal_distance) {}

    /**
     * @brief Construct a thin-lens perspective camera from explicit frustum.
     *
     * @param fov_y_rad     Vertical field of view in radians.
     * @param aspect_xy     Aspect ratio (width / height).
     * @param near          Near plane in camera z.
     * @param far           Far plane in camera z.
     * @param resolution_x  Film resolution in x (pixels).
     * @param resolution_y  Film resolution in y (pixels).
     * @param lens_radius   Radius of the circular lens aperture.
     * @param focal_distance Distance from lens to focal plane along +z.
     */
    ThinLensPerspectiveCamera(
        const T fov_y_rad,
        const T aspect_xy,
        const T near, const T far,
        const T resolution_x, const T resolution_y, T focal_distance
    ) : ProjectiveCamera<ThinLensPerspectiveCamera<T>, T>(CameraProjection<T>::perspective(
        fov_y_rad, 
        aspect_xy, 
        near, far, 
        resolution_x, resolution_y)
    ), m_focal_distance(focal_distance) { }


    /**
     * @brief Construct a thin-lens perspective camera from film parameters.
     *
     * The perspective projection is derived from the film size and near
     * plane; depth of field is controlled by lens_radius and focal_distance.
     *
     * @param film_resolution    Film resolution (width, height) in pixels.
     * @param film_physical_size Physical film size in x and y.
     * @param near               Near plane in camera z.
     * @param far                Far plane in camera z.
     * @param lens_radius        Radius of the circular lens aperture.
     * @param focal_distance     Distance from lens to focal plane along +z.
     */
    ThinLensPerspectiveCamera(
        const math::Vector<int, 2>& film_resolution, 
        const math::Vector<T, 2>& film_physical_size,
        T near, T far, T focal_distance
    ) : ProjectiveCamera<ThinLensPerspectiveCamera<T>, T>(
            CameraProjection<T>::create_perspective_projection(film_resolution, film_physical_size, near, far)
        ), m_focal_distance(focal_distance) {}

private:
    /**
     * @brief Generate a ray using a thin-lens perspective model.
     *
     * The algorithm:
     * 1. Compute the pinhole direction from the origin through the
     *    back-projected film point in camera space.
     * 2. Intersect this ray with a plane at z = focal_distance to
     *    obtain the focus point.
     * 3. Sample a point on a disk of radius m_lens_radius in the lens
     *    plane (z = 0) and shoot a ray from that point to the focus point.
     */
    geometry::Ray<T, 3> generate_ray_impl(const CameraSample<T>& sample) const {
        auto p_camera = this->m_projection.apply_viewport_to_camera(sample.p_film);
        //auto p_lens = sampler::sample_uniform_disk_concentric(sample.p_lens, m_lens_radius);
        auto origin = math::Point<T, 3>(sample.p_lens.x(), sample.p_lens.y(), 0);
        auto pinhole_ray = geometry::Ray<T, 3>(math::Point<T, 3>(0, 0, 0), p_camera);
        T t = -m_focal_distance / pinhole_ray.direction().z();
        auto p_focus = pinhole_ray.at(t);
        auto direction = (p_focus - origin).normalized();
        return geometry::Ray<T, 3>(origin, direction);
    }

    /// Generate ray differentials by perturbing the film sample position.
    geometry::RayDifferential<T, 3> generate_differential_ray_impl(const CameraSample<T>& sample) const {
        geometry::Ray<T, 3> main_ray = this->generate_ray_impl(sample);

        T eps = static_cast<T>(1e-3);
        auto p_film_x = sample.p_film + math::Vector<T, 2>(eps, 0); 
        auto p_film_y = sample.p_film + math::Vector<T, 2>(0, eps);

        CameraSample<T> sample_x = { p_film_x, sample.p_lens };
        CameraSample<T> sample_y = { p_film_y, sample.p_lens };

        geometry::Ray<T, 3> ray_x = this->generate_ray_impl(sample_x);
        geometry::Ray<T, 3> ray_y = this->generate_ray_impl(sample_y);

        return geometry::RayDifferential<T, 3>(main_ray, {ray_x, ray_y});
    }
};

/**
 * @brief Mapping used by the spherical camera.
 *
 * - EqualRectangular: image coordinates map linearly to spherical angles
 *   (theta, phi), like a standard latitude-longitude environment map.
 * - EqualArea: image coordinates are warped so that each pixel covers
 *   approximately equal area on the unit sphere.
 */
enum class SphericalCameraMapping {
    EqualRectangular,
    EqualArea
};

/**
 * @brief Spherical camera that generates rays covering the full sphere.
 *
 * The spherical camera maps film coordinates to directions on the unit
 * sphere. Depending on the selected mapping, the sampling may be uniform
 * in angle (equirectangular) or approximately uniform in solid angle
 * (equal-area).
 *
 * @tparam T Scalar type (e.g. float or double).
 */
template<typename T>
class SphericalCamera : public Camera<SphericalCamera<T>, T> {
    friend class Camera<SphericalCamera<T>, T>;

private:
    /// Mapping from film coordinates to directions.
    SphericalCameraMapping m_mapping{SphericalCameraMapping::EqualRectangular};
    /// Film resolution in pixels (width, height).
    math::Vector<int, 2> m_film_resolution{1920, 960};

public:
    /**
     * @brief Construct a spherical camera.
     *
     * @param film_resolution Film resolution (width, height).
     * @param mapping         Mapping from image plane to sphere.
     */
    SphericalCamera(
        const math::Vector<T, 2>& film_resolution,
        SphericalCameraMapping mapping = SphericalCameraMapping::EqualRectangular)
        : m_mapping(mapping), m_film_resolution(film_resolution) {}

    /// Get the current spherical mapping mode.
    SphericalCameraMapping mapping() const {
        return m_mapping;
    }

private:
    /**
     * @brief Generate a ray for the spherical camera.
     *
     * The algorithm:
     * 1. Normalize film coordinates to [0, 1] to obtain uv.
     * 2. For EqualRectangular: convert uv to spherical angles
     *    theta = pi * u, phi = 2 * pi * v, then to a direction vector.
     * 3. For EqualArea: warp uv to an equal-area square and then map to
     *    a direction on the unit sphere.
     * 4. Swap y and z components so that the camera's up and forward
     *    directions match the chosen convention.
     */
    geometry::Ray<T, 3> generate_ray_impl(const CameraSample<T>& sample) const {
        math::Point<T, 2> uv = math::Point<T, 2>(
            sample.p_film.x() / static_cast<T>(m_film_resolution.x()),
            sample.p_film.y() / static_cast<T>(m_film_resolution.y())
        );
        T theta = math::pi_v<T> * uv.x(), phi = 2 * math::pi_v<T> * uv.y();

        math::Vector<T, 3> dir;
        if (m_mapping == SphericalCameraMapping::EqualRectangular) {
            dir = geometry::SphericalPoint<T, 3>(math::Vector<T, 2>{theta, phi}, 1).to_cartesian();
        } else if (m_mapping == SphericalCameraMapping::EqualArea) {
            dir = geometry::equal_area_square_to_sphere(
                geometry::warp_equal_area_square(uv).to_vector()
            );
        }
        std::swap(dir.y(), dir.z());
        return geometry::Ray<T, 3>(math::Point<T, 3>(0, 0, 0), dir);
    }

    /// Generate ray differentials by perturbing the film sample position.
    geometry::RayDifferential<T, 3> generate_differential_ray_impl(const CameraSample<T>& sample) const {
        geometry::Ray<T, 3> main_ray = this->generate_ray_impl(sample);

        T eps = static_cast<T>(1e-3);
        auto p_film_x = sample.p_film + math::Vector<T, 2>(eps, 0); 
        auto p_film_y = sample.p_film + math::Vector<T, 2>(0, eps);

        CameraSample<T> sample_x = { p_film_x, sample.p_lens };
        CameraSample<T> sample_y = { p_film_y, sample.p_lens };

        geometry::Ray<T, 3> ray_x = this->generate_ray_impl(sample_x);
        geometry::Ray<T, 3> ray_y = this->generate_ray_impl(sample_y);

        return geometry::RayDifferential<T, 3>(main_ray, {ray_x, ray_y});
    }

    /// Return the film resolution in pixels.
    math::Vector<int, 2> film_resolution_impl() const {
        return m_film_resolution;
    }
};

} // namespace pbpt::camera
