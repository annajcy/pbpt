#pragma once

#include <string>

#include "pbpt/camera/camera.hpp"

namespace pbpt::camera {

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

        T eps = static_cast<T>(1);
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

        T eps = static_cast<T>(1);
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

        T eps = static_cast<T>(1);
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

    /***
     * @brief Construct a thin-lens perspective camera from film parameters.
     *
     * The perspective projection is derived from the film size and near
     * plane; depth of field is controlled by lens_radius and focal_distance.
     *
     * @param film_resolution    Film resolution (width, height) in pixels.
     * @param fov_degrees        Field of view in degrees.
     * @param fov_axis           Axis to which FOV applies: "x", "y", "smaller", "larger".
     * @param near               Near plane in camera z.
     * @param far                Far plane in camera z.
     * @param lens_radius        Radius of the circular lens aperture.
     * @param focal_distance     Distance from lens to focal plane along +z.
    ***/
    ThinLensPerspectiveCamera(
        const math::Vector<T, 2>& film_resolution,
        T fov_degrees, const std::string& fov_axis,
        T near, T far, T focal_distance
    ) : ProjectiveCamera<ThinLensPerspectiveCamera<T>, T>(
        CameraProjection<T>::create_perspective_projection_by_fov(
            film_resolution,
            fov_degrees, fov_axis,
            near, far
        )
    ), m_focal_distance(focal_distance) {}

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

        T eps = static_cast<T>(1);
        auto p_film_x = sample.p_film + math::Vector<T, 2>(eps, 0);
        auto p_film_y = sample.p_film + math::Vector<T, 2>(0, eps);

        CameraSample<T> sample_x = { p_film_x, sample.p_lens };
        CameraSample<T> sample_y = { p_film_y, sample.p_lens };

        geometry::Ray<T, 3> ray_x = this->generate_ray_impl(sample_x);
        geometry::Ray<T, 3> ray_y = this->generate_ray_impl(sample_y);

        return geometry::RayDifferential<T, 3>(main_ray, {ray_x, ray_y});
    }
};

} // namespace pbpt::camera
