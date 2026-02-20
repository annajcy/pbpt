#pragma once

#include "pbpt/camera/camera.hpp"
#include "pbpt/camera/fov_axis.hpp"
#include "pbpt/camera/plugin/film/film_type.hpp"

namespace pbpt::camera {

/**
 * @brief Ideal orthographic (pinhole) camera.
 *
 * Rays are cast with a constant direction along +z and origins located
 * at the back-projected film positions. There is no perspective foreshortening.
 *
 * @tparam T Scalar type (e.g. float or double).
 */
template <typename T>
class OrthographicCamera : public ProjectiveCamera<OrthographicCamera<T>, T> {
    friend class Camera<OrthographicCamera<T>, T>;
    friend class ProjectiveCamera<OrthographicCamera<T>, T>;

private:
    T m_left{-1};
    T m_right{1};
    T m_bottom{-1};
    T m_top{1};
    T m_near{T(-0.1)};
    T m_far{T(-10000)};
    AnyFilm<T> m_film{};

    CameraProjection<T> get_projection_impl() const {
        return CameraProjection<T>::orthographic(m_left, m_right, m_bottom, m_top, m_near, m_far, T(width()),
                                                 T(height()));
    }

    math::Vector<int, 2> film_resolution_impl() const { return math::Vector<int, 2>(width(), height()); }

public:
    OrthographicCamera() = default;

    /**
     * @brief Construct an orthographic camera from explicit bounds and film.
     */
    OrthographicCamera(AnyFilm<T> film, T left, T right, T bottom, T top, T near, T far)
        : m_left(left),
          m_right(right),
          m_bottom(bottom),
          m_top(top),
          m_near(near),
          m_far(far),
          m_film(std::move(film)) {}

    int width() const {
        return std::visit([](const auto& f) { return f.resolution().x(); }, m_film);
    }
    int height() const {
        return std::visit([](const auto& f) { return f.resolution().y(); }, m_film);
    }

    AnyFilm<T>& film() { return m_film; }
    const AnyFilm<T>& film() const { return m_film; }

private:
    /// Generate a primary ray for an orthographic camera.
    geometry::Ray<T, 3> generate_ray_impl(const CameraSample<T>& sample) const {
        auto proj = this->get_projection();
        auto origin = proj.apply_viewport_to_camera(sample.p_film);
        auto direction = math::Vector<T, 3>(0, 0, -1);
        return geometry::Ray<T, 3>(origin, direction);
    }

    /// Generate ray differentials by perturbing the film sample position.
    geometry::RayDifferential<T, 3> generate_differential_ray_impl(const CameraSample<T>& sample) const {
        geometry::Ray<T, 3> main_ray = this->generate_ray_impl(sample);

        T eps = static_cast<T>(1);
        auto p_film_x = sample.p_film + math::Vector<T, 2>(eps, 0);
        auto p_film_y = sample.p_film + math::Vector<T, 2>(0, eps);

        CameraSample<T> sample_x = {p_film_x, sample.p_lens};
        CameraSample<T> sample_y = {p_film_y, sample.p_lens};

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
template <typename T>
class PerspectiveCamera : public ProjectiveCamera<PerspectiveCamera<T>, T> {
    friend class Camera<PerspectiveCamera<T>, T>;
    friend class ProjectiveCamera<PerspectiveCamera<T>, T>;

private:
    T m_fov_degrees{45};
    FovAxis m_fov_axis{FovAxis::Smaller};
    T m_near{T(-0.1)};
    T m_far{T(-10000)};
    AnyFilm<T> m_film{};

    CameraProjection<T> get_projection_impl() const {
        return CameraProjection<T>::create_perspective_projection_by_fov(
            math::Vector<int, 2>(width(), height()), m_fov_degrees, fov_axis_to_string(m_fov_axis), m_near, m_far);
    }

    math::Vector<int, 2> film_resolution_impl() const { return math::Vector<int, 2>(width(), height()); }

public:
    PerspectiveCamera() = default;

    /**
     * @brief Construct a perspective camera from film and projection params.
     */
    PerspectiveCamera(AnyFilm<T> film, T fov_degrees, FovAxis fov_axis, T near, T far)
        : m_fov_degrees(fov_degrees), m_fov_axis(fov_axis), m_near(near), m_far(far), m_film(std::move(film)) {}

    int width() const {
        return std::visit([](const auto& f) { return f.resolution().x(); }, m_film);
    }
    int height() const {
        return std::visit([](const auto& f) { return f.resolution().y(); }, m_film);
    }

    AnyFilm<T>& film() { return m_film; }
    const AnyFilm<T>& film() const { return m_film; }

private:
    /// Generate a primary ray for a perspective camera.
    geometry::Ray<T, 3> generate_ray_impl(const CameraSample<T>& sample) const {
        auto proj = this->get_projection();
        auto origin = math::Point<T, 3>(0, 0, 0);
        auto p_camera = proj.apply_viewport_to_camera(sample.p_film);
        return geometry::Ray<T, 3>(origin, p_camera);
    }

    /// Generate ray differentials by perturbing the film sample position.
    geometry::RayDifferential<T, 3> generate_differential_ray_impl(const CameraSample<T>& sample) const {
        geometry::Ray<T, 3> main_ray = this->generate_ray_impl(sample);

        T eps = static_cast<T>(1);
        auto p_film_x = sample.p_film + math::Vector<T, 2>(eps, 0);
        auto p_film_y = sample.p_film + math::Vector<T, 2>(0, eps);

        CameraSample<T> sample_x = {p_film_x, sample.p_lens};
        CameraSample<T> sample_y = {p_film_y, sample.p_lens};

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
template <typename T>
class ThinLensOrthographicCamera : public ProjectiveCamera<ThinLensOrthographicCamera<T>, T> {
    friend class Camera<ThinLensOrthographicCamera<T>, T>;
    friend class ProjectiveCamera<ThinLensOrthographicCamera<T>, T>;

private:
    T m_left{-1};
    T m_right{1};
    T m_bottom{-1};
    T m_top{1};
    T m_near{T(-0.1)};
    T m_far{T(-10000)};
    /// Distance from the lens to the focal plane along +z.
    T m_focal_distance{1};
    AnyFilm<T> m_film{};

    CameraProjection<T> get_projection_impl() const {
        return CameraProjection<T>::orthographic(m_left, m_right, m_bottom, m_top, m_near, m_far, T(width()),
                                                 T(height()));
    }

    math::Vector<int, 2> film_resolution_impl() const { return math::Vector<int, 2>(width(), height()); }

public:
    ThinLensOrthographicCamera() = default;

    /**
     * @brief Construct a thin-lens orthographic camera.
     */
    ThinLensOrthographicCamera(AnyFilm<T> film, T left, T right, T bottom, T top, T near, T far, T focal_distance)
        : m_left(left),
          m_right(right),
          m_bottom(bottom),
          m_top(top),
          m_near(near),
          m_far(far),
          m_focal_distance(focal_distance),
          m_film(std::move(film)) {}

    int width() const {
        return std::visit([](const auto& f) { return f.resolution().x(); }, m_film);
    }
    int height() const {
        return std::visit([](const auto& f) { return f.resolution().y(); }, m_film);
    }

    AnyFilm<T>& film() { return m_film; }
    const AnyFilm<T>& film() const { return m_film; }

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
        auto proj = this->get_projection();
        // 1. 获取视口映射点 (位于近平面，但这不重要，我们只需要它的 x 和 y)
        auto p_camera = proj.apply_viewport_to_camera(sample.p_film);

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

        CameraSample<T> sample_x = {p_film_x, sample.p_lens};
        CameraSample<T> sample_y = {p_film_y, sample.p_lens};

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
template <typename T>
class ThinLensPerspectiveCamera : public ProjectiveCamera<ThinLensPerspectiveCamera<T>, T> {
    friend class Camera<ThinLensPerspectiveCamera<T>, T>;
    friend class ProjectiveCamera<ThinLensPerspectiveCamera<T>, T>;

private:
    T m_fov_degrees{45};
    FovAxis m_fov_axis{FovAxis::Smaller};
    T m_near{T(-0.1)};
    T m_far{T(-10000)};
    /// Distance from the lens to the focal plane along +z.
    T m_focal_distance{1};
    AnyFilm<T> m_film{};

    CameraProjection<T> get_projection_impl() const {
        return CameraProjection<T>::create_perspective_projection_by_fov(
            math::Vector<int, 2>(width(), height()), m_fov_degrees, fov_axis_to_string(m_fov_axis), m_near, m_far);
    }

    math::Vector<int, 2> film_resolution_impl() const { return math::Vector<int, 2>(width(), height()); }

public:
    ThinLensPerspectiveCamera() = default;

    /***
     * @brief Construct a thin-lens perspective camera.
     *
     * @param film           AnyFilm<T> (owns width/height).
     * @param fov_degrees    Field of view in degrees.
     * @param fov_axis       Axis to which FOV applies.
     * @param near           Near plane in camera z.
     * @param far            Far plane in camera z.
     * @param focal_distance Distance from lens to focal plane along +z.
     ***/
    ThinLensPerspectiveCamera(AnyFilm<T> film, T fov_degrees, FovAxis fov_axis, T near, T far, T focal_distance)
        : m_fov_degrees(fov_degrees),
          m_fov_axis(fov_axis),
          m_near(near),
          m_far(far),
          m_focal_distance(focal_distance),
          m_film(std::move(film)) {}

    int width() const {
        return std::visit([](const auto& f) { return f.resolution().x(); }, m_film);
    }
    int height() const {
        return std::visit([](const auto& f) { return f.resolution().y(); }, m_film);
    }

    T fov_degrees() const { return m_fov_degrees; }
    FovAxis fov_axis() const { return m_fov_axis; }
    T near_clip() const { return m_near; }
    T far_clip() const { return m_far; }
    T focal_distance() const { return m_focal_distance; }
    AnyFilm<T>& film() { return m_film; }
    const AnyFilm<T>& film() const { return m_film; }

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
        auto proj = this->get_projection();
        auto p_camera = proj.apply_viewport_to_camera(sample.p_film);
        // auto p_lens = sampler::sample_uniform_disk_concentric(sample.p_lens, m_lens_radius);
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

        CameraSample<T> sample_x = {p_film_x, sample.p_lens};
        CameraSample<T> sample_y = {p_film_y, sample.p_lens};

        geometry::Ray<T, 3> ray_x = this->generate_ray_impl(sample_x);
        geometry::Ray<T, 3> ray_y = this->generate_ray_impl(sample_y);

        return geometry::RayDifferential<T, 3>(main_ray, {ray_x, ray_y});
    }
};

}  // namespace pbpt::camera
