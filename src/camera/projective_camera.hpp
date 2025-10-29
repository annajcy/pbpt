#pragma once

#include <cmath>

#include "geometry/ray.hpp"
#include "geometry/transform.hpp"

#include "math/sampling.hpp"
#include "math/point.hpp"
#include "math/vector.hpp"

#include "camera.hpp"

namespace pbpt::camera {

template<typename T>
class CameraProjection {
private:
    geometry::Transform<T> m_clip_to_viewport{};
    geometry::Transform<T> m_camera_to_clip{};
    geometry::Transform<T> m_camera_to_viewport{};

public:
    CameraProjection(
        const geometry::Transform<T>& camera_to_clip, 
        const geometry::Transform<T>& clip_to_viewport
    ) : m_clip_to_viewport(clip_to_viewport), m_camera_to_clip(camera_to_clip) {
        m_camera_to_viewport = m_clip_to_viewport * m_camera_to_clip;
    }

    geometry::Transform<T> camera_to_clip() const {
        return m_camera_to_clip;
    }

    geometry::Transform<T> clip_to_viewport() const {
        return m_clip_to_viewport;
    }

    geometry::Transform<T> camera_to_viewport() const {
        return m_camera_to_viewport;
    }

    geometry::Transform<T> viewport_to_clip() const {
        return m_clip_to_viewport.inversed();
    }

    geometry::Transform<T> clip_to_camera() const {
        return m_camera_to_clip.inversed();
    }

    geometry::Transform<T> viewport_to_camera() const {
        return m_camera_to_viewport.inversed();
    }

    math::Point<T, 3> apply_viewport_to_camera(const math::Point<T, 2>& p) const {
        math::Homogeneous<T, 4> hp = math::Homogeneous<T, 4>::from_point(math::Point<T, 3>(p.x(), p.y(), 0.0));
        auto hc = viewport_to_camera() * hp;
        return hc.to_point();
    }

    static CameraProjection<T> orthographic(
        T left, T right, T bottom, T top, T near, T far, 
        T width, T height
    ) {
        geometry::Transform<T> camera_to_clip = geometry::Transform<T>::orthographic(left, right, bottom, top, near, far);
        geometry::Transform<T> clip_to_viewport = geometry::Transform<T>::viewport(width, height);
        return CameraProjection<T>(camera_to_clip, clip_to_viewport);
    }

    static CameraProjection<T> perspective(
        T fov_y_rad, T aspect_xy, T near, T far, 
        T width, T height
    ) {
        geometry::Transform<T> camera_to_clip = geometry::Transform<T>::perspective(fov_y_rad, aspect_xy, near, far);
        geometry::Transform<T> clip_to_viewport = geometry::Transform<T>::viewport(width, height);
        return CameraProjection<T>(camera_to_clip, clip_to_viewport);
    }
};

template<typename Derived, typename T>
class ProjectiveCamera : public Camera<Derived, T> {
protected:
    CameraProjection<T> m_projection{};

public:
    ProjectiveCamera(
        const CameraProjection<T>& projection
    ) : m_projection(projection) {}

    const CameraProjection<T>& projection() const {
        return m_projection;
    }
};

template<typename T>
inline CameraProjection<T> create_orthographic_projection(
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

template<typename T>
inline CameraProjection<T> create_perspective_projection(
    const math::Vector<int, 2>& film_resolution,
    const math::Vector<T, 2>& film_physical_size,
    T near, T far
) {
    T aspect = film_physical_size.x() / film_physical_size.y();
    T fov_y = 2 * std::atan2(film_physical_size.y() / 2, near);
    return CameraProjection<T>::perspective(
        fov_y,
        aspect,
        near,
        far,
        film_resolution.x(),
        film_resolution.y()
    );
}

template<typename T>
class OrthographicCamera : public ProjectiveCamera<OrthographicCamera<T>, T> {
    friend class Camera<OrthographicCamera<T>, T>;
    friend class ProjectiveCamera<OrthographicCamera<T>, T>;

public:
    OrthographicCamera(
        T left, T right, T bottom, T top, T near, T far, 
        T resolution_x, T resolution_y
    ) : ProjectiveCamera<OrthographicCamera<T>, T>(CameraProjection<T>::orthographic(
        left, right, bottom, top, near, far, 
        resolution_x, resolution_y)
    ) { }

    OrthographicCamera(
        const math::Vector<int, 2>& film_resolution, 
        const math::Vector<T, 2>& film_physical_size,
        T near, T far
    ) : ProjectiveCamera<OrthographicCamera<T>, T>(
        create_orthographic_projection(film_resolution, film_physical_size, near, far)
    ) {}

private:
    geometry::Ray<T, 3> generate_ray_impl(const CameraSample<T>& sample) const {
        auto origin = this->m_projection.apply_viewport_to_camera(sample.p_film);
        auto direction = math::Vector<T, 3>(0, 0, 1);
        return geometry::Ray<T, 3>(origin, direction);
    }

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

template<typename T>
class PerspectiveCamera : public ProjectiveCamera<PerspectiveCamera<T>, T> {
    friend class Camera<PerspectiveCamera<T>, T>;
    friend class ProjectiveCamera<PerspectiveCamera<T>, T>;

public:
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

    PerspectiveCamera(
        const math::Vector<int, 2>& film_resolution, 
        const math::Vector<T, 2>& film_physical_size,
        T near, T far
    ) : ProjectiveCamera<PerspectiveCamera<T>, T>(
        create_perspective_projection(film_resolution, film_physical_size, near, far)
    ) {}

private:
    geometry::Ray<T, 3> generate_ray_impl(const CameraSample<T>& sample) const {
        auto origin = math::Point<T, 3>(0, 0, 0);
        auto p_camera = this->m_projection.apply_viewport_to_camera(sample.p_film);
        return geometry::Ray<T, 3>(origin, p_camera);
    }

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

template<typename T>
class ThinLensOrthographicCamera : public ProjectiveCamera<ThinLensOrthographicCamera<T>, T> {
    friend class Camera<ThinLensOrthographicCamera<T>, T>;
    friend class ProjectiveCamera<ThinLensOrthographicCamera<T>, T>;

private:
    T m_lens_radius{0};
    T m_focal_distance{1};

public:

    ThinLensOrthographicCamera(
        T left, T right, T bottom, T top, T near, T far, 
        T resolution_x, T resolution_y,
        T lens_radius, T focal_distance
    ) : ProjectiveCamera<ThinLensOrthographicCamera<T>, T>(CameraProjection<T>::orthographic(
        left, right, bottom, top, near, far, 
        resolution_x, resolution_y)
    ), m_lens_radius(lens_radius), m_focal_distance(focal_distance) { }

    ThinLensOrthographicCamera(
        const math::Vector<int, 2>& film_resolution, 
        const math::Vector<T, 2>& film_physical_size,
        T near, T far, T lens_radius, T focal_distance
    ) : ProjectiveCamera<ThinLensOrthographicCamera<T>, T>(
            create_orthographic_projection(film_resolution, film_physical_size, near, far)
        ), 
        m_lens_radius(lens_radius), 
        m_focal_distance(focal_distance) {}
    
private:
    geometry::Ray<T, 3> generate_ray_impl(const CameraSample<T>& sample) const {
        auto p_camera = this->m_projection.apply_viewport_to_camera(sample.p_film);
        auto pinhole_ray = geometry::Ray<T, 3>(p_camera, math::Vector<T, 3>(0, 0, 1));
        T t = m_focal_distance / pinhole_ray.direction().z();
        auto p_focus = pinhole_ray.at(t);

        auto p_lens = math::sample_uniform_disk_concentric(sample.p_lens, m_lens_radius);
        auto origin = math::Point<T, 3>(p_lens.x(), p_lens.y(), 0);
        auto direction = (p_focus - origin).normalized();
        return geometry::Ray<T, 3>(origin, direction);
    }

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

template<typename T>
class ThinLensPerspectiveCamera : public ProjectiveCamera<ThinLensPerspectiveCamera<T>, T>{
    friend class Camera<ThinLensPerspectiveCamera<T>, T>;
    friend class ProjectiveCamera<ThinLensPerspectiveCamera<T>, T>;

private:
    T m_lens_radius{0};
    T m_focal_distance{1};
    
public:
    ThinLensPerspectiveCamera(
        const T fov_y_rad,
        const T aspect_xy,
        const T near, const T far,
        const T resolution_x, const T resolution_y,
        T lens_radius, T focal_distance
    ) : ProjectiveCamera<ThinLensPerspectiveCamera<T>, T>(CameraProjection<T>::perspective(
        fov_y_rad, 
        aspect_xy, 
        near, far, 
        resolution_x, resolution_y)
    ), m_lens_radius(lens_radius), m_focal_distance(focal_distance) { }


    ThinLensPerspectiveCamera(
        const math::Vector<int, 2>& film_resolution, 
        const math::Vector<T, 2>& film_physical_size,
        T near, T far, T lens_radius, T focal_distance
    ) : ProjectiveCamera<ThinLensPerspectiveCamera<T>, T>(
            create_perspective_projection(film_resolution, film_physical_size, near, far)
        ), 
        m_lens_radius(lens_radius), 
        m_focal_distance(focal_distance) {}

private:
    geometry::Ray<T, 3> generate_ray_impl(const CameraSample<T>& sample) const {
        auto p_camera = this->m_projection.apply_viewport_to_camera(sample.p_film);
        auto p_lens = math::sample_uniform_disk_concentric(sample.p_lens, m_lens_radius);
        auto origin = math::Point<T, 3>(p_lens.x(), p_lens.y(), 0);
        auto pinhole_ray = geometry::Ray<T, 3>(math::Point<T, 3>(0, 0, 0), p_camera);
        T t = m_focal_distance / pinhole_ray.direction().z();
        auto p_focus = pinhole_ray.at(t);
        auto direction = (p_focus - origin).normalized();
        return geometry::Ray<T, 3>(origin, direction);
    }

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

} // namespace pbpt::camera
