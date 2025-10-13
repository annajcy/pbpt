#pragma once

#include <cmath>

#include "camera/camera_sample.hpp"
#include "geometry/ray.hpp"

#include "math/point.hpp"
#include "math/vector.hpp"
#include "camera_projection.hpp"


namespace pbpt::camera {

template<typename T>
class Camera {
public:
    Camera() = default;
    virtual ~Camera() = default;
    virtual geometry::Ray<T, 3> generate_ray(const CameraSample<T>& sample) = 0;
    virtual geometry::RayDifferential<T, 3> generate_differential_ray(const CameraSample<T>& sample) = 0;
};

template<typename T>
class ProjectiveCamera : public Camera<T> {
protected:
    CameraProjection<T> m_projection{};

public:
    ProjectiveCamera(const CameraProjection<T>& projection) : m_projection(projection) {}

    const CameraProjection<T>& projection() const {
        return m_projection;
    }
};

template<typename T>
class OrthographicCamera : public ProjectiveCamera<T> {
public:
    template<typename FilmType>
    OrthographicCamera(const FilmType& film, T near, T far)
        : ProjectiveCamera<T>(
              CameraProjection<T>::orthographic(
                -film.physical_size().x() / 2, 
                film.physical_size().x() / 2, 
                -film.physical_size().y() / 2, 
                film.physical_size().y() / 2, 
                near, 
                far, 
                film.resolution().x(), 
                film.resolution().y())) {}

    geometry::Ray<T, 3> generate_ray(const CameraSample<T>& sample) override {
        auto origin = this->m_projection.apply_viewport_to_camera(sample.p_film);
        auto direction = math::Vector<T, 3>(0, 0, 1);
        return geometry::Ray<T, 3>(origin, direction);
    }

    geometry::RayDifferential<T, 3> generate_differential_ray(const CameraSample<T>& sample) override {
        geometry::Ray<T, 3> main_ray = this->generate_ray(sample);

        T eps = static_cast<T>(1e-3);
        auto p_film_x = sample.p_film + math::Vector<T, 2>(eps, 0); 
        auto p_film_y = sample.p_film + math::Vector<T, 2>(0, eps);

        CameraSample<T> sample_x = { p_film_x};
        CameraSample<T> sample_y = { p_film_y};

        geometry::Ray<T, 3> ray_x = this->generate_ray(sample_x);
        geometry::Ray<T, 3> ray_y = this->generate_ray(sample_y);

        return geometry::RayDifferential<T, 3>(main_ray, {ray_x, ray_y});
    }
};

template<typename T>
class PerspectiveCamera : public ProjectiveCamera<T> {
public:
    PerspectiveCamera(const CameraProjection<T>& projection)
        : ProjectiveCamera<T>(projection) {}

    template<typename FilmType>
    PerspectiveCamera(const FilmType& film, T near, T far)
        : ProjectiveCamera<T>(
              CameraProjection<T>::perspective(
                std::atan2(film.physical_size().y(), 2 * near) * 2,
                film.physical_size().x() / film.physical_size().y(),
                near,
                far,
                film.resolution().x(),
                film.resolution().y())) {}

    geometry::Ray<T, 3> generate_ray(const CameraSample<T>& sample) override {
        auto origin = math::Point<T, 3>(0, 0, 0);
        auto p_camera = this->m_projection.apply_viewport_to_camera(sample.p_film);
        return geometry::Ray<T, 3>(origin, p_camera);
    }

    geometry::RayDifferential<T, 3> generate_differential_ray(const CameraSample<T>& sample) override {
        
        geometry::Ray<T, 3> main_ray = this->generate_ray(sample);

        T eps = static_cast<T>(1e-3);
        auto p_film_x = sample.p_film + math::Vector<T, 2>(eps, 0); 
        auto p_film_y = sample.p_film + math::Vector<T, 2>(0, eps);

        CameraSample<T> sample_x = { p_film_x };
        CameraSample<T> sample_y = { p_film_y };

        geometry::Ray<T, 3> ray_x = this->generate_ray(sample_x);
        geometry::Ray<T, 3> ray_y = this->generate_ray(sample_y);

        return geometry::RayDifferential<T, 3>(main_ray, {ray_x, ray_y});
    }
};

// 将 [0,1]^2 映射到 [-1,1]^2
template<typename T>
math::Point<T, 2> sample_lens_concentric(const math::Point<T, 2>& p_lens, T lens_radius) {
    math::Point<T, 2> p_offset = 2.0 * p_lens.to_vector() - math::Vector<T, 2>(1, 1);

    if (p_offset.x() == 0 && p_offset.y() == 0) {
        return math::Point<T, 2>(0, 0);
    }

    T theta, r;
    if (std::abs(p_offset.x()) > std::abs(p_offset.y())) {
        r = p_offset.x();
        theta = (M_PI / 4.0) * (p_offset.y() / p_offset.x());
    } else {
        r = p_offset.y();
        theta = (M_PI / 2.0) - (M_PI / 4.0) * (p_offset.x() / p_offset.y());
    }

    return math::Point<T, 2>(lens_radius * r * math::Vector<T, 2>(std::cos(theta), std::sin(theta)));
}

template<typename T>
class ThinLensOrthographicCamera : public OrthographicCamera<T> {
private:
    T m_lens_radius{0};
    T m_focal_distance{1};

public:
    ThinLensOrthographicCamera(const CameraProjection<T>& projection, T lens_radius, T focal_distance)
        : OrthographicCamera<T>(projection), m_lens_radius(lens_radius), m_focal_distance(focal_distance) {}

    template<typename FilmType>
    ThinLensOrthographicCamera(const FilmType& film, T near, T far, T lens_radius, T focal_distance)
        : OrthographicCamera<T>(film, near, far), m_lens_radius(lens_radius), m_focal_distance(focal_distance) {}

    // Overload for ThinLensCameraSample
    geometry::Ray<T, 3> generate_ray(const ThinLensCameraSample<T>& sample) const {
        auto p_camera = this->m_projection.apply_viewport_to_camera(sample.p_film);
        auto pinhole_ray = geometry::Ray<T, 3>(p_camera, math::Vector<T, 3>(0, 0, 1));
        T t = m_focal_distance / pinhole_ray.direction().z();
        auto p_focus = pinhole_ray.at(t);

        auto p_lens = sample_lens_concentric(sample.p_lens, m_lens_radius);
        auto origin = math::Point<T, 3>(p_lens.x(), p_lens.y(), 0);
        auto direction = (p_focus - origin).normalized();
        return geometry::Ray<T, 3>(origin, direction);
    }

    geometry::RayDifferential<T, 3> generate_differential_ray(const ThinLensCameraSample<T>& sample) const {
       
        geometry::Ray<T, 3> main_ray = this->generate_ray(sample);

        T eps = static_cast<T>(1e-3);
        auto p_film_x = sample.p_film + math::Vector<T, 2>(eps, 0); 
        auto p_film_y = sample.p_film + math::Vector<T, 2>(0, eps);

        ThinLensCameraSample<T> sample_x = { p_film_x, sample.p_lens };
        ThinLensCameraSample<T> sample_y = { p_film_y, sample.p_lens };

        geometry::Ray<T, 3> ray_x = this->generate_ray(sample_x);
        geometry::Ray<T, 3> ray_y = this->generate_ray(sample_y);

        return geometry::RayDifferential<T, 3>(main_ray, {ray_x, ray_y});
    }

};

template<typename T>
class ThinLensPerspectiveCamera : public PerspectiveCamera<T> {
private:
    T m_lens_radius{0};
    T m_focal_distance{1};
    
public:
    ThinLensPerspectiveCamera(const CameraProjection<T>& projection, T lens_radius, T focal_distance)
        : PerspectiveCamera<T>(projection), m_lens_radius(lens_radius), m_focal_distance(focal_distance) {}

    template<typename FilmType>
    ThinLensPerspectiveCamera(const FilmType& film, T near, T far, T lens_radius, T focal_distance)
        : PerspectiveCamera<T>(film, near, far), m_lens_radius(lens_radius), m_focal_distance(focal_distance) {}

    // Overload for ThinLensCameraSample
    geometry::Ray<T, 3> generate_ray(const ThinLensCameraSample<T>& sample) {
        auto p_camera = this->m_projection.apply_viewport_to_camera(sample.p_film);
        auto p_lens = sample_lens_concentric(sample.p_lens, m_lens_radius);
        auto origin = math::Point<T, 3>(p_lens.x(), p_lens.y(), 0);
        auto pinhole_ray = geometry::Ray<T, 3>(math::Point<T, 3>(0, 0, 0), p_camera);
        T t = m_focal_distance / pinhole_ray.direction().z();
        auto p_focus = pinhole_ray.at(t);
        auto direction = (p_focus - origin).normalized();
        return geometry::Ray<T, 3>(origin, direction);
    }

    geometry::RayDifferential<T, 3> generate_differential_ray(const ThinLensCameraSample<T>& sample) {
  
        geometry::Ray<T, 3> main_ray = this->generate_ray(sample);

        T eps = static_cast<T>(1e-3);
        auto p_film_x = sample.p_film + math::Vector<T, 2>(eps, 0); 
        auto p_film_y = sample.p_film + math::Vector<T, 2>(0, eps);

        ThinLensCameraSample<T> sample_x = { p_film_x, sample.p_lens };
        ThinLensCameraSample<T> sample_y = { p_film_y, sample.p_lens };

        geometry::Ray<T, 3> ray_x = this->generate_ray(sample_x);
        geometry::Ray<T, 3> ray_y = this->generate_ray(sample_y);

        return geometry::RayDifferential<T, 3>(main_ray, {ray_x, ray_y});
    }
};

} // namespace pbpt::camera
