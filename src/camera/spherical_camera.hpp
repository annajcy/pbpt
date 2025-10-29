#pragma once

#include <utility>
#include "camera.hpp"
#include "geometry/ray.hpp"
#include "geometry/spherical.hpp"
#include "math/function.hpp"
#include "math/point.hpp"
#include "math/vector.hpp"

namespace pbpt::camera {

enum class SphericalCameraMapping {
    EqualRectangular,
    EqualArea
};

template<typename T>
class SphericalCamera : public Camera<SphericalCamera<T>, T> {
    friend class Camera<SphericalCamera<T>, T>;

private:
    SphericalCameraMapping m_mapping{SphericalCameraMapping::EqualRectangular};
    math::Vector<int, 2> m_film_resolution{1920, 960};

public:
    SphericalCamera(
        const math::Vector<T, 2>& film_resolution,
        SphericalCameraMapping mapping = SphericalCameraMapping::EqualRectangular)
        : m_mapping(mapping), m_film_resolution(film_resolution) {}

    SphericalCameraMapping mapping() const {
        return m_mapping;
    }

    const math::Vector<int, 2>& film_resolution() const {
        return m_film_resolution;
    }

private:
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


};