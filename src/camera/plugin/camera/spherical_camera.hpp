#pragma once

#include <utility>

#include "pbpt/camera/camera.hpp"

namespace pbpt::camera {

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
     * @brief Construct a spherical camera from explicit resolution.
     *
     * @param film_resolution The resolution of the film/sensor in pixels.
     * @param mapping         How to map film coordinates to the sphere (default: Equirectangular).
     */
    SphericalCamera(
        const math::Vector<int, 2>& film_resolution,
        SphericalCameraMapping mapping = SphericalCameraMapping::EqualRectangular
    ) : m_mapping(mapping), m_film_resolution(film_resolution) {}

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

        T eps = static_cast<T>(1);
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
