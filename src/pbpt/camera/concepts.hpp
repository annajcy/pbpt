#pragma once

#include <concepts>

#include "pbpt/camera/camera.hpp"
#include "pbpt/geometry/ray.hpp"
#include "pbpt/math/point.hpp"
#include "pbpt/radiometry/sampled_spectrum.hpp"

namespace pbpt::camera {

template <typename CameraT, typename T>
concept CameraRayGeneratorConcept = requires(const CameraT& camera, const CameraSample<T>& sample) {
    { camera.generate_ray(sample) } -> std::same_as<geometry::Ray<T, 3>>;
    { camera.generate_differential_ray(sample) } -> std::same_as<geometry::RayDifferential<T, 3>>;
};

template <typename PixelFilterT, typename T>
concept PixelFilterSampleConcept =
    requires(const PixelFilterT& pixel_filter, const math::Point<int, 2>& pixel, const math::Point<T, 2>& uv) {
        { pixel_filter.sample_film_position(pixel, uv).film_position };
        { pixel_filter.sample_film_position(pixel, uv).weight } -> std::convertible_to<T>;
    };

template <typename FilmT, typename T, int N>
concept FilmAccumulationConcept = requires(
    FilmT& film, const math::Point<int, 2>& pixel, const radiometry::SampledSpectrum<T, N>& radiance,
    const radiometry::SampledWavelength<T, N>& wavelengths, const radiometry::SampledPdf<T, N>& pdf, T weight) {
    film.resolution();
    film.template add_sample<N>(pixel, radiance, wavelengths, pdf, weight);
    film.develop();
};

template <typename RenderTransformT, typename T>
concept RenderTransformRayConcept =
    requires(const RenderTransformT& render_transform, const geometry::Ray<T, 3>& ray,
             const geometry::RayDifferential<T, 3>& ray_diff) {
        render_transform.camera_to_render().transform_ray_main(ray);
        render_transform.camera_to_render().transform_ray_differential(ray_diff);
    };

}  // namespace pbpt::camera
