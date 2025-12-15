#pragma once

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <limits>
#include <optional>
#include <vector>

#include "camera/film.hpp"
#include "camera/pixel_filter.hpp"
#include "camera/pixel_sensor.hpp"
#include "camera/projective_camera.hpp"
#include "math/random_generator.hpp"
#include "math/vector.hpp"
#include "radiometry/color.hpp"
#include "radiometry/color_spectrum_optimizer.hpp"
#include "radiometry/constant/illuminant_spectrum.hpp"
#include "radiometry/constant/standard_color_spaces.hpp"
#include "radiometry/constant/swatch_reflectances_spectrum.hpp"
#include "radiometry/constant/xyz_spectrum.hpp"
#include "radiometry/sampled_spectrum.hpp"
#include "radiometry/spectrum_distribution.hpp"
#include "shape/shape.hpp"
#include "shape/sphere.hpp"
#include "utils/exr_writer.hpp"
#include "utils/progress_bar.hpp"

namespace pbpt::scene {

/**
 * @brief Minimal sphere-based scene with spectral rendering support.
 *
 * Provides a helper to set up a thin-lens camera, simple objects, and render
 * them to an EXR image using the project's radiometric pipeline.
 */
template<typename T>
class SimpleScene {
public:
    /**
     * @brief Renderable scene primitive consisting of a sphere and an albedo.
     */
    struct SceneObject {
        /// Sphere geometry with transform.
        shape::TransformedShape<T, shape::Sphere> sphere;
        /// RGB albedo used to derive spectral reflectance.
        radiometry::RGB<T> rgb_albedo;
    };

private:
    using PixelFilterType = camera::TentFilter<T>;
    using Illuminant = decltype(radiometry::constant::CIE_D65_ilum<T>);
    using SensorResponse = radiometry::constant::XYZSpectrumType<T>;
    using PixelSensorType = camera::PixelSensor<T, Illuminant, Illuminant, SensorResponse>;
    using FilmType = camera::RGBFilm<T, PixelSensorType>;
    using CameraType = camera::ThinLensPerspectiveCamera<T>;

    static constexpr int SpectrumSampleCount = 4;
    using SampledSpectrumType = radiometry::SampledSpectrum<T, SpectrumSampleCount>;
    using SampledWavelengthType = radiometry::SampledWavelength<T, SpectrumSampleCount>;
    using SampledPdfType = radiometry::SampledPdf<T, SpectrumSampleCount>;

    using SceneObjectAlbedoSpectrumDistributionType = radiometry::RGBAlbedoSpectrumDistribution<T, radiometry::RGBSigmoidPolynomialNormalized>;

    CameraType m_camera{};
    PixelFilterType m_pixel_filter{};

    std::vector<SceneObject> m_scene_objects{};
    radiometry::constant::SwatchReflectance m_background_reflectance{radiometry::constant::SwatchReflectance::Cyan};
    std::vector<SceneObjectAlbedoSpectrumDistributionType> m_sphere_albedo_spectra{};

    Illuminant m_scene_illuminant{radiometry::constant::CIE_D65_ilum<T>};

public:
    /**
     * @brief Construct a simple scene with a camera and a set of spheres.
     *
     * Precomputes spectral albedo distributions for each scene object.
     */
    SimpleScene(
        const camera::ThinLensPerspectiveCamera<T>& camera,
        const std::vector<SceneObject> &scene_objects,
        radiometry::constant::SwatchReflectance background_reflectance = radiometry::constant::SwatchReflectance::Cyan,
        PixelFilterType pixel_filter = PixelFilterType{}
    ) : m_camera(camera),
        m_pixel_filter(pixel_filter),
        m_scene_objects(scene_objects),
        m_background_reflectance(background_reflectance) {
        for (const auto& obj : m_scene_objects) {
            m_sphere_albedo_spectra.emplace_back(
                radiometry::create_albedo_spectrum(obj.rgb_albedo)
            );
        }
    }

    /**
     * @brief Render the scene to an EXR file.
     * @param output_path Destination path (default: `scene.exr`).
     */
    void render(const std::filesystem::path& output_path = std::filesystem::path("scene.exr")) const {
        auto resolution = m_camera.film_resolution();
        math::Vector<T, 2> physical_size(
            static_cast<T>(resolution.x()),
            static_cast<T>(resolution.y())
        );

        PixelSensorType pixel_sensor(
            m_scene_illuminant,
            radiometry::constant::sRGB<T>
        );

        FilmType film(resolution, physical_size, pixel_sensor);
        const math::Point<T, 2> lens_sample(T(0.5), T(0.5));

       

        const int width = resolution.x();
        const int height = resolution.y();
        std::cout << "Starting render: " << width << "x" << height << " pixels." << std::endl;
        const std::size_t total_pixels = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
        utils::ProgressBar progress_bar(total_pixels, 40, "Rendering");
        progress_bar.start(std::cout);

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                math::Point<int, 2> pixel(x, y);
                for (const auto& filtered_sample : m_pixel_filter.get_camera_samples(pixel, 2, 2)) {
                    auto sample = camera::CameraSample<T>::create_thinlens_sample(filtered_sample.film_position, lens_sample);
                    auto ray = m_camera.generate_ray(sample);
                    math::RandomGenerator<T, 1> rng;
                    // const auto wavelengths = radiometry::sample_uniform_wavelengths_stratified<T, SpectrumSampleCount>(rng.generate_uniform(0, T(1))[0]);
                    // const auto pdf = radiometry::sample_uniform_wavelengths_pdf(wavelengths);
                    const auto wavelengths = radiometry::sample_visible_wavelengths_stratified<T, SpectrumSampleCount>(rng.generate_uniform(0, T(1))[0]);
                    const auto pdf = radiometry::sample_visible_wavelengths_pdf(wavelengths);
                    auto spectrum = trace_ray(ray, wavelengths);
                    film.template add_sample<SpectrumSampleCount>(pixel, spectrum, wavelengths, pdf, filtered_sample.weight);
                }
                progress_bar.update(std::cout);
            }
        }

        progress_bar.finish(std::cout);
        utils::write_exr(film, output_path, width, height);
    }

private:
    SampledSpectrumType trace_ray(
        const geometry::Ray<T, 3>& ray,
        const SampledWavelengthType& wavelengths
    ) const {
        T closest = std::numeric_limits<T>::infinity();
        std::optional<geometry::SurfaceInteraction<T>> closest_hit{};

        int shape_index = -1;
        for (const auto& obj : m_scene_objects) {
            if (auto hit = obj.sphere.intersect(ray)) {
                const auto& [si, t_hit] = *hit;
                if (t_hit < closest) {
                    closest = t_hit;
                    closest_hit = si;
                    shape_index = &obj - &m_scene_objects[0];
                }
            }
        }

        if (closest_hit.has_value()) {
            return shade_hit(closest_hit.value(), ray, wavelengths, shape_index);
        }
        
        return shade_background(ray.direction(), wavelengths);
    }

    SampledSpectrumType shade_hit(
        const geometry::SurfaceInteraction<T>& si,
        const geometry::Ray<T, 3>& ray,
        const SampledWavelengthType& wavelengths,
        const int sphere_index
    ) const {
        auto normal = si.n().to_vector().normalized();
        const T intensity = std::max(T(0), normal.dot(-ray.direction()));
        auto albedo_sample = m_sphere_albedo_spectra[sphere_index].sample(wavelengths);
        auto illuminant_sample = m_scene_illuminant.sample(wavelengths);
        return albedo_sample * illuminant_sample * intensity;
    }

    SampledSpectrumType shade_background(
        const math::Vector<T, 3>& dir,
        const SampledWavelengthType& wavelengths
    ) const {
        auto background = radiometry::constant::get_swatch_reflectance<T>(m_background_reflectance);
        return background.sample(wavelengths) * m_scene_illuminant.sample(wavelengths);
    }
};

}
