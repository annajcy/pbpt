#pragma once

#include <filesystem>
#include <iostream>
#include <limits>
#include <optional>
#include <vector>

#include "camera/film.hpp"
#include "camera/pixel_filter.hpp"
#include "camera/pixel_sensor.hpp"
#include "camera/projective_camera.hpp"
#include "camera/render_transform.hpp"
#include "geometry/interaction.hpp"
#include "geometry/transform.hpp"
#include "light/solid_angle_area_light.hpp"
#include "math/random_generator.hpp"
#include "math/vector.hpp"
#include "radiometry/color.hpp"
#include "radiometry/color_spectrum_lut.hpp"
#include "radiometry/constant/illuminant_spectrum.hpp"
#include "radiometry/constant/standard_color_spaces.hpp"
#include "shape/sphere.hpp"
#include "shape/triangle.hpp"
#include "utils/exr_writer.hpp"
#include "utils/progress_bar.hpp"
#include "utils/system_info.hpp"

namespace pbpt::scene {

/**
 * @brief Minimal triangle-based test scene with a single spherical area light.
 *
 * Templated so callers can pick float/double. Construct with camera,
 * a list of triangle objects, and a spherical area light, then call render().
 */
template <typename T>
class TriangleScene {
public:
    struct SceneObject {
        pbpt::shape::Triangle<T> triangle;
        pbpt::radiometry::RGB<T> rgb_albedo;
    };

    struct SceneAreaLight {
        pbpt::shape::Sphere<T> sphere;
        T intensity{T(1)};
    };

private:
    using PixelFilterType = pbpt::camera::TentFilter<T>;
    using StandardIlluminant = decltype(pbpt::radiometry::constant::CIE_D65_ilum<T>);
    using SensorResponse = pbpt::radiometry::constant::XYZSpectrumType<T>;
    using PixelSensorType = pbpt::camera::PixelSensor<T, StandardIlluminant, StandardIlluminant, SensorResponse>;
    using FilmType = pbpt::camera::RGBFilm<T, PixelSensorType>;
    using CameraType = pbpt::camera::ThinLensPerspectiveCamera<T>;

    static constexpr int SpectrumSampleCount = 4;
    using SampledSpectrumType = pbpt::radiometry::SampledSpectrum<T, SpectrumSampleCount>;
    using SampledWavelengthType = pbpt::radiometry::SampledWavelength<T, SpectrumSampleCount>;
    using SceneObjectAlbedoSpectrumDistributionType =
        pbpt::radiometry::RGBAlbedoSpectrumDistribution<T, pbpt::radiometry::RGBSigmoidPolynomialNormalized>;

    struct SceneAggregate {
        const std::vector<SceneObject>* objects{};
        const SceneAreaLight* area_light{};
        int skip_index{-1};

        bool is_intersected(const pbpt::geometry::Ray<T, 3>& ray) const {
            if (area_light && area_light->sphere.is_intersected(ray).has_value()) {
                return true;
            }

            for (int i = 0; i < static_cast<int>(objects->size()); ++i) {
                if (i == skip_index) {
                    continue;
                }
                if ((*objects)[i].triangle.is_intersected(ray).has_value()) {
                    return true;
                }
            }
            return false;
        }
    };

private:
    CameraType m_camera{};
    PixelFilterType m_pixel_filter{};

    std::vector<SceneObject> m_scene_objects{};
    std::vector<SceneObjectAlbedoSpectrumDistributionType> m_object_albedo_spectra{};

    SceneAreaLight m_area_light{};
    StandardIlluminant m_sensor_illuminant{pbpt::radiometry::constant::CIE_D65_ilum<T>};

public:
    TriangleScene(
        const CameraType& camera,
        const std::vector<SceneObject>& scene_objects,
        const SceneAreaLight& area_light,
        PixelFilterType pixel_filter = PixelFilterType{}
    ) : m_camera(camera),
        m_pixel_filter(pixel_filter),
        m_scene_objects(scene_objects),
        m_area_light(area_light) {
        m_object_albedo_spectra.reserve(m_scene_objects.size());
        for (const auto& obj : m_scene_objects) {
            m_object_albedo_spectra.emplace_back(
                pbpt::radiometry::create_srgb_albedo_spectrum(obj.rgb_albedo)
            );
        }
    }

    void render(
        const std::filesystem::path& output_path = std::filesystem::path("triangle_scene.exr"),
        const pbpt::geometry::Transform<T>& camera_to_render = pbpt::geometry::Transform<T>::identity()
    ) const {
        auto resolution = m_camera.film_resolution();
        pbpt::math::Vector<T, 2> physical_size(
            static_cast<T>(resolution.x()),
            static_cast<T>(resolution.y())
        );

        PixelSensorType pixel_sensor(
            m_sensor_illuminant,
            pbpt::radiometry::constant::sRGB<T>
        );

        FilmType film(resolution, physical_size, pixel_sensor);
        const pbpt::math::Point<T, 2> lens_sample(T(0.5), T(0.5));

        const int width = resolution.x();
        const int height = resolution.y();
        std::cout << "Starting triangle scene render: " << width << "x" << height << " pixels." << std::endl;
        const std::size_t total_pixels = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
        pbpt::utils::ProgressBar progress_bar(total_pixels, 40, "Rendering");
        progress_bar.start(std::cout);

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                pbpt::math::Point<int, 2> pixel(x, y);
                for (const auto& filtered_sample : m_pixel_filter.template get_camera_samples<2, 2>(pixel)) {
                    auto sample =
                        pbpt::camera::CameraSample<T>::create_thinlens_sample(filtered_sample.film_position, lens_sample);
                    auto ray = m_camera.generate_ray(sample);
                    ray = camera_to_render.transform_ray(ray);

                    pbpt::math::RandomGenerator<T, 1> rng1d;
                    auto wavelengths = pbpt::radiometry::sample_visible_wavelengths_stratified<T, SpectrumSampleCount>(
                        rng1d.generate_uniform(T(0), T(1))
                    );
                    auto pdf = pbpt::radiometry::sample_visible_wavelengths_pdf(wavelengths);

                    pbpt::math::RandomGenerator<T, 2> rng2d;
                    auto spectrum = trace_ray(ray, wavelengths, rng2d);
                    film.template add_sample<SpectrumSampleCount>(pixel, spectrum, wavelengths, pdf, filtered_sample.weight);
                }
            }
            progress_bar.update(std::cout, width);
        }

        progress_bar.finish(std::cout);
        pbpt::utils::write_exr(film, output_path, width, height);
    }

private:
    auto make_area_light() const {
        auto power_spectrum = pbpt::radiometry::ConstantSpectrumDistribution<T>(m_area_light.intensity)
                              * pbpt::radiometry::constant::CIE_D65_ilum<T>;

        return pbpt::light::SolidAngleAreaLight<T, pbpt::shape::Sphere<T>, decltype(power_spectrum)>(
            m_area_light.sphere,
            power_spectrum
        );
    }

    template <typename RNG2D>
    SampledSpectrumType trace_ray(
        const pbpt::geometry::Ray<T, 3>& ray,
        const SampledWavelengthType& wavelengths,
        RNG2D& rng2d
    ) const {
        T closest = std::numeric_limits<T>::infinity();
        std::optional<pbpt::geometry::SurfaceInteraction<T>> closest_hit{};
        bool hit_light = false;
        int shape_index = -1;

        for (int i = 0; i < static_cast<int>(m_scene_objects.size()); ++i) {
            const auto& obj = m_scene_objects[i];
            if (auto hit = obj.triangle.intersect(ray)) {
                const auto& [si, t_hit] = *hit;
                if (t_hit < closest) {
                    closest = t_hit;
                    closest_hit = si;
                    hit_light = false;
                    shape_index = i;
                }
            }
        }

        if (auto hit = m_area_light.sphere.intersect(ray)) {
            const auto& [si, t_hit] = *hit;
            if (t_hit < closest) {
                closest = t_hit;
                closest_hit = si;
                hit_light = true;
                shape_index = -1;
            }
        }

        if (!closest_hit.has_value()) {
            return shade_background(wavelengths);
        }

        if (hit_light) {
            return shade_emissive(closest_hit.value(), ray, wavelengths);
        }

        return shade_diffuse(closest_hit.value(), ray, wavelengths, shape_index, rng2d);
    }

    SampledSpectrumType shade_emissive(
        const pbpt::geometry::SurfaceInteraction<T>& si,
        const pbpt::geometry::Ray<T, 3>& ray,
        const SampledWavelengthType& wavelengths
    ) const {
        T cos_theta = si.n().dot(-ray.direction());
        if (cos_theta <= T(0)) {
            return SampledSpectrumType{};
        }
        auto area_light = make_area_light();
        return area_light.template emission_spectrum<SpectrumSampleCount>(
            wavelengths,
            si.point(),
            -ray.direction()
        );
    }

    template <typename RNG2D>
    SampledSpectrumType shade_diffuse(
        const pbpt::geometry::SurfaceInteraction<T>& si,
        const pbpt::geometry::Ray<T, 3>& ray,
        const SampledWavelengthType& wavelengths,
        int tri_index,
        RNG2D& rng2d
    ) const {
        auto albedo = m_object_albedo_spectra[tri_index].sample(wavelengths);

        pbpt::geometry::NormalInteraction<T> ref_interaction(
            si.p_lower(),
            si.p_upper(),
            si.wo(),
            si.n()
        );

        const pbpt::math::Point<T, 2> u_light = pbpt::math::Point<T, 2>::from_array(rng2d.generate_uniform(T(0), T(1)));
        auto area_light = make_area_light();
        auto sample_opt = area_light.template sample_light<SpectrumSampleCount>(wavelengths, ref_interaction, u_light);
        if (!sample_opt.has_value()) {
            return SampledSpectrumType{};
        }

        const auto& light_sample = sample_opt.value();
        if (light_sample.pdf <= T(0)) {
            return SampledSpectrumType{};
        }

        T cos_theta = std::max(T(0), si.n().dot(light_sample.wi));
        if (cos_theta <= T(0)) {
            return SampledSpectrumType{};
        }

        SceneAggregate aggregate{&m_scene_objects, &m_area_light, tri_index};
        if (!light_sample.visibility_tester.is_unoccluded(aggregate)) {
            return SampledSpectrumType{};
        }

        auto f = albedo * (T(1) / pbpt::math::pi_v<T>);
        return f * light_sample.radiance * (cos_theta / light_sample.pdf);
    }

    SampledSpectrumType shade_background(
        const SampledWavelengthType& wavelengths
    ) const {
        (void)wavelengths;
        return SampledSpectrumType{};
    }
};

}  // namespace pbpt::scene
