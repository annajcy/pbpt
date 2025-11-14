#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

#include "stb_image_write.h"
#include "camera/film.hpp"
#include "camera/pixel_sensor.hpp"
#include "camera/projective_camera.hpp"
#include "math/vector.hpp"
#include "radiometry/color.hpp"
#include "radiometry/color_spectrum_optimizer.hpp"
#include "radiometry/constant/illuminant_spectrum.hpp"
#include "radiometry/constant/standard_color_spaces.hpp"
#include "radiometry/constant/xyz_spectrum.hpp"
#include "radiometry/sampled_spectrum.hpp"
#include "radiometry/spectrum_distribution.hpp"
#include "shape/shape.hpp"
#include "shape/sphere.hpp"

namespace pbpt::scene {

template<typename T>
class Scene {
private:
    using Illuminant = decltype(radiometry::constant::CIE_D65_ilum<T>);
    using SensorResponse = radiometry::constant::XYZSpectrumType<T>;
    using PixelSensorType = camera::PixelSensor<T, Illuminant, Illuminant, SensorResponse>;
    using FilmType = camera::RGBFilm<T, PixelSensorType>;
    static constexpr int SpectrumSampleCount = 20;
    using SampledSpectrumType = radiometry::SampledSpectrum<T, SpectrumSampleCount>;
    using SampledWavelengthType = radiometry::SampledWavelength<T, SpectrumSampleCount>;
    using SampledPdfType = radiometry::SampledPdf<T, SpectrumSampleCount>;

    camera::ThinLensPerspectiveCamera<T> m_camera{};
    std::vector<shape::TransformedShape<T, shape::Sphere>> m_spheres{};
    radiometry::RGB<T> m_sphere_albedo{};
    radiometry::RGBAlbedoSpectrumDistribution<T, radiometry::RGBSigmoidPolynomialNormalized> m_sphere_albedo_spectrum{};
    Illuminant m_scene_illuminant{radiometry::constant::CIE_D65_ilum<T>};
    radiometry::RGBSigmoidPolynomialNormalized<T> m_background_bottom_rsp{};
    radiometry::RGBSigmoidPolynomialNormalized<T> m_background_top_rsp{};

public:
    Scene(
        const camera::ThinLensPerspectiveCamera<T>& camera,
        const std::vector<shape::TransformedShape<T, shape::Sphere>>& spheres,
        const radiometry::RGB<T>& sphere_albedo
    ) : m_camera(camera),
        m_spheres(spheres),
        m_sphere_albedo(sphere_albedo),
        m_sphere_albedo_spectrum(create_albedo_spectrum(m_sphere_albedo)),
        m_background_bottom_rsp(optimize_rgb_to_rsp(radiometry::RGB<T>(T(0.05), T(0.07), T(0.15)))),
        m_background_top_rsp(optimize_rgb_to_rsp(radiometry::RGB<T>(T(0.7), T(0.8), T(1.0))))
    {}

    void render(const std::filesystem::path& output_path = std::filesystem::path("scene.png")) const {
        auto resolution = film_resolution_from_camera();
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
        const auto wavelengths = create_wavelength_samples();
        const auto pdf = create_wavelength_pdf();

        const int width = resolution.x();
        const int height = resolution.y();

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                math::Point<T, 2> p_film(
                    static_cast<T>(x) + T(0.5),
                    static_cast<T>(y) + T(0.5)
                );
                auto sample = camera::CameraSample<T>::create_thinlens_sample(p_film, lens_sample);
                auto ray = m_camera.generate_ray(sample);
                auto spectrum = trace_ray(ray, wavelengths);
                film.template add_sample<SpectrumSampleCount>(math::Point<int, 2>(x, y), spectrum, wavelengths, pdf, T(1));
            }
        }

        write_png(film, output_path, width, height);
    }

private:
    math::Vector<int, 2> film_resolution_from_camera() const {
        const auto viewport = m_camera.projection().clip_to_viewport();
        const auto& mat = viewport.matrix();
        auto width_value = mat.at(0, 0) * T(2);
        auto height_value = mat.at(1, 1) * T(2);
        const int width = std::max(1, static_cast<int>(std::lround(static_cast<double>(width_value))));
        const int height = std::max(1, static_cast<int>(std::lround(static_cast<double>(height_value))));
        return math::Vector<int, 2>(width, height);
    }

    SampledSpectrumType trace_ray(
        const geometry::Ray<T, 3>& ray,
        const SampledWavelengthType& wavelengths
    ) const {
        T closest = std::numeric_limits<T>::infinity();
        std::optional<geometry::SurfaceInteraction<T>> closest_hit{};
        for (const auto& sphere : m_spheres) {
            if (auto hit = sphere.intersect(ray)) {
                const auto& [si, t_hit] = *hit;
                if (t_hit < closest) {
                    closest = t_hit;
                    closest_hit = si;
                }
            }
        }

        if (closest_hit.has_value()) {
            return shade_hit(closest_hit.value(), ray, wavelengths);
        }
        return shade_background(ray.direction(), wavelengths);
    }

    SampledSpectrumType shade_hit(
        const geometry::SurfaceInteraction<T>& si,
        const geometry::Ray<T, 3>& ray,
        const SampledWavelengthType& wavelengths
    ) const {
        auto normal = si.n().to_vector().normalized();
        const math::Vector<T, 3> light_dir = math::Vector<T, 3>(T(-0.4), T(0.8), T(-1)).normalized();
        const T ndotl = std::max(T(0), normal.dot(light_dir));
        const T facing = std::max(T(0), normal.dot(-ray.direction()));
        const T intensity = std::clamp(T(0.1) + ndotl * T(0.9) + facing * T(0.2), T(0), T(1));
        auto albedo_sample = m_sphere_albedo_spectrum.sample(wavelengths);
        auto illuminant_sample = m_scene_illuminant.sample(wavelengths);
        return albedo_sample * illuminant_sample * intensity;
    }

    SampledSpectrumType shade_background(
        const math::Vector<T, 3>& dir,
        const SampledWavelengthType& wavelengths
    ) const {
        auto unit_dir = dir.normalized();
        T t = T(0.5) * (unit_dir.y() + T(1));
        t = std::clamp(t, T(0), T(1));
        auto bottom = sample_rgb_illuminant_spectrum(m_background_bottom_rsp, wavelengths);
        auto top = sample_rgb_illuminant_spectrum(m_background_top_rsp, wavelengths);
        return bottom * (T(1) - t) + top * t;
    }

    static std::uint8_t to_byte(T value) {
        auto clamped = std::clamp(value, T(0), T(1));
        return static_cast<std::uint8_t>(std::lround(static_cast<double>(clamped) * 255.0));
    }

    void write_png(
        const FilmType& film,
        const std::filesystem::path& output_path,
        int width,
        int height
    ) const {
        if (!output_path.parent_path().empty()) {
            std::filesystem::create_directories(output_path.parent_path());
        }

        std::vector<std::uint8_t> buffer(
            static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 3
        );

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                auto rgb = film.get_pixel_rgb(math::Point<int, 2>(x, y)).clamp();
                std::size_t idx = (
                    static_cast<std::size_t>(height - 1 - y) * static_cast<std::size_t>(width) +
                    static_cast<std::size_t>(x)
                ) * 3;
                buffer[idx + 0] = to_byte(rgb.r());
                buffer[idx + 1] = to_byte(rgb.g());
                buffer[idx + 2] = to_byte(rgb.b());
            }
        }

        if (stbi_write_png(output_path.string().c_str(), width, height, 3, buffer.data(), width * 3) == 0) {
            throw std::runtime_error("Failed to write image to " + output_path.string());
        }
    }

    SampledWavelengthType create_wavelength_samples() const {
        math::Vector<T, SpectrumSampleCount> values{};
        const T min_lambda = radiometry::lambda_min<T>;
        const T max_lambda = radiometry::lambda_max<T>;
        const T span = max_lambda - min_lambda;
        const T step = span / static_cast<T>(SpectrumSampleCount);
        for (int i = 0; i < SpectrumSampleCount; ++i) {
            values[i] = min_lambda + step * (static_cast<T>(i) + T(0.5));
        }
        return SampledWavelengthType(values);
    }

    SampledPdfType create_wavelength_pdf() const {
        const T span = radiometry::lambda_max<T> - radiometry::lambda_min<T>;
        const T pdf_value = T(1) / span;
        return SampledPdfType(math::Vector<T, SpectrumSampleCount>::filled(pdf_value));
    }

    SampledSpectrumType sample_rgb_illuminant_spectrum(
        const radiometry::RGBSigmoidPolynomialNormalized<T>& rsp,
        const SampledWavelengthType& wavelengths
    ) const {
        radiometry::RGBIlluminantSpectrumDistribution<
            T,
            radiometry::RGBSigmoidPolynomialNormalized,
            Illuminant
        > spectrum(rsp, m_scene_illuminant);
        return spectrum.sample(wavelengths);
    }

    static radiometry::RGBSigmoidPolynomialNormalized<T> optimize_rgb_to_rsp(
        const radiometry::RGB<T>& rgb
    ) {
        auto optim_res = radiometry::optimize_albedo_rgb_sigmoid_polynomial(
            rgb,
            radiometry::constant::sRGB<T>,
            radiometry::constant::CIE_D65_ilum<T>
        );
        auto coeff = optim_res.normalized_coeffs;
        return radiometry::RGBSigmoidPolynomialNormalized<T>{coeff};
    }

    static radiometry::RGBAlbedoSpectrumDistribution<T, radiometry::RGBSigmoidPolynomialNormalized> create_albedo_spectrum(
        const radiometry::RGB<T>& rgb
    ) {
        return radiometry::RGBAlbedoSpectrumDistribution<
            T,
            radiometry::RGBSigmoidPolynomialNormalized
        >(optimize_rgb_to_rsp(rgb));
    }
};

}
