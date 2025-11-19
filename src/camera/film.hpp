#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <vector>

#include "math/point.hpp"
#include "math/utils.hpp"
#include "math/vector.hpp"

#include "radiometry/color.hpp"
#include "radiometry/sampled_spectrum.hpp"

namespace pbpt::camera {

template<typename Derived, typename T>
class Film {
private:
    math::Vector<int, 2> m_resolution{};
    math::Vector<T, 2> m_physical_size{};

public:
    Film(const math::Vector<int, 2>& resolution, const math::Vector<T, 2>& physical_size)
        : m_resolution(resolution), m_physical_size(physical_size) {}

    const math::Vector<int, 2>& resolution() const { return m_resolution; }
    const math::Vector<T, 2>& physical_size() const { return m_physical_size; }

    template<int N>
    void add_sample(
        const math::Point<int, 2>& p_film,
        const radiometry::SampledSpectrum<T, N>& radiance,
        const radiometry::SampledWavelength<T, N>& wavelengths,
        const radiometry::SampledPdf<T, N>& pdf,
        T weight
    ) {
        as_derived().template add_sample_impl<N>(p_film, radiance, wavelengths, pdf, weight);
    }

    template<typename SpectrumType>
    void add_sample(
        const math::Point<int, 2>& p_film,
        const SpectrumType& radiance,
        T weight
    ) {
        as_derived().add_sample_impl(p_film, radiance, weight);
    }

    Derived& as_derived() {
        return static_cast<Derived&>(*this);
    }

    const Derived& as_derived() const {
        return static_cast<const Derived&>(*this);
    }
};

template<typename T, typename PixelSensorType>
class RGBFilm : public Film<RGBFilm<T, PixelSensorType>, T> {
public:

    struct Pixel {
        std::array<T, 3> rgb_sum{0.0, 0.0, 0.0};
        double weight_sum{0.0};

        void add_sample(const radiometry::RGB<T>& rgb, T weight) {
            double w = static_cast<double>(weight);
            rgb_sum[0] += w * static_cast<double>(rgb[0]);
            rgb_sum[1] += w * static_cast<double>(rgb[1]);
            rgb_sum[2] += w * static_cast<double>(rgb[2]);
            weight_sum += w;
        }

        radiometry::RGB<T> resolve() const {
            if (math::is_zero(weight_sum)) {
                return radiometry::RGB<T>(T(0), T(0), T(0));
            }
            double inv_weight = 1.0 / weight_sum;
            return radiometry::RGB<T>(
                static_cast<T>(rgb_sum[0] * inv_weight),
                static_cast<T>(rgb_sum[1] * inv_weight),
                static_cast<T>(rgb_sum[2] * inv_weight)
            );
        }

        void clear() {
            rgb_sum = {0.0, 0.0, 0.0};
            weight_sum = 0.0;
        }   
    };

private:
    PixelSensorType m_pixel_sensor;
    std::vector<Pixel> m_pixels;

public:
    RGBFilm(
        const math::Vector<int, 2>& resolution,
        const math::Vector<T, 2>& physical_size,
        const PixelSensorType& pixel_sensor
    ) : Film<RGBFilm<T, PixelSensorType>, T>(resolution, physical_size),
        m_pixel_sensor(pixel_sensor) {
        int width = this->resolution().x();
        int height = this->resolution().y();
        math::assert_if(width <= 0 || height <= 0, "RGBFilm requires a positive resolution");
        m_pixels.resize(static_cast<std::size_t>(width) * static_cast<std::size_t>(height));
    }

    template<int N>
    void add_sample_impl(
        const math::Point<int, 2>& p_film,
        const radiometry::SampledSpectrum<T, N>& radiance,
        const radiometry::SampledWavelength<T, N>& wavelengths,
        const radiometry::SampledPdf<T, N>& pdf,
        T weight
    ) {
        Pixel& pixel = pixel_at(p_film);
        auto sensor_rgb = m_pixel_sensor.template radiance_to_sensor_rgb<N>(radiance, wavelengths, pdf);
        auto display_rgb = m_pixel_sensor.sensor_rgb_to_color_space_rgb(sensor_rgb);
        pixel.add_sample(display_rgb, weight);
    }

    template<typename SpectrumType>
    void add_sample_impl(
        const math::Point<int, 2>& p_film,
        const SpectrumType& radiance,
        T weight
    ) {
        Pixel& pixel = pixel_at(p_film);
        auto sensor_rgb = m_pixel_sensor.radiance_to_sensor_rgb(radiance);
        auto display_rgb = m_pixel_sensor.sensor_rgb_to_color_space_rgb(sensor_rgb);
        pixel.add_sample(display_rgb, weight);
    }

    void add_color_sample(
        const math::Point<int, 2>& p_film,
        const radiometry::RGB<T>& rgb,
        T weight
    ) {
        Pixel& pixel = pixel_at(p_film);
        pixel.add_sample(rgb, weight);
    }

    radiometry::RGB<T> get_pixel_rgb(const math::Point<int, 2>& p_film) const {
        return pixel_at(p_film).resolve();
    }

    void clear() {
        for (auto& pixel : m_pixels) {
            pixel.clear();
        }
    }

    const PixelSensorType& pixel_sensor() const {
        return m_pixel_sensor;
    }

    PixelSensorType& pixel_sensor() {
        return m_pixel_sensor;
    }

    const std::vector<Pixel>& pixels() const {
        return m_pixels;
    }

    std::vector<Pixel>& pixels() {
        return m_pixels;
    }

private:
    Pixel& pixel_at(const math::Point<int, 2>& p_film) {
        math::assert_if(!is_pixel_in_bounds(p_film), "Film sample coordinate out of range");
        return m_pixels[pixel_index(p_film)];
    }

    const Pixel& pixel_at(const math::Point<int, 2>& p_film) const {
        math::assert_if(!is_pixel_in_bounds(p_film), "Film sample coordinate out of range");
        return m_pixels[pixel_index(p_film)];
    }

    bool is_pixel_in_bounds(const math::Point<int, 2>& p_film) const {
        int width = this->resolution().x();
        int height = this->resolution().y();
        return p_film.x() >= 0 && p_film.x() < width &&
               p_film.y() >= 0 && p_film.y() < height;
    }

    std::size_t pixel_index(const math::Point<int, 2>& p_film) const {
        std::size_t width = static_cast<std::size_t>(this->resolution().x());
        return static_cast<std::size_t>(p_film.y()) * width + static_cast<std::size_t>(p_film.x());
    }
};

//TODO G-Buffer film

};
