/**
 * @file
 * @brief Film abstractions for accumulating radiance samples into pixels.
 */
#pragma once

#include <array>
#include <cstddef>
#include <vector>

#include "math/point.hpp"
#include "math/utils.hpp"
#include "math/vector.hpp"

#include "radiometry/color.hpp"
#include "radiometry/sampled_spectrum.hpp"

namespace pbpt::camera {

/**
 * @brief CRTP base class for film implementations.
 *
 * A film represents a 2D discrete sensor that accumulates radiance samples
 * at integer pixel coordinates. Derived classes implement how spectral
 * radiance is converted to stored pixel values (e.g. RGB, G-buffer, etc.).
 *
 * @tparam Derived Concrete film type.
 * @tparam T       Scalar type (e.g. float or double).
 */
template<typename Derived, typename T>
class Film {
private:
    /// Pixel resolution (width, height).
    math::Vector<int, 2> m_resolution{};
    /// Physical size of the film (e.g. in millimeters) in x and y.
    math::Vector<T, 2> m_physical_size{};

public:
    Film() = default;
    
    /**
     * @brief Construct a film with given resolution and physical size.
     *
     * @param resolution    Pixel resolution (width, height).
     * @param physical_size Physical size of the film in x and y.
     */
    Film(const math::Vector<int, 2>& resolution, const math::Vector<T, 2>& physical_size)
        : m_resolution(resolution), m_physical_size(physical_size) {}

    /// Get the film resolution in pixels.
    const math::Vector<int, 2>& resolution() const { return m_resolution; }
    /// Get the physical size of the film.
    const math::Vector<T, 2>& physical_size() const { return m_physical_size; }

    /**
     * @brief Add a spectral sample using Monte Carlo sampled wavelengths.
     *
     * The derived film decides how to convert the sampled spectral radiance
     * plus sampling PDF into pixel values (e.g. via a pixel sensor model).
     *
     * @tparam N Number of spectral samples.
     * @param p_film      Pixel coordinate.
     * @param radiance    Sampled spectral radiance.
     * @param wavelengths Sampled wavelengths.
     * @param pdf         PDF for each wavelength sample.
     * @param weight      Monte Carlo weight applied to this sample.
     */
    template<int N>
    void add_sample(
        const math::Point<int, 2>& p_film,
        const radiometry::SampledSpectrum<T, N>& radiance,
        const radiometry::SampledWavelength<T, N>& wavelengths,
        const radiometry::SampledPdf<T, N>& pdf,
        T weight
    ) {
        as_derived().template add_sample_impl<N>(
            p_film, 
            radiance, 
            wavelengths, 
            pdf, 
            weight
        );
    }

    /**
     * @brief Add a spectral sample using a full spectrum type.
     *
     * This overload is used when the spectrum type can be integrated
     * directly without Monte Carlo sampling of wavelengths.
     *
     * @tparam SpectrumType Type representing a spectral radiance distribution.
     * @param p_film   Pixel coordinate.
     * @param radiance Spectral radiance.
     * @param weight   Monte Carlo weight applied to this sample.
     */
    template<typename SpectrumType>
    void add_sample(
        const math::Point<int, 2>& p_film,
        const SpectrumType& radiance,
        T weight
    ) {
        as_derived().add_sample_impl(
            p_film, 
            radiance, 
            weight
        );
    }

    /// Access the derived implementation (non-const).
    Derived& as_derived() {
        return static_cast<Derived&>(*this);
    }

    /// Access the derived implementation (const).
    const Derived& as_derived() const {
        return static_cast<const Derived&>(*this);
    }
};

/**
 * @brief Film that accumulates RGB values produced by a pixel sensor.
 *
 * This film stores for each pixel a running weighted sum of RGB values
 * and the sum of weights. The final pixel color is obtained by dividing
 * the accumulated RGB sum by the weight sum.
 *
 * @tparam T              Scalar type (e.g. float or double).
 * @tparam PixelSensorType Pixel sensor type used to convert spectra to RGB.
 */
template<typename T, typename PixelSensorType>
class HDRFilm : public Film<HDRFilm<T, PixelSensorType>, T> {
public:

    /**
     * @brief Per-pixel accumulation buffer.
     *
     * Stores a running sum of RGB values (in double precision for better
     * numerical stability) and the total weight of samples contributing
     * to this pixel.
     */
    struct Pixel {
        /// Accumulated weighted RGB sum.
        std::array<T, 3> rgb_sum{0.0, 0.0, 0.0};
        /// Sum of sample weights.
        double weight_sum{0.0};

        /**
         * @brief Accumulate a single RGB sample into this pixel.
         *
         * @param rgb    Sampled RGB value.
         * @param weight Monte Carlo weight associated with the sample.
         */
        void add_sample(const radiometry::RGB<T>& rgb, T weight) {
            double w = static_cast<double>(weight);
            rgb_sum[0] += w * static_cast<double>(rgb[0]);
            rgb_sum[1] += w * static_cast<double>(rgb[1]);
            rgb_sum[2] += w * static_cast<double>(rgb[2]);
            weight_sum += w;
        }

        /**
         * @brief Resolve the final pixel color.
         *
         * Returns the average RGB value by dividing the accumulated sum
         * by the total weight. If no samples have contributed, returns
         * black (0, 0, 0).
         *
         * @return Final RGB value for this pixel.
         */
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

        /// Reset accumulation to zero.
        void clear() {
            rgb_sum = {0.0, 0.0, 0.0};
            weight_sum = 0.0;
        }   
    };

private:
    /// Pixel sensor used to convert spectral radiance to RGB.
    PixelSensorType m_pixel_sensor;
    /// Linear buffer of per-pixel accumulators.
    std::vector<Pixel> m_pixels;

public:
    HDRFilm() = default;
    
    /**
     * @brief Construct an RGB film with a given resolution and sensor.
     *
     * @param resolution    Pixel resolution (width, height).
     * @param physical_size Physical size of the film.
     * @param pixel_sensor  Pixel sensor used for spectral to RGB conversion.
     */
    HDRFilm(
        const math::Vector<int, 2>& resolution,
        const math::Vector<T, 2>& physical_size,
        const PixelSensorType& pixel_sensor
    ) : Film<HDRFilm<T, PixelSensorType>, T>(resolution, physical_size),
        m_pixel_sensor(pixel_sensor) {
        int width = this->resolution().x();
        int height = this->resolution().y();
        math::assert_if(width <= 0 || height <= 0, "RGBFilm requires a positive resolution");
        m_pixels.resize(static_cast<std::size_t>(width) * static_cast<std::size_t>(height));
    }

    /**
     * @brief Implementation of spectral sample accumulation for sampled spectra.
     *
     * The spectrum is first converted to sensor RGB using the pixel sensor
     * and then mapped into the target color space. The resulting RGB is
     * accumulated into the pixel buffer.
     */
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

    /**
     * @brief Implementation of spectral sample accumulation for full spectra.
     *
     * This overload uses the pixel sensor's direct spectrum integration
     * method to convert radiance to sensor RGB, then to display RGB.
     */
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

    /**
     * @brief Directly accumulate an RGB sample into the film.
     *
     * This is useful when the radiance has already been converted to
     * display RGB outside the film (for example, when debugging).
     */
    void add_color_sample(
        const math::Point<int, 2>& p_film,
        const radiometry::RGB<T>& rgb,
        T weight
    ) {
        Pixel& pixel = pixel_at(p_film);
        pixel.add_sample(rgb, weight);
    }

    /**
     * @brief Get the resolved RGB value for a pixel.
     *
     * @param p_film Pixel coordinate.
     * @return Averaged RGB value for that pixel.
     */
    radiometry::RGB<T> get_pixel_rgb(const math::Point<int, 2>& p_film) const {
        return pixel_at(p_film).resolve();
    }

    /// Clear all pixel accumulators.
    void clear() {
        for (auto& pixel : m_pixels) {
            pixel.clear();
        }
    }

    /// Get a const reference to the pixel sensor.
    const PixelSensorType& pixel_sensor() const {
        return m_pixel_sensor;
    }

    /// Get a mutable reference to the pixel sensor.
    PixelSensorType& pixel_sensor() {
        return m_pixel_sensor;
    }

    /// Get a const reference to the internal pixel buffer.
    const std::vector<Pixel>& pixels() const {
        return m_pixels;
    }

    /// Get a mutable reference to the internal pixel buffer.
    std::vector<Pixel>& pixels() {
        return m_pixels;
    }

private:
    /// Access a pixel by integer coordinate (with bounds checks).
    Pixel& pixel_at(const math::Point<int, 2>& p_film) {
        math::assert_if(!is_pixel_in_bounds(p_film), "Film sample coordinate out of range");
        return m_pixels[pixel_index(p_film)];
    }

    /// Access a pixel by integer coordinate (const, with bounds checks).
    const Pixel& pixel_at(const math::Point<int, 2>& p_film) const {
        math::assert_if(!is_pixel_in_bounds(p_film), "Film sample coordinate out of range");
        return m_pixels[pixel_index(p_film)];
    }

    /// Check if a pixel coordinate lies inside the film bounds.
    bool is_pixel_in_bounds(const math::Point<int, 2>& p_film) const {
        int width = this->resolution().x();
        int height = this->resolution().y();
        return p_film.x() >= 0 && p_film.x() < width &&
               p_film.y() >= 0 && p_film.y() < height;
    }

    /// Convert a 2D pixel coordinate to a linear index.
    std::size_t pixel_index(const math::Point<int, 2>& p_film) const {
        std::size_t width = static_cast<std::size_t>(this->resolution().x());
        return static_cast<std::size_t>(p_film.y()) * width + static_cast<std::size_t>(p_film.x());
    }
};

//TODO G-Buffer film

};
