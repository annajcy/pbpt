#pragma once

#include "camera/film.hpp"

namespace pbpt::camera {

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
    friend class Film<HDRFilm<T, PixelSensorType>, T>;

private:
    /// Pixel sensor used to convert spectral radiance to RGB.
    PixelSensorType m_pixel_sensor;
    /// Linear buffer of per-pixel accumulators.
    std::vector<Pixel<T>> m_pixels;

public:
    HDRFilm() = default;

    /**
     * @brief Construct an RGB film with a given resolution and sensor.
     *
     * @param resolution    Pixel resolution (width, height).
     * @param pixel_sensor  Pixel sensor used for spectral to RGB conversion.
     */
    HDRFilm(
        const math::Vector<int, 2>& resolution,
        const PixelSensorType& pixel_sensor
    ) : Film<HDRFilm<T, PixelSensorType>, T>(resolution),
        m_pixel_sensor(pixel_sensor) {
        int width = this->resolution().x();
        int height = this->resolution().y();
        math::assert_if(width <= 0 || height <= 0, "RGBFilm requires a positive resolution");
        m_pixels.resize(static_cast<std::size_t>(width) * static_cast<std::size_t>(height));
    }

    /**
     * @brief Construct an RGB film from a camera and sensor.
     *
     * @param camera        Camera object to derive resolution from.
     * @param pixel_sensor  Pixel sensor used for spectral to RGB conversion.
     */
    template <typename CameraType>
    HDRFilm(
        const CameraType& camera,
        const PixelSensorType& pixel_sensor
    ) : Film<HDRFilm<T, PixelSensorType>, T>(camera),
        m_pixel_sensor(pixel_sensor) {
        int width = this->resolution().x();
        int height = this->resolution().y();
        math::assert_if(width <= 0 || height <= 0, "RGBFilm requires a positive resolution");
        m_pixels.resize(static_cast<std::size_t>(width) * static_cast<std::size_t>(height));
    }

private:
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
        Pixel<T>& pixel = pixel_at(p_film);
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
        Pixel<T>& pixel = pixel_at(p_film);
        auto sensor_rgb = m_pixel_sensor.radiance_to_sensor_rgb(radiance);
        auto display_rgb = m_pixel_sensor.sensor_rgb_to_color_space_rgb(sensor_rgb);
        pixel.add_sample(display_rgb, weight);
    }

     /**
     * @brief Develop the film into an Image.
     */
    texture::ImageN<T, 3> develop_impl() const {
        int width = this->resolution().x();
        int height = this->resolution().y();
        texture::ImageN<T, 3> image(width, height);

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                math::Point<int, 2> p(x, y);
                auto rgb = get_pixel_rgb(p);
                image.get_pixel(x, y) = math::Vector<T, 3>(rgb.r(), rgb.g(), rgb.b());
            }
        }
        return image;
    }

public:
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
        Pixel<T>& pixel = pixel_at(p_film);
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
    const std::vector<Pixel<T>>& pixels() const {
        return m_pixels;
    }

    /// Get a mutable reference to the internal pixel buffer.
    std::vector<Pixel<T>>& pixels() {
        return m_pixels;
    }

private:
    /// Access a pixel by integer coordinate (with bounds checks).
    Pixel<T>& pixel_at(const math::Point<int, 2>& p_film) {
        math::assert_if(!is_pixel_in_bounds(p_film), "Film sample coordinate out of range");
        return m_pixels[pixel_index(p_film)];
    }

    /// Access a pixel by integer coordinate (const, with bounds checks).
    const Pixel<T>& pixel_at(const math::Point<int, 2>& p_film) const {
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

}  // namespace pbpt::camera
