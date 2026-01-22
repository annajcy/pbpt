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
#include "texture/image.hpp"

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

public:
    Film() = default;
    
    /**
     * @brief Construct a film with given resolution.
     *
     * @param resolution Pixel resolution (width, height).
     */
    explicit Film(const math::Vector<int, 2>& resolution)
        : m_resolution(resolution) {}

    /**
     * @brief Construct a film from a camera.
     * 
     * Infers film resolution from the camera.
     */
    template <typename CameraType>
    explicit Film(const CameraType& camera) 
        : m_resolution(camera.film_resolution()) {}

    /// Get the film resolution in pixels.
    const math::Vector<int, 2>& resolution() const { return m_resolution; }

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

    /**
     * @brief Develop the film into an image-like object.
     * 
     * The return type depends on the specific Film implementation
     * (e.g. ImageN<T, 3> for HDRFilm).
     */
    auto develop() const {
        return as_derived().develop_impl();
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
* @brief Per-pixel accumulation buffer.
*
* Stores a running sum of RGB values (in double precision for better
* numerical stability) and the total weight of samples contributing
* to this pixel.
*/
template<typename T>
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

};
