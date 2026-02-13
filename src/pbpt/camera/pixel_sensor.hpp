/**
 * @file
 * @brief Pixel sensor model mapping spectral radiance to RGB/XYZ/color-space values.
 *
 * The sensor is modeled as a 3-channel device with spectral sensitivities
 * and an associated scene illuminant. Spectral radiance at a pixel is
 * integrated against these curves to produce sensor RGB values, which are
 * then linearly transformed into CIE XYZ and finally into a target RGB
 * color space.
 */
#pragma once

#include "pbpt/math/matrix.hpp"
#include "pbpt/math/point.hpp"
#include "pbpt/radiometry/color.hpp"
#include "pbpt/radiometry/color_space.hpp"
#include "pbpt/radiometry/constant/swatch_reflectances_spectrum.hpp"
#include "pbpt/radiometry/constant/xyz_spectrum.hpp"
#include "pbpt/radiometry/sampled_spectrum.hpp"

namespace pbpt::camera {

/**
 * @brief Pixel sensor that converts spectral radiance into RGB/XYZ values.
 *
 * The sensor is parameterized by a scene illuminant, a reference standard
 * illuminant, a 3-channel spectral response, and a target RGB color space.
 * Conceptually, for each sensor channel we integrate the product of
 * spectral radiance, scene illuminant and that channel's sensitivity
 * over wavelength to obtain the channel response.
 *
 * The resulting sensor RGB triplet is mapped to CIE XYZ via a 3×3 matrix
 * that is typically estimated by fitting a linear transform between
 * simulated sensor responses for a set of reference reflectances and their
 * ground-truth XYZ under a standard illuminant.
 *
 * @tparam T                           Scalar type (e.g. float or double).
 * @tparam SceneIlluminantSpectrumType Spectrum type of the scene illuminant.
 * @tparam StandardIlluminantSpectrumType Spectrum type of the standard illuminant.
 * @tparam SensorResponseSpectrumType  Spectrum type describing the 3-channel sensor response.
 */
template<typename T, typename SceneIlluminantSpectrumType, typename StandardIlluminantSpectrumType, typename SensorResponseSpectrumType>
class PixelSensor {
private:
    /// Spectral power distribution of the scene illuminant used for rendering.
    SceneIlluminantSpectrumType m_scene_illuminant;
    /// Reference standard illuminant used to define the XYZ color space (e.g. D65).
    StandardIlluminantSpectrumType m_standard_illuminant;   
    /// Target RGB color space into which XYZ values are converted.
    radiometry::RGBColorSpace<T> m_color_space;
    /// Spectral response of the 3 sensor channels.
    radiometry::ResponseSpectrum<SensorResponseSpectrumType> m_sensor_response;
    /// 3×3 matrix that maps sensor RGB to CIE XYZ tristimulus values.
    math::Matrix<T, 3, 3> m_sensor_rgb_to_xyz{};
    /// Global scale factor applied to sensor RGB (e.g. exposure/image ratio).
    T m_image_ratio{1.0};

public:
    PixelSensor() = default;
    
    /**
     * @brief Construct a pixel sensor with an explicit sensor response and calibration.
     *
     * This constructor simulates sensor RGB responses and reference XYZ values
     * for a set of standard reflectance swatches, then solves for a 3×3 matrix
     * that best maps the simulated sensor RGBs to the reference XYZ values
     * in a least-squares sense.
     *
     * For each swatch, we compute:
     * - a 3-component sensor RGB response by integrating reflectance *
     *   scene_illuminant * sensor_response over wavelength;
     * - a 3-component reference XYZ response by integrating reflectance *
     *   standard_illuminant * XYZ color-matching functions.
     *
     * The calibration matrix is then chosen so that M * rgb_camera
     * is as close as possible (in least-squares sense) to xyz_output
     * over all swatches.
     *
     * @param scene_illuminant    Spectrum of the scene illuminant.
     * @param standard_illuminant Spectrum of the reference standard illuminant.
     * @param color_space         Target RGB color space definition.
     * @param sensor_response     Spectral response of the 3-channel sensor.
     * @param image_ratio         Global scale factor applied to sensor RGB.
     */
    PixelSensor(
        const SceneIlluminantSpectrumType& scene_illuminant,
        const StandardIlluminantSpectrumType& standard_illuminant,
        const radiometry::RGBColorSpace<T>& color_space,
        const radiometry::ResponseSpectrum<SensorResponseSpectrumType>& sensor_response,
        T image_ratio = T{1.0}
    ) : m_scene_illuminant(scene_illuminant),
        m_standard_illuminant(standard_illuminant),
        m_sensor_response(sensor_response), 
        m_color_space(color_space),
        m_image_ratio(image_ratio) {

            math::Matrix<T, 3, radiometry::constant::swatch_reflectances_count> rgb_camera{};
            for (int i = 0; i < radiometry::constant::swatch_reflectances_count; ++i) {
                auto reflectance = radiometry::constant::swatch_reflectances<T>[i];
                auto rgb = radiometry::project_reflectance<T, radiometry::RGB>(
                    reflectance, 
                    m_scene_illuminant, 
                    m_sensor_response, 
                    false
                );
                rgb_camera[0][i] = rgb[0];
                rgb_camera[1][i] = rgb[1];
                rgb_camera[2][i] = rgb[2];
            }

            math::Matrix<T, 3, radiometry::constant::swatch_reflectances_count> xyz_output{};
            for (int i = 0; i < radiometry::constant::swatch_reflectances_count; ++i) {
                auto reflectance = radiometry::constant::swatch_reflectances<T>[i];
                auto xyz = radiometry::project_reflectance<T, radiometry::XYZ>(
                    reflectance, 
                    m_standard_illuminant, 
                    radiometry::ResponseSpectrum<radiometry::constant::XYZSpectrumType<T>>(
                        radiometry::constant::CIE_X<T>,
                        radiometry::constant::CIE_Y<T>,
                        radiometry::constant::CIE_Z<T>
                    ), 
                    false);
                xyz_output[0][i] = xyz[0];
                xyz_output[1][i] = xyz[1];
                xyz_output[2][i] = xyz[2];  
            }

            m_sensor_rgb_to_xyz = math::solve_LMS(
                rgb_camera,
                xyz_output
            );
    }
    
    /**
     * @brief Convert spectral radiance to sensor RGB using full spectra.
     *
     * Conceptually this evaluates, for each channel, the integral over
     * wavelength of:
     * radiance(lambda) * scene_illuminant(lambda) * sensor_response(lambda).
     * The resulting sensor RGB is then multiplied by the image_ratio factor.
     *
     * @tparam SpectrumType Type representing a spectral radiance distribution.
     * @param radiance      Spectral radiance at the pixel.
     * @return Sensor RGB triplet proportional to the captured signal.
     */
    template<typename SpectrumType>
    radiometry::RGB<T> radiance_to_sensor_rgb(
        const SpectrumType& radiance
    ) const {
        auto sensor_rgb = radiometry::project_emission<T, radiometry::RGB, SpectrumType, SceneIlluminantSpectrumType, SensorResponseSpectrumType>(
            radiance,
            m_scene_illuminant,
            m_sensor_response
        );
        return sensor_rgb * m_image_ratio;
    }

    /**
     * @brief Convert sampled spectral radiance to sensor RGB using Monte Carlo integration.
     *
     * This overload approximates the spectral integral by Monte Carlo sampling.
     * For N sampled wavelengths lambda_i with sampling PDF p(lambda_i),
     * the sensor RGB is estimated by averaging
     *   radiance(lambda_i) * scene_illuminant(lambda_i)
     *   * sensor_response(lambda_i) / p(lambda_i)
     * over all samples. The result is then scaled by the image_ratio factor.
     *
     * @tparam N Number of spectral samples.
     * @param radiance     Sampled spectral radiance values.
     * @param wavelengths  Sampled wavelengths corresponding to @p radiance.
     * @param pdf          Sampling PDF for each wavelength sample.
     * @return Sensor RGB triplet estimated from the sampled spectrum.
     */
    template<int N>
    radiometry::RGB<T> radiance_to_sensor_rgb(
        const radiometry::SampledSpectrum<T, N>& radiance, 
        const radiometry::SampledWavelength<T, N>& wavelengths,
        const radiometry::SampledPdf<T, N> &pdf
    ) const {
        auto sampled_illuminant = m_scene_illuminant.template sample<N>(wavelengths);
        auto sensor_rgb = radiometry::project_sampled_emission<T, radiometry::RGB, N, radiometry::SampledSpectrum<T, N>, radiometry::SampledSpectrum<T, N>, SensorResponseSpectrumType>(
            radiance,
            sampled_illuminant,
            wavelengths,
            pdf,
            m_sensor_response
        );
        return sensor_rgb * m_image_ratio;
    }

    /**
     * @brief Get the 3×3 matrix mapping sensor RGB to CIE XYZ.
     *
     * The matrix is either obtained by least-squares fitting on reference
     * color swatches or constructed as a white-balance/identity matrix,
     * depending on which constructor was used.
     *
     * @return Sensor-RGB-to-XYZ transformation matrix.
     */
    math::Matrix<T, 3, 3> sensor_rgb_to_xyz_matrix() const {
        return m_sensor_rgb_to_xyz;
    }

    /**
     * @brief Transform sensor RGB values to CIE XYZ using a 3×3 matrix.
     *
     * Applies the precomputed 3×3 matrix so that the output XYZ is
     * a linear transform of the input sensor RGB.
     *
     * @param sensor_rgb Sensor RGB values.
     * @return Corresponding CIE XYZ tristimulus values.
     */
    radiometry::XYZ<T> sensor_rgb_to_xyz(const radiometry::RGB<T>& sensor_rgb) const {
        return m_sensor_rgb_to_xyz * sensor_rgb;
    }

    /**
     * @brief Transform sensor RGB values directly to the target RGB color space.
     *
     * This is equivalent to first converting to XYZ using
     * sensor_rgb_to_xyz() and then applying the RGB color space's
     * inverse transformation to obtain linear RGB values.
     *
     * @param sensor_rgb Sensor RGB values.
     * @return Linear RGB values in the target color space.
     */
    radiometry::RGB<T> sensor_rgb_to_color_space_rgb(const radiometry::RGB<T>& sensor_rgb) const {
        radiometry::XYZ<T> xyz = sensor_rgb_to_xyz(sensor_rgb);
        return m_color_space.to_rgb(xyz);
    }

};

};
