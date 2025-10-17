#pragma once

#include <tuple>
#include "math/matrix.hpp"
#include "radiometry/color.hpp"
#include "radiometry/color_space.hpp"
#include "radiometry/sampled_spectrum.hpp"
#include "radiometry/spectrum_distribution.hpp"

namespace pbpt::camera {

template<typename ResponseSpectrumType>
struct RGBResponseSpectrum {
    ResponseSpectrumType r;
    ResponseSpectrumType g;
    ResponseSpectrumType b;
};

template<typename T, typename SceneIlluminantSpectrumType, typename StandardIlluminantSpectrumType, typename SensorSpectrumType>
class PixelSensor {
private:
    SceneIlluminantSpectrumType m_scene_illuminant;
    StandardIlluminantSpectrumType m_standard_illuminant;   
    radiometry::RGBColorSpace<T> m_color_space;

    RGBResponseSpectrum<SensorSpectrumType> m_sensor_response;
    math::Matrix<T, 3, 3> m_sensor_rgb_to_xyz{};

    T m_image_ratio{1.0};

public:
    PixelSensor(
        const SceneIlluminantSpectrumType& scene_illuminant,
        const StandardIlluminantSpectrumType& standard_illuminant,
        const radiometry::RGBColorSpace<T>& color_space,
        const RGBResponseSpectrum<SensorSpectrumType>& sensor_response,
        T image_ratio = T{1.0}
    ) : m_scene_illuminant(scene_illuminant),
        m_standard_illuminant(standard_illuminant),
        m_sensor_response(sensor_response), 
        m_color_space(color_space),
        m_image_ratio(image_ratio) {
            //TODO: 计算m_sensor_rgb_to_xyz
    }

    template<int N>
    radiometry::RGB<T> radiance_to_sensor_rgb(
        const radiometry::SampledSpectrum<T, N>& radiance, 
        const radiometry::SampledWavelength<T, N>& wavelengths,
        const radiometry::SampledPdf<T, N> &pdf
    ) const {
        auto inv_pdf = pdf.inv();
        return m_image_ratio * radiometry::RGB<T>(
            (m_sensor_response.r.sample(wavelengths) * radiance * inv_pdf).average(),
            (m_sensor_response.g.sample(wavelengths) * radiance * inv_pdf).average(),
            (m_sensor_response.b.sample(wavelengths) * radiance * inv_pdf).average()
        );
    }

    math::Matrix<T, 3, 3> sensor_rgb_to_xyz_matrix() const {
        return m_sensor_rgb_to_xyz;
    }

    radiometry::XYZ<T> sensor_rgb_to_xyz(const radiometry::RGB<T>& sensor_rgb) const {
        return m_sensor_rgb_to_xyz * sensor_rgb;
    }

    radiometry::RGB<T> sensor_rgb_to_color_space_rgb(const radiometry::RGB<T>& sensor_rgb) const {
        radiometry::XYZ<T> xyz = sensor_rgb_to_xyz(sensor_rgb);
        return m_color_space.to_rgb(xyz);
    }

    template<template <typename> class Color, typename ReflectanceSpectrumType, typename IlluminantSpectrumType, typename ResponseSpectrumType> 
    Color<T> project_reflectance(
        const ReflectanceSpectrumType& reflectance,
        const IlluminantSpectrumType& illuminant,
        const RGBResponseSpectrum<SensorSpectrumType>& response
    ) const {
        T r{}, g{}, b{};
        T g_integral{};
        for (int lambda = radiometry::lambda_min<int>; lambda <= radiometry::lambda_max<int>; ++lambda) {
            g_integral += illuminant.at(lambda) * response.g.at(lambda);
            r += reflectance.at(lambda) * illuminant.at(lambda) * response.r.at(lambda);
            g += reflectance.at(lambda) * illuminant.at(lambda) * response.g.at(lambda);
            b += reflectance.at(lambda) * illuminant.at(lambda) * response.b.at(lambda);
        }
        return Color<T> (
            r / g_integral,
            g / g_integral,
            b / g_integral
        );
    }

};


};