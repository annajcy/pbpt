#pragma once

#include "math/matrix.hpp"
#include "radiometry/color.hpp"
#include "radiometry/color_space.hpp"
#include "radiometry/sampled_spectrum.hpp"

namespace pbpt::camera {

template<typename T, typename SceneIlluminantSpectrumType, typename StandardIlluminantSpectrumType, typename SensorResponseSpectrumType>
class PixelSensor {
private:
    SceneIlluminantSpectrumType m_scene_illuminant;
    StandardIlluminantSpectrumType m_standard_illuminant;   
    radiometry::RGBColorSpace<T> m_color_space;
    radiometry::ResponseSpectrum<SensorResponseSpectrumType> m_sensor_response;
    math::Matrix<T, 3, 3> m_sensor_rgb_to_xyz{};
    T m_image_ratio{1.0};

public:
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
            //TODO: 计算m_sensor_rgb_to_xyz
    }

    template<int N>
    radiometry::RGB<T> radiance_to_sensor_rgb(
        const radiometry::SampledSpectrum<T, N>& radiance, 
        const radiometry::SampledWavelength<T, N>& wavelengths,
        const radiometry::SampledPdf<T, N> &pdf
    ) const {
        auto sensor_rgb = radiometry::project_sampled_spectrum<T, radiometry::RGB, N, SensorResponseSpectrumType>(
            radiance,
            wavelengths,
            pdf,
            m_sensor_response
        );
        return sensor_rgb * m_image_ratio;
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

};


};