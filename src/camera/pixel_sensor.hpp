#pragma once

#include "math/matrix.hpp"
#include "math/point.hpp"
#include "radiometry/color.hpp"
#include "radiometry/color_space.hpp"
#include "radiometry/constant/swatch_reflectances_spectrum.hpp"
#include "radiometry/constant/xyz_spectrum.hpp"
#include "radiometry/sampled_spectrum.hpp"

namespace pbpt::camera {

template<typename T>
inline constexpr math::Matrix<T, 3, 3> white_balance(
    math::Point<T, 2> src_chroma_xy,
    math::Point<T, 2> dst_chroma_xy
) {
    auto src_XYZ = radiometry::XYZ<T>::from_xyY(src_chroma_xy, T{1}), dst_XYZ = radiometry::XYZ<T>::from_xyY(dst_chroma_xy, T{1});
    auto src_LMS = radiometry::LMS<T>::from_xyz(src_XYZ), dst_LMS = radiometry::LMS<T>::from_xyz(dst_XYZ);
    T scale_X = dst_LMS.x() / src_LMS.x();
    T scale_Y = dst_LMS.y() / src_LMS.y();
    T scale_Z = dst_LMS.z() / src_LMS.z();
    math::Matrix<T, 3, 3> wb_matrix = math::Matrix<T, 3, 3>::zeros();
    wb_matrix[0][0] = scale_X;
    wb_matrix[1][1] = scale_Y;
    wb_matrix[2][2] = scale_Z;
    return  radiometry::LMS<T>::lms_to_xyz_matrix() * 
            wb_matrix * 
            radiometry::LMS<T>::xyz_to_lms_matrix();
}

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

    PixelSensor(
        const SceneIlluminantSpectrumType& scene_illuminant,
        const StandardIlluminantSpectrumType& standard_illuminant,
        const radiometry::RGBColorSpace<T>& color_space,
        T image_ratio = T{1.0}
    ) : m_scene_illuminant(scene_illuminant),
        m_standard_illuminant(standard_illuminant),
        m_sensor_response(
            radiometry::ResponseSpectrum<radiometry::constant::XYZSpectrumType<T>>(
                radiometry::constant::CIE_X<T>,
                radiometry::constant::CIE_Y<T>,
                radiometry::constant::CIE_Z<T>
            )
        ), 
        m_color_space(color_space),
        m_image_ratio(image_ratio) {
            math::Point<T, 2> src_chroma_xy = radiometry::XYZ<T>::from_illuminant(m_scene_illuminant).to_xy();
            math::Point<T, 2> dst_chroma_xy = radiometry::XYZ<T>::from_illuminant(m_standard_illuminant).to_xy();
            auto wb_matrix = white_balance<T>(src_chroma_xy, dst_chroma_xy);
            m_sensor_rgb_to_xyz = wb_matrix;
    }

    PixelSensor(
        const StandardIlluminantSpectrumType& standard_illuminant,
        const radiometry::RGBColorSpace<T>& color_space,
        T image_ratio = T{1.0}
    ) : m_scene_illuminant(standard_illuminant),
        m_standard_illuminant(standard_illuminant),
        m_sensor_response(
            radiometry::ResponseSpectrum<radiometry::constant::XYZSpectrumType<T>>(
                radiometry::constant::CIE_X<T>,
                radiometry::constant::CIE_Y<T>,
                radiometry::constant::CIE_Z<T>
            )
        ), 
        m_color_space(color_space),
        m_image_ratio(image_ratio) {
            m_sensor_rgb_to_xyz = math::Matrix<T, 3, 3>::identity();
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