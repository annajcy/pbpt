#pragma once

#include "math/matrix.hpp"
#include "radiometry/color.hpp"
#include "radiometry/color_space.hpp"
#include "radiometry/spectrum_distribution.hpp"
#include "radiometry/spectrum_utils.hpp"

namespace pbpt::camera {

template<typename T, typename SceneIlluminantType, typename StandardIlluminantType>
class PixelSensor {
private:
    SceneIlluminantType m_scene_illuminant;
    StandardIlluminantType m_standard_illuminant;   
    radiometry::RGBColorSpace<T> m_color_space;

    radiometry::TabularSpectrumDistribution<T, radiometry::lambda_min<int>, radiometry::lambda_max<int>> m_r_response;
    radiometry::TabularSpectrumDistribution<T, radiometry::lambda_min<int>, radiometry::lambda_max<int>> m_g_response;
    radiometry::TabularSpectrumDistribution<T, radiometry::lambda_min<int>, radiometry::lambda_max<int>> m_b_response;

    math::Matrix<T, 3, 3> m_sensor_rgb_to_xyz{};

public:
    PixelSensor(
        const SceneIlluminantType& scene_illuminant,
        const StandardIlluminantType& standard_illuminant,
        const radiometry::RGBColorSpace<T>& color_space,
        const radiometry::TabularSpectrumDistribution<T, radiometry::lambda_min<int>, radiometry::lambda_max<int>>& r_response,
        const radiometry::TabularSpectrumDistribution<T, radiometry::lambda_min<int>, radiometry::lambda_max<int>>& g_response,
        const radiometry::TabularSpectrumDistribution<T, radiometry::lambda_min<int>, radiometry::lambda_max<int>>& b_response
    ) : m_scene_illuminant(scene_illuminant),
        m_color_space(color_space),
        m_r_response(r_response),
        m_g_response(g_response),
        m_b_response(b_response) {
            //TODO: 计算m_sensor_rgb_to_xyz
    }

};


};