#pragma once

#include <variant>

#include "pbpt/camera/plugin/film/hdr_film.hpp"
#include "pbpt/camera/pixel_sensor.hpp"
#include "pbpt/radiometry/constant/illuminant_spectrum.hpp"
#include "pbpt/radiometry/constant/standard_color_spaces.hpp"

namespace pbpt::camera {

// 2. 锁定 PixelSensor 和 Film 的组合
// 绝大多数情况下，我们不需要在每条光线上切换 Sensor 的物理光谱特性
// 我们将其锁定为标准的 "CIE D65 场景照明 + CIE XYZ 传感器响应"
template <typename T>
using StandardPixelSensor =
    camera::PixelSensor<T, radiometry::constant::CIED65SpectrumType<T>, radiometry::constant::CIED65SpectrumType<T>,
                        radiometry::constant::XYZSpectrumType<T>>;

template <typename T>
inline auto make_standard_pixel_sensor(T ratio = T(1)) {
    return StandardPixelSensor<T>(
        radiometry::constant::CIE_D65_ilum<T>, radiometry::constant::CIE_D65_ilum<T>, radiometry::constant::sRGB<T>,
        radiometry::ResponseSpectrum<radiometry::constant::XYZSpectrumType<T>>(
            radiometry::constant::CIE_X<T>, radiometry::constant::CIE_Y<T>, radiometry::constant::CIE_Z<T>),
        ratio);
}

// 具体 Film 类型定义
template <typename T>
using StandardHDRFilm = camera::HDRFilm<T, StandardPixelSensor<T>>;

// Film Variant
// 支持多种 Film 类型，例如 HDRFilm, SpectrumFilm, GBufferFilm 等
template <typename T>
using AnyFilm = std::variant<StandardHDRFilm<T>>;

}  // namespace pbpt::camera
