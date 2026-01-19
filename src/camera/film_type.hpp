#pragma once

#include <variant>

#include "camera/film.hpp"
#include "camera/pixel_sensor.hpp"
#include "radiometry/constant/illuminant_spectrum.hpp"

namespace pbpt::camera {

// 2. 锁定 PixelSensor 和 Film 的组合
// 绝大多数情况下，我们不需要在每条光线上切换 Sensor 的物理光谱特性
// 我们将其锁定为标准的 "CIE D65 场景照明 + CIE XYZ 传感器响应"
template<typename T>
using StandardPixelSensor = camera::PixelSensor<T,
    radiometry::constant::CIED65SpectrumType<T>,
    radiometry::constant::CIED65SpectrumType<T>,
    radiometry::constant::XYZSpectrumType<T>
>;

// 具体 Film 类型定义
template<typename T>
using StandardHDRFilm = camera::HDRFilm<T, StandardPixelSensor<T>>;

// Film Variant
// 支持多种 Film 类型，例如 HDRFilm, SpectrumFilm, GBufferFilm 等
template<typename T>
using AnyFilm = std::variant<
    StandardHDRFilm<T>
>;
    
}