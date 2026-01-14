#pragma once

#include "math/point.hpp"
#include "../color_space.hpp"
#include "illuminant_spectrum.hpp"

namespace pbpt::radiometry::constant {

template<typename T>
inline static RGBColorSpace<T> sRGB(
    math::Point<T, 2> {0.64, 0.33},
    math::Point<T, 2> {0.3, 0.6},
    math::Point<T, 2> {0.15, 0.06},
    XYZ<T>::from_illuminant(CIE_D65_ilum<T>)
);

template<typename T>
inline static RGBColorSpace<T> DCI_P3(
    math::Point<T, 2> {0.68, 0.32},
    math::Point<T, 2> {0.265, 0.69},
    math::Point<T, 2> {0.15, 0.06},
    XYZ<T>::from_illuminant(CIE_D65_ilum<T>)
);

template<typename T>
inline static RGBColorSpace<T> Rec2020(
    math::Point<T, 2> {0.708, 0.292},
    math::Point<T, 2> {0.17, 0.797},
    math::Point<T, 2> {0.131, 0.046},
    XYZ<T>::from_illuminant(CIE_D65_ilum<T>)
);

template<typename T>
auto get_standard_color_space(const std::string& name) {
    if (name == "sRGB") {
        return sRGB<T>;
    } else if (name == "DCI-P3") {
        return DCI_P3<T>;
    } else if (name == "Rec.2020") {
        return Rec2020<T>;
    } else {
        throw std::runtime_error("Unknown standard color space: " + name);
    }
}

}; 
