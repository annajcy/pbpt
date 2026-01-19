#pragma once

#include <variant>
#include "radiometry/constant/illuminant_spectrum.hpp"
#include "radiometry/spectrum_distribution.hpp"
#include "utils/library.hpp"

namespace pbpt::radiometry {
// Light Variant
// 假设光谱分布类型也是标准的 PiecewiseLinear * type(D65)
template<typename T>
using StandardEmissionSpectrum = radiometry::MultipliedSpectrumDistribution<T, 
    radiometry::PiecewiseLinearSpectrumDistribution<T>, 
    radiometry::constant::CIED65SpectrumType<T>
>;

template<typename T>
using EmissionSpectrumLibrary = utils::Library<T, StandardEmissionSpectrum<T>>;

template<typename T>
using NamedEmissionSpectrumLibrary = utils::NamedLibrary<T, StandardEmissionSpectrum<T>>;

template<typename T>
using StandardReflectanceSpectrum = radiometry::PiecewiseLinearSpectrumDistribution<T>;

template<typename T>
using ReflectanceSpectrumLibrary = utils::Library<T, StandardReflectanceSpectrum<T>>;

template<typename T>
using NamedReflectanceSpectrumLibrary = utils::NamedLibrary<T, StandardReflectanceSpectrum<T>>;

}
