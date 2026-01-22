#pragma once

#include <variant>
#include "area_light.hpp"
#include "radiometry/plugin/spectrum_distribution/spectrum_distribution_type.hpp"
#include "shape/plugin/shape/triangle.hpp"
#include "utils/library.hpp"

namespace pbpt::light {

template<typename T>
using StandardAreaLightType = light::AreaLight<T, 
    shape::Triangle<T>, 
    radiometry::StandardEmissionSpectrum<T>
>;

template<typename T>
using AnyLight = std::variant<
    StandardAreaLightType<T>
>;

template<typename T>
using AnyLightLibrary = utils::Library<T, AnyLight<T>>;

template<typename T>
using NamedAnyLightLibrary = utils::NamedLibrary<T, AnyLight<T>>;

}
