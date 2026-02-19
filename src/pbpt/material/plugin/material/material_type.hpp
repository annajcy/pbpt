#pragma once

#include <variant>
#include "pbpt/material/plugin/material/lambertian_material.hpp"
#include "pbpt/material/plugin/material/dielectric_material.hpp"
#include "pbpt/material/plugin/material/dielectric_specular_material.hpp"
#include "pbpt/material/plugin/material/dielectric_rough_material.hpp"
#include "pbpt/material/plugin/material/conductor_material.hpp"
#include "pbpt/material/plugin/material/conductor_specular_material.hpp"
#include "pbpt/material/plugin/material/conductor_rough_material.hpp"
#include "pbpt/utils/library.hpp"

namespace pbpt::material {

template <typename T>
using AnyMaterial = std::variant<LambertianMaterial<T>, DielectricMaterial<T>, DielectricSpecularMaterial<T>,
                                 DielectricRoughMaterial<T>, ConductorMaterial<T>, ConductorSpecularMaterial<T>,
                                 ConductorRoughMaterial<T>>;

template <typename T>
using AnyMaterialLibrary = utils::Library<T, AnyMaterial<T>>;

template <typename T>
using NamedAnyMaterialLibrary = utils::NamedLibrary<T, AnyMaterial<T>>;

}  // namespace pbpt::material
