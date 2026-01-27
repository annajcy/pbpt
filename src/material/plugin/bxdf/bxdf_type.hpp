#pragma once

#include <variant>

#include "material/plugin/bxdf/conductor_specular_bxdf.hpp"
#include "material/plugin/bxdf/dielectric_specular_bxdf.hpp"
#include "material/plugin/bxdf/lambertian_bxdf.hpp"

namespace pbpt::material {

// --- AnyBxDF 定义 ---
template<typename T, int N>
using AnyBxDF = std::variant<
    LambertianBxDF<T, N>,
    DielectricSpecularBxDF<T, N>,
    ConductorSpecularBxDF<T, N>
>;

} // namespace pbpt::material
