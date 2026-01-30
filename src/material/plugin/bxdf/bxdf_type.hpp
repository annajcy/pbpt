#pragma once

#include <variant>

#include "material/plugin/bxdf/conductor_rough_bxdf.hpp"
#include "material/plugin/bxdf/conductor_specular_bxdf.hpp"
#include "material/plugin/bxdf/dielectric_rough_bxdf.hpp"
#include "material/plugin/bxdf/dielectric_specular_bxdf.hpp"
#include "material/plugin/bxdf/lambertian_bxdf.hpp"

namespace pbpt::material {

// --- AnyBxDF 定义 ---
template<typename T, int N>
using AnyBxDF = std::variant<
    LambertianBxDF<T, N>,
    DielectricSpecularBxDF<T, N>,
    DielectricRoughBxDF<T, N>,
    ConductorSpecularBxDF<T, N>,
    ConductorRoughBxDF<T, N>
>;

} // namespace pbpt::material
