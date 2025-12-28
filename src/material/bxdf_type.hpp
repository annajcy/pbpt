#pragma once

#include <variant>
#include "dielectric_bxdf.hpp"
#include "lambertian_bxdf.hpp"

namespace pbpt::material {

// --- AnyBxDF 定义 ---
template<typename T, int N>
using AnyBxDF = std::variant<
    LambertianBxDF<T, N>,
    DielectricBxDF<T, N>
>;

} // namespace pbpt::material