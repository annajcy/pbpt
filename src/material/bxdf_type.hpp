#pragma once

#include <variant>
#include "bxdf.hpp"

namespace pbpt::material {

// --- AnyBxDF 定义 ---
template<typename T, int N>
using AnyBxDF = std::variant<
    LambertianBxDF<T, N>
>;

} // namespace pbpt::material