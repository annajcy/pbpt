#pragma once

#include <variant>
#include "bxdf.hpp"

namespace pbpt::material {

template<typename T, int N>
using AnyBxDF = std::variant<
    LambertianBxDF<T, N>
>;

}