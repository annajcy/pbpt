#pragma once
#include <variant>
#include "camera/pixel_filter.hpp"

namespace pbpt::camera {
    
// Filter Variant
template<typename T>
using AnyPixelFilter = std::variant<
    camera::GaussianFilter<T>,
    camera::BoxFilter<T>,
    camera::TentFilter<T>
>;

}