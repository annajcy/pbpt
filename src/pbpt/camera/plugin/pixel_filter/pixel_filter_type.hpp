#pragma once
#include <variant>
#include "pbpt/camera/plugin/pixel_filter/box_filter.hpp"
#include "pbpt/camera/plugin/pixel_filter/gaussian_filter.hpp"
#include "pbpt/camera/plugin/pixel_filter/tent_filter.hpp"

namespace pbpt::camera {

// Filter Variant
template <typename T>
using AnyPixelFilter = std::variant<camera::GaussianFilter<T>, camera::BoxFilter<T>, camera::TentFilter<T> >;

}  // namespace pbpt::camera
