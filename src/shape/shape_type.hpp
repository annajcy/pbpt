#pragma once

#include <variant>
#include "sphere.hpp"
#include "triangle.hpp"

namespace pbpt::shape {

template<typename T>
using AnyShape = std::variant<
    Sphere<T>,
    Triangle<T>
>;

};