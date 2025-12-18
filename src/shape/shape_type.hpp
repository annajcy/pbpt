#pragma once

#include <variant>
#include "sphere.hpp"
#include "triangle_mesh.hpp"

namespace pbpt::shape {

template<typename T>
using AnyShape = std::variant<
    Sphere<T>,
    TriangleMesh<T>
>;

template<typename T>
using AnyTransformedShape = std::variant<
    TransformedShape<T, Sphere>,
    TransformedShape<T, TriangleMesh>
>;

};