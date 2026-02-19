#pragma once

#include <variant>
#include "sphere.hpp"
#include "triangle.hpp"
#include "pbpt/utils/library.hpp"

namespace pbpt::shape {

template <typename T>
using AnyShape = std::variant<Sphere<T>, Triangle<T>>;

template <typename T>
using MeshLibrary = utils::Library<T, TriangleMesh<T>>;

template <typename T>
using NamedMeshLibrary = utils::NamedLibrary<T, TriangleMesh<T>>;

};  // namespace pbpt::shape
