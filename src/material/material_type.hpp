#pragma once

#include <variant>
#include "material.hpp"
#include "utils/library.hpp"

namespace pbpt::material {

template<typename T>
using AnyMaterial = std::variant<
    LambertianMaterial<T>
>;

template<typename T>
using AnyMaterialLibrary = utils::Library<T, AnyMaterial<T>>;

template<typename T>
using NamedAnyMaterialLibrary = utils::NamedLibrary<T, AnyMaterial<T>>;   

}