#pragma once

#include "pbpt/loader/loader_registry.hpp"
#include "pbpt/material/plugin/material/material_type.hpp"
#include "pbpt/texture/plugin/texture/texture_type.hpp"

namespace pbpt::loader {

template <typename T>
using MaterialLoaderRegistry = LoaderRegistry<T, material::AnyMaterial<T>>;

template <typename T>
using TextureLoaderRegistry = LoaderRegistry<T, texture::AnyTexture<T>>;

template <typename T>
using ShapeLoaderRegistry = LoaderRegistry<T, void>;

}  // namespace pbpt::loader
