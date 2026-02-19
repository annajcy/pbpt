#pragma once

#include <variant>

#include "pbpt/texture/bitmap_texture.hpp"
#include "pbpt/texture/texture.hpp"
#include "pbpt/utils/library.hpp"

namespace pbpt::texture {

template<typename T>
using AnyReflectanceTexture = std::variant<
    ConstantTexture<T, radiometry::RGB<T>>,
    CheckerboardTexture<T>,
    BitmapTexture<T>
>;

template<typename T>
using ReflectanceTextureLibrary = utils::Library<T, AnyReflectanceTexture<T>>;

template<typename T>
using NamedReflectanceTextureLibrary = utils::NamedLibrary<T, AnyReflectanceTexture<T>>;

}
