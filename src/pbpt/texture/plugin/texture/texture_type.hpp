#pragma once

#include <variant>

#include "pbpt/texture/plugin/texture/bitmap_texture.hpp"
#include "pbpt/texture/plugin/texture/checkerboard_texture.hpp"
#include "pbpt/texture/plugin/texture/constant_texture.hpp"
#include "pbpt/texture/plugin/texture/rsp_spectrum_texture.hpp"
#include "pbpt/utils/library.hpp"

namespace pbpt::texture {

template <typename T>
using AnyReflectanceTexture = std::variant<ConstantTexture<T, radiometry::RGB<T>>, CheckerboardTexture<T>,
                                           BitmapTexture<T>, RSPSpectrumTexture<T>>;

template <typename T>
using ReflectanceTextureLibrary = utils::Library<T, AnyReflectanceTexture<T>>;

template <typename T>
using NamedReflectanceTextureLibrary = utils::NamedLibrary<T, AnyReflectanceTexture<T>>;

}  // namespace pbpt::texture
