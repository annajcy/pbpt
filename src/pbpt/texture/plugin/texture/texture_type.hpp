#pragma once

#include <variant>

#include "pbpt/texture/plugin/texture/bitmap_texture.hpp"
#include "pbpt/texture/plugin/texture/checkerboard_texture.hpp"
#include "pbpt/texture/plugin/texture/constant_texture.hpp"
#include "pbpt/texture/plugin/texture/rsp_spectrum_texture.hpp"
#include "pbpt/utils/library.hpp"

namespace pbpt::texture {

template <typename T>
using AnyTexture = std::variant<
    ConstantTexture<T, radiometry::RGB<T>>, 
    CheckerboardTexture<T>,
    BitmapTexture<T>, 
    RSPSpectrumTexture<T>
>;

template <typename T>
using TextureLibrary = utils::Library<T, AnyTexture<T>>;

template <typename T>
using NamedTextureLibrary = utils::NamedLibrary<T, AnyTexture<T>>;

}  // namespace pbpt::texture
