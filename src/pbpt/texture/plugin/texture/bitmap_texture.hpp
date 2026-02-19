#pragma once

#include <filesystem>

#include "pbpt/radiometry/color.hpp"
#include "pbpt/texture/mipmap.hpp"
#include "pbpt/texture/texture.hpp"
#include "pbpt/utils/image_io.hpp"

namespace pbpt::texture {

template <typename T>
class BitmapTexture : public Texture<BitmapTexture<T>, T, radiometry::RGB<T>> {
    friend class Texture<BitmapTexture<T>, T, radiometry::RGB<T>>;
private:
    MipMap<T, math::Vector<T, 3>> m_mipmap{};

public:
    BitmapTexture() = default;

    BitmapTexture(Image<math::Vector<T, 3>> image, WrapMode wrap_u = WrapMode::Repeat,
                  WrapMode wrap_v = WrapMode::Repeat)
        : m_mipmap(std::move(image), wrap_u, wrap_v) {}

    BitmapTexture(const std::filesystem::path& filename, WrapMode wrap_u = WrapMode::Repeat,
                  WrapMode wrap_v = WrapMode::Repeat)
        : m_mipmap(utils::read_image<T>(filename), wrap_u, wrap_v) {}

    radiometry::RGB<T> eval_impl(const TextureEvalContext<T>& ctx) const {
        const auto texel = m_mipmap.sample(ctx, FilterMode::Trilinear);
        return radiometry::RGB<T>(texel.x(), texel.y(), texel.z());
    }
};

}  // namespace pbpt::texture
