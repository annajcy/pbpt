#pragma once

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
    WrapMode m_wrap_u{WrapMode::Repeat};
    WrapMode m_wrap_v{WrapMode::Repeat};

public:
    BitmapTexture() = default;

    BitmapTexture(Image<math::Vector<T, 3>> image, WrapMode wrap_u = WrapMode::Repeat,
                  WrapMode wrap_v = WrapMode::Repeat)
        : m_mipmap(std::move(image), wrap_u, wrap_v), m_wrap_u(wrap_u), m_wrap_v(wrap_v) {}

    BitmapTexture(const std::filesystem::path& filename, WrapMode wrap_u = WrapMode::Repeat,
                  WrapMode wrap_v = WrapMode::Repeat)
        : m_mipmap(utils::read_image<T>(filename), wrap_u, wrap_v), m_wrap_u(wrap_u), m_wrap_v(wrap_v) {}

    /// Access the mipmap (for serialization — level(0) is the base image).
    const MipMap<T, math::Vector<T, 3>>& mipmap() const { return m_mipmap; }

    WrapMode wrap_u() const { return m_wrap_u; }
    WrapMode wrap_v() const { return m_wrap_v; }

    radiometry::RGB<T> eval_impl(const TextureEvalContext<T>& ctx) const {
        const auto texel = m_mipmap.sample(ctx, FilterMode::Trilinear);
        return radiometry::RGB<T>(texel.x(), texel.y(), texel.z());
    }
};

}  // namespace pbpt::texture
