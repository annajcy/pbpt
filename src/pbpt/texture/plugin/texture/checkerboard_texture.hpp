#pragma once

#include "pbpt/texture/texture.hpp"
#include "pbpt/radiometry/color.hpp"

namespace pbpt::texture {

template <typename T>
class CheckerboardTexture : public Texture<CheckerboardTexture<T>, T, radiometry::RGB<T>> {
    friend class Texture<CheckerboardTexture<T>, T, radiometry::RGB<T>>;
private:
    radiometry::RGB<T> m_color0{T(0), T(0), T(0)};
    radiometry::RGB<T> m_color1{T(1), T(1), T(1)};
    T m_uscale{T(1)};
    T m_vscale{T(1)};
    T m_uoffset{T(0)};
    T m_voffset{T(0)};

public:
    CheckerboardTexture() = default;

    CheckerboardTexture(const radiometry::RGB<T>& color0, const radiometry::RGB<T>& color1, T uscale = T(1),
                        T vscale = T(1), T uoffset = T(0), T voffset = T(0))
        : m_color0(color0),
          m_color1(color1),
          m_uscale(uscale),
          m_vscale(vscale),
          m_uoffset(uoffset),
          m_voffset(voffset) {}

    radiometry::RGB<T> eval_impl(const TextureEvalContext<T>& ctx) const {
        // Use logic from original implementation
        // Need to replicate logic, checking if there are dependencies
        // It uses floor and bitwise ops.

        T u = ctx.uv.x() * m_uscale + m_uoffset;
        T v = ctx.uv.y() * m_vscale + m_voffset;

        // Ensure positive coordinates for checkerboard logic if needed,
        // but standard checkerboard usually works with floor.
        // Original code used:
        auto iu = static_cast<std::int64_t>(std::floor(u));
        auto iv = static_cast<std::int64_t>(std::floor(v));

        // (iu + iv) & 1 check for parity
        bool use_first = ((iu + iv) & 1) == 0;

        return use_first ? m_color0 : m_color1;
    }
};

}  // namespace pbpt::texture
