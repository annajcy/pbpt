#pragma once

#include <cmath>
#include <cstdint>
#include <optional>

#include "pbpt/geometry/interaction.hpp"
#include "pbpt/math/point.hpp"
#include "pbpt/math/vector.hpp"
#include "pbpt/radiometry/color.hpp"

namespace pbpt::texture {

enum class WrapMode {
    Repeat,
    Clamp
};

enum class FilterMode {
    Trilinear
};

template<typename T>
struct TextureEvalContext {
    math::Point<T, 2> uv{};
    std::optional<geometry::SurfaceDifferentials<T>> differentials{};
    math::Vector<T, 3> dpdu{};
    math::Vector<T, 3> dpdv{};
};

template<typename Derived, typename T, typename ValueType>
class Texture {
public:
    Derived& derived() {
        return static_cast<Derived&>(*this);
    }

    const Derived& derived() const {
        return static_cast<const Derived&>(*this);
    }

    ValueType eval(const TextureEvalContext<T>& ctx) const {
        return derived().eval(ctx);
    }
};

template<typename T, typename ValueType>
class ConstantTexture : public Texture<ConstantTexture<T, ValueType>, T, ValueType> {
    friend class Texture<ConstantTexture<T, ValueType>, T, ValueType>;
private:
    ValueType m_value{};

public:
    ConstantTexture() = default;
    explicit ConstantTexture(const ValueType& value) : m_value(value) {}

    ValueType eval(const TextureEvalContext<T>&) const {
        return m_value;
    }
};

template<typename T>
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

    CheckerboardTexture(
        const radiometry::RGB<T>& color0,
        const radiometry::RGB<T>& color1,
        T uscale = T(1),
        T vscale = T(1),
        T uoffset = T(0),
        T voffset = T(0)
    ) : m_color0(color0),
        m_color1(color1),
        m_uscale(uscale),
        m_vscale(vscale),
        m_uoffset(uoffset),
        m_voffset(voffset) {}

    radiometry::RGB<T> eval(const TextureEvalContext<T>& ctx) const {
        const T u = ctx.uv.x() * m_uscale + m_uoffset;
        const T v = ctx.uv.y() * m_vscale + m_voffset;

        const auto iu = static_cast<std::int64_t>(std::floor(u));
        const auto iv = static_cast<std::int64_t>(std::floor(v));
        const bool use_first = ((iu + iv) & 1LL) == 0;
        return use_first ? m_color0 : m_color1;
    }
};

}
