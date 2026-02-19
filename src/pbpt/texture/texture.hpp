#pragma once

#include <cmath>
#include <cstdint>
#include <optional>

#include "pbpt/geometry/interaction.hpp"
#include "pbpt/math/point.hpp"
#include "pbpt/math/vector.hpp"
#include "pbpt/radiometry/color.hpp"

namespace pbpt::texture {

enum class WrapMode { Repeat, Clamp };

enum class FilterMode { Trilinear };

template <typename T>
struct TextureEvalContext {
    math::Point<T, 2> uv{};
    std::optional<geometry::SurfaceDifferentials<T>> differentials{};
    math::Vector<T, 3> dpdu{};
    math::Vector<T, 3> dpdv{};
};

template <typename Derived, typename T, typename ValueType>
class Texture {
public:
    Derived& derived() { return static_cast<Derived&>(*this); }

    const Derived& derived() const { return static_cast<const Derived&>(*this); }

    ValueType eval(const TextureEvalContext<T>& ctx) const { return derived().eval(ctx); }
};

}  // namespace pbpt::texture
