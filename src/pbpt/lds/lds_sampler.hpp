#pragma once

#include "pbpt/math/point.hpp"

namespace pbpt::lds {

template <typename Derived, typename T>
class LowDescrepencySequenceSampler {
public:
    LowDescrepencySequenceSampler() = default;

    Derived& as_derived() { return static_cast<Derived&>(*this); }

    const Derived& as_derived() const { return static_cast<const Derived&>(*this); }

    T next_1d() { return as_derived().next_1d_impl(); }

    math::Point<T, 2> next_2d() { return as_derived().next_2d_impl(); }
};

}  // namespace pbpt::lds
