#pragma once

#include "vector.hpp"

namespace pbpt::math {

template <typename T, int N>
requires (N > 0) && (std::is_floating_point_v<T>)
class Normal : public Vector<T, N> {
public:
    using Vector<T, N>::Vector;
    explicit constexpr Normal(const Vector<T, N>& vec) : Vector<T, N>(vec.normalized()) {}
};

using Normal2 = Normal<Float, 2>;
using Normal3 = Normal<Float, 3>;
using Normal4 = Normal<Float, 4>;

};