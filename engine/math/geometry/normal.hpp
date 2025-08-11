#pragma once

#include "math/geometry/tuple.hpp"
#include "math/geometry/vector.hpp"
#include "math/global/operator.hpp"

namespace pbpt::math {

template <typename T, int N>
    requires(N > 0) && (std::is_floating_point_v<T> || std::is_integral_v<T>)
class Normal : public Tuple<Normal, T, N> {
private:
    using Base = Tuple<Normal, T, N>;
    using Base::m_data;

public:
    using Base::Base;

    static std::string name() { return std::format("Normal<{}, {}>", typeid(T).name(), N); }

    // 将 Vector 转换为 Normal
    static constexpr auto from_vector(const Vector<T, N>& vec) {
        Normal<T, N> p;
        for (int i = 0; i < N; ++i)
            p[i] = vec[i];
        return p;
    }

    constexpr auto to_vector() const {
        return Vector<T, N>::from_array(this->to_array());
    }

    constexpr auto face_forward(const Vector<T, N>& vec) const {
        if (is_less(vec.dot(this->to_vector()), 0.0)) {
            Normal<T, N> result = (*this);
            for (int i = 0; i < N; ++i) {
                result[i] = static_cast<T>(this->m_data[i]) * static_cast<T>(-1);
            }
            return result;
        } else {
            return (*this);
        }
    }
};

template <typename T, int N>
auto cross(const Normal<T, N>&, const Normal<T, N>&) = delete;

using Normal2 = Normal<Float, 2>;
using Normal3 = Normal<Float, 3>;
using Normal4 = Normal<Float, 4>;

};  // namespace pbpt::math