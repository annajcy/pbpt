#pragma once

#include "operator.hpp"
#include "tuple.hpp"
#include "vector.hpp"
#include "vector_ops.hpp"

namespace pbpt::math {

/**
 * @brief Geometric surface normal vector.
 *
 * This is a thin wrapper around `VectorOps` that is semantically
 * distinct from a direction vector. Helper functions such as
 * `face_forward` make it convenient to orient normals consistently
 * with respect to another vector.
 *
 * @tparam T Scalar type.
 * @tparam N Dimension of the normal.
 */
template <typename T, int N>
class Normal : public VectorOps<Normal, T, N> {
private:
    using Base = VectorOps<Normal, T, N>;
    using Base::m_data;

public:
    using Base::Base;
    
    /// Constructs a normal from a plain vector by copying components.
    static constexpr auto from_vector(const Vector<T, N>& vec) {
        Normal<T, N> p;
        for (int i = 0; i < N; ++i)
            p[i] = vec[i];
        return p;
    }

    /// Converts this normal back to a plain vector with the same components.
    constexpr auto to_vector() const {
        return Vector<T, N>::from_array(this->to_array());
    }

    /**
     * @brief Returns a copy of this normal oriented to face `vec`.
     *
     * If the dot product between the normal and `vec` is negative,
     * the normal is flipped; otherwise it is returned unchanged.
     */
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

    /// Returns a copy of the normal with all components negated.
    Normal<T, N> flipped() const {
        Normal<T, N> result = (*this);
        for (int i = 0; i < N; ++i) {
            result[i] = static_cast<T>(this->m_data[i]) * static_cast<T>(-1);
        }
        return result;
    }

    /// Negates all components of the normal in-place.
    Normal<T, N>& flip() {
        for (int i = 0; i < N; ++i) {
            this->m_data[i] = static_cast<T>(this->m_data[i]) * static_cast<T>(-1);
        }
        return *this;
    }
};

template <typename T, int N>
auto cross(const Normal<T, N>&, const Normal<T, N>&) = delete;

using Normal2 = Normal<Float, 2>;
using Normal3 = Normal<Float, 3>;
using Normal4 = Normal<Float, 4>;

};  // namespace pbpt::math
