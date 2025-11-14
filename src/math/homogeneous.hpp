#pragma once

#include <stdexcept>
#include <type_traits>

#include "function.hpp"
#include "operator.hpp"
#include "point.hpp"
#include "tuple.hpp"
#include "utils.hpp"
#include "vector.hpp"
#include "vector_ops.hpp"

namespace pbpt::math {

template <typename T, int N>
class Homogeneous : public VectorOps<Homogeneous, T, N> {
private:
    using Base = VectorOps<Homogeneous, T, N>;
    using Base::m_data;

public:
    using Base::Base;
    
    static constexpr auto point_at_origin() { 
        Homogeneous<T, N> result;
        for (int i = 0; i < N; ++i)
            result[i] = T(0);
        result[N - 1] = T(1);  // w = 1 for point
        return result;
    }
    
    static constexpr auto zero_vector() {
        Homogeneous<T, N> result;
        for (int i = 0; i < N; ++i)
            result[i] = T(0);  // w = 0 for vector
        return result;
    }

    static constexpr auto from_point(const Point<T, N - 1>& p) {
        Homogeneous<T, N> result;
        for (int i = 0; i < N - 1; ++i)
            result[i] = p[i];
        result[N - 1] = T(1);
        return result;
    }

    static constexpr auto from_vector(const Vector<T, N - 1>& v) {
        Homogeneous<T, N> result;
        for (int i = 0; i < N - 1; ++i)
            result[i] = v[i];
        result[N - 1] = T(0);
        return result;
    }

    static constexpr auto from_vector_raw(const Vector<T, N>& v) {
        Homogeneous<T, N> result;
        for (int i = 0; i < N; ++i)
            result[i] = v[i];
        return result;
    }

    // Accessors with improved error handling
    constexpr const T& w() const { return (*this)[N - 1]; }
    constexpr T& w() { return (*this)[N - 1]; }

    constexpr int dims() const { return N; }
    constexpr int spatial_dims() const { return N - 1; }

    // Type checking methods
    constexpr bool is_point() const { 
        if constexpr (std::is_floating_point_v<T>) {
            return !is_zero((*this)[N - 1]);
        } else {
            return (*this)[N - 1] != 0;
        }
    }

    constexpr bool is_vector() const { 
        if constexpr (std::is_floating_point_v<T>) {
            return is_zero((*this)[N - 1]);
        } else {
            return (*this)[N - 1] == 0;
        }
    }

    // Conversion methods with improved error handling
    constexpr Point<T, N - 1> to_point() const {
        assert_if_ex<std::domain_error>([&]() { return is_vector(); }, 
                                       "Cannot convert homogeneous vector (w=0) to Point");
        
        Vector<T, N - 1> result_coords;
        const T inv_w = T(1) / (*this)[N - 1];
        for (int i = 0; i < N - 1; ++i)
            result_coords[i] = (*this)[i] * inv_w;
        return Point<T, N - 1>::from_vector(result_coords);
    }

    constexpr Vector<T, N - 1> to_vector() const {
        assert_if_ex<std::domain_error>([&]() { return is_point(); }, 
                                       "Cannot convert homogeneous point (w!=0) to Vector");
        
        Vector<T, N - 1> result_coords;
        for (int i = 0; i < N - 1; ++i)
            result_coords[i] = (*this)[i];
        return result_coords;
    }

    constexpr Vector<T, N> to_vector_raw() const {
        Vector<T, N> result_coords;
        for (int i = 0; i < N; ++i)
            result_coords[i] = (*this)[i];
        return result_coords;
    }

    constexpr Vector<T, N>& to_vector_raw() {
        return reinterpret_cast<Vector<T, N>&>(*this);
    }

    template <typename U>
        requires std::is_arithmetic_v<U>
    constexpr auto& operator/=(const U& scalar) {
        assert_if_ex<std::domain_error>([&scalar]() { return is_equal(scalar, U(0)); },
                                        "Division by zero in homogeneous coordinate division");
        return Base::operator/=(scalar);
    }

    template <typename U>
        requires std::is_arithmetic_v<U>
    constexpr auto operator/(const U& scalar) const {
        assert_if_ex<std::domain_error>([&scalar]() { return is_equal(scalar, U(0)); },
                                        "Division by zero in homogeneous coordinate division");
        return Base::operator/(scalar);
    }

    /// @brief Normalize the homogeneous coordinate (make w = 1 if it's a point)
    /// @return Normalized homogeneous coordinate
    constexpr auto standardized() const {
        if (is_vector()) {
            return *this;  // Cannot normalize a vector or w=0
        }
        
        Homogeneous<T, N> result{};
        const T inv_w = T(1) / (*this)[N - 1];
        for (int i = 0; i < N - 1; ++i) {
            result[i] = (*this)[i] * inv_w;
        }
        result[N - 1] = T(1);
        return result;
    }

    /// @brief Normalize the homogeneous coordinate (make w = 1 if it's a point)
    /// @return Normalized homogeneous coordinate
    constexpr auto& standardize() {
        if (is_vector()) {
            return *this;  // Cannot normalize a vector or w=0
        }

        const T inv_w = T(1) / (*this)[N - 1];
        for (int i = 0; i < N - 1; ++i) {
            (*this)[i] *= inv_w;
        }
        (*this)[N - 1] = T(1);
        return *this;
    }

    /// @brief Check if the homogeneous coordinate is normalized (w = 1 for points, w = 0 for vectors)
    constexpr bool is_standardized() const {
        if (is_vector()) {
            return true;  // Vectors are always standardized (w = 0)
        } else {
            return is_equal((*this)[N - 1], T(1));
        }
    }
};

using Homo4 = Homogeneous<Float, 4>;
using Homo3 = Homogeneous<Float, 3>;

}  // namespace pbpt::math
