#pragma once

#include <stdexcept>
#include <type_traits>

#include "function.hpp"
#include "operator.hpp"
#include "point.hpp"
#include "utils.hpp"
#include "vector.hpp"
#include "vector_ops.hpp"

namespace pbpt::math {

/**
 * @brief Homogeneous coordinates built on top of a vector.
 *
 * Represents points and direction vectors in homogeneous form. The last
 * component `w` distinguishes points (w != 0) from vectors (w == 0) and
 * allows projective transformations to be expressed as linear maps.
 *
 * @tparam T Scalar type.
 * @tparam N Dimension of the homogeneous coordinate (spatial dimension is N-1).
 */
template <typename T, int N>
class Homogeneous : public VectorOps<Homogeneous, T, N> {
private:
    using Base = VectorOps<Homogeneous, T, N>;
    using Base::m_data;

public:
    using Base::Base;
    
    /// Returns a homogeneous point at the origin (0,...,0,1).
    static constexpr auto point_at_origin() { 
        Homogeneous<T, N> result;
        for (int i = 0; i < N; ++i)
            result[i] = T(0);
        result[N - 1] = T(1);  // w = 1 for point
        return result;
    }
    
    /// Returns the homogeneous representation of the zero vector (0,...,0,0).
    static constexpr auto zero_vector() {
        Homogeneous<T, N> result;
        for (int i = 0; i < N; ++i)
            result[i] = T(0);  // w = 0 for vector
        return result;
    }

    /// Lifts an inhomogeneous point into homogeneous coordinates (w = 1).
    static constexpr auto from_point(const Point<T, N - 1>& p) {
        Homogeneous<T, N> result;
        for (int i = 0; i < N - 1; ++i)
            result[i] = p[i];
        result[N - 1] = T(1);
        return result;
    }

    /// Lifts an inhomogeneous vector into homogeneous coordinates (w = 0).
    static constexpr auto from_vector(const Vector<T, N - 1>& v) {
        Homogeneous<T, N> result;
        for (int i = 0; i < N - 1; ++i)
            result[i] = v[i];
        result[N - 1] = T(0);
        return result;
    }

    /// Builds a homogeneous coordinate directly from a raw vector of size N.
    static constexpr auto from_vector_raw(const Vector<T, N>& v) {
        Homogeneous<T, N> result;
        for (int i = 0; i < N; ++i)
            result[i] = v[i];
        return result;
    }

    /// Returns the homogeneous w component (const).
    constexpr const T& w() const { return (*this)[N - 1]; }
    /// Returns the homogeneous w component (mutable).
    constexpr T& w() { return (*this)[N - 1]; }

    /// Total number of homogeneous components (including w).
    constexpr int dims() const { return N; }
    /// Number of spatial components (excluding w).
    constexpr int spatial_dims() const { return N - 1; }

    /// Returns true if this homogeneous value encodes a point (w != 0).
    constexpr bool is_point() const { 
        if constexpr (std::is_floating_point_v<T>) {
            return !is_zero((*this)[N - 1]);
        } else {
            return (*this)[N - 1] != 0;
        }
    }

    /// Returns true if this homogeneous value encodes a vector (w == 0).
    constexpr bool is_vector() const { 
        if constexpr (std::is_floating_point_v<T>) {
            return is_zero((*this)[N - 1]);
        } else {
            return (*this)[N - 1] == 0;
        }
    }

    /**
        * @brief Converts a homogeneous point to an inhomogeneous point.
        *
        * Divides spatial coordinates by w and asserts that the value
        * represents a point (w != 0).
        */
    constexpr Point<T, N - 1> to_point() const {
        assert_if_ex<std::domain_error>([&]() { return is_vector(); }, 
                                        "Cannot convert homogeneous vector (w=0) to Point");
        
        Vector<T, N - 1> result_coords;
        const T inv_w = T(1) / (*this)[N - 1];
        for (int i = 0; i < N - 1; ++i)
            result_coords[i] = (*this)[i] * inv_w;
        return Point<T, N - 1>::from_vector(result_coords);
    }

    /**
        * @brief Converts a homogeneous vector to an inhomogeneous vector.
        *
        * Drops the homogeneous w component and asserts that the value
        * represents a vector (w == 0).
        */
    constexpr Vector<T, N - 1> to_vector() const {
        assert_if_ex<std::domain_error>([&]() { return is_point(); }, 
                                    "Cannot convert homogeneous point (w!=0) to Vector");
    
        Vector<T, N - 1> result_coords;
        for (int i = 0; i < N - 1; ++i)
            result_coords[i] = (*this)[i];
        return result_coords;
    }

    /// Returns a plain vector containing all N homogeneous components.
    constexpr Vector<T, N> to_vector_raw() const {
        Vector<T, N> result_coords;
        for (int i = 0; i < N; ++i)
            result_coords[i] = (*this)[i];
        return result_coords;
    }

    /// Returns a mutable reference to the underlying vector storage.
    constexpr Vector<T, N>& to_vector_raw() {
        return reinterpret_cast<Vector<T, N>&>(*this);
    }

    /// Divides all homogeneous components by a scalar in-place.
    template <typename U>
        requires std::is_arithmetic_v<U>
    constexpr auto& operator/=(const U& scalar) {
        assert_if_ex<std::domain_error>([&scalar]() { return is_equal(scalar, U(0)); },
                                        "Division by zero in homogeneous coordinate division");
        return Base::operator/=(scalar);
    }

    /// Returns a copy divided by a scalar.
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
