#pragma once

#include <format>
#include <stdexcept>
#include <type_traits>

#include "function.hpp"
#include "operator.hpp"
#include "utils.hpp"
#include "point.hpp"
#include "vector.hpp"
#include "tuple.hpp"

namespace pbpt::math {

template <typename T, int N>
    requires std::is_floating_point_v<T> && (N > 0)
class Homogeneous : public Tuple<Homogeneous, T, N + 1> {
private:
    using Base = Tuple<Homogeneous, T, N + 1>;
    using Base::m_data;

public:
    using Base::Base;

    static std::string name() { 
        return std::format("Homogeneous<{}, {}>", typeid(T).name(), N); 
    }
    
    static constexpr auto point_at_origin() { 
        Homogeneous<T, N> result;
        for (int i = 0; i < N; ++i)
            result[i] = T(0);
        result[N] = T(1);  // w = 1 for point
        return result;
    }
    
    static constexpr auto zero_vector() {
        Homogeneous<T, N> result;
        for (int i = 0; i < N + 1; ++i)
            result[i] = T(0);  // w = 0 for vector
        return result;
    }

    static constexpr auto from_point(const Point<T, N>& p) {
        Homogeneous<T, N> result;
        for (int i = 0; i < N; ++i)
            result[i] = p[i];
        result[N] = T(1);
        return result;
    }

    static constexpr auto from_vector(const Vector<T, N>& v) {
        Homogeneous<T, N> result;
        for (int i = 0; i < N; ++i)
            result[i] = v[i];
        result[N] = T(0);
        return result;
    }

    static constexpr auto from_vector_raw(const Vector<T, N + 1>& v) {
        Homogeneous<T, N> result;
        for (int i = 0; i < N; ++i)
            result[i] = v[i];
        result[N] = v[N];
        return result;
    }

    // Accessors with improved error handling
    constexpr const T& w() const { return (*this)[N]; }
    constexpr T& w() { return (*this)[N]; }

    constexpr int dims() const { return N + 1; }
    constexpr int spatial_dims() const { return N; }

    // Type checking methods
    constexpr bool is_point() const { 
        if constexpr (std::is_floating_point_v<T>) {
            return !is_zero((*this)[N]);
        } else {
            return (*this)[N] != 0;
        }
    }

    constexpr bool is_vector() const { 
        if constexpr (std::is_floating_point_v<T>) {
            return is_zero((*this)[N]);
        } else {
            return (*this)[N] == 0;
        }
    }

    // Conversion methods with improved error handling
    constexpr Point<T, N> to_point() const {
        assert_if_ex<std::domain_error>([&]() { return is_vector(); }, 
                                       "Cannot convert homogeneous vector (w=0) to Point");
        
        Vector<T, N> result_coords;
        const T inv_w = T(1) / (*this)[N];
        for (int i = 0; i < N; ++i)
            result_coords[i] = (*this)[i] * inv_w;
        return Point<T, N>::from_vector(result_coords);
    }

    constexpr Vector<T, N> to_vector() const {
        assert_if_ex<std::domain_error>([&]() { return is_point(); }, 
                                       "Cannot convert homogeneous point (w!=0) to Vector");
        
        Vector<T, N> result_coords;
        for (int i = 0; i < N; ++i)
            result_coords[i] = (*this)[i];
        return result_coords;
    }

    constexpr Vector<T, N + 1> to_vector_raw() const {
        Vector<T, N + 1> result_coords;
        for (int i = 0; i < N; ++i)
            result_coords[i] = (*this)[i];
        result_coords[N] = (*this)[N];
        return result_coords;
    }

    constexpr Vector<T, N + 1>& to_vector_raw() {
        return reinterpret_cast<Vector<T, N + 1>&>(*this);
    }

    // Unified comparison operators
    template <typename U>
    constexpr bool operator==(const Homogeneous<U, N>& rhs) const {
        for (int i = 0; i < N + 1; ++i) {
            if (!is_equal((*this)[i], static_cast<T>(rhs[i]))) {
                return false;
            }
        }
        return true;
    }

    template <typename U>
    constexpr bool operator!=(const Homogeneous<U, N>& rhs) const { 
        return !(*this == rhs); 
    }

    // Assignment operators with type safety
    template <typename U>
    constexpr auto& operator+=(const Homogeneous<U, N>& rhs) {
        for (int i = 0; i < N + 1; ++i)
            (*this)[i] += static_cast<T>(rhs[i]);
        return *this;
    }

    template <typename U>
    constexpr auto& operator-=(const Homogeneous<U, N>& rhs) {
        for (int i = 0; i < N + 1; ++i)
            (*this)[i] -= static_cast<T>(rhs[i]);
        return *this;
    }

    template <typename U>
        requires std::is_arithmetic_v<U>
    constexpr auto& operator*=(const U& scalar) {
        for (int i = 0; i < N + 1; ++i)
            (*this)[i] *= static_cast<T>(scalar);
        return *this;
    }

    template <typename U>
        requires std::is_arithmetic_v<U>
    constexpr auto& operator/=(const U& scalar) {
        assert_if_ex<std::domain_error>([&scalar]() { return is_equal(scalar, U(0)); }, 
                                       "Division by zero in homogeneous coordinate division");
        for (int i = 0; i < N + 1; ++i)
            (*this)[i] /= static_cast<T>(scalar);
        return *this;
    }

    // Arithmetic operators with type promotion
    constexpr auto operator-() const {
        Homogeneous<T, N> result{};
        for (int i = 0; i < N + 1; ++i)
            result[i] = -(*this)[i];
        return result;
    }

    template <typename U>
    constexpr auto operator+(const Homogeneous<U, N>& rhs) const {
        using ResultType = std::common_type_t<T, U>;
        Homogeneous<ResultType, N> result{};
        for (int i = 0; i < N + 1; ++i) {
            result[i] = static_cast<ResultType>((*this)[i]) + 
                       static_cast<ResultType>(rhs[i]);
        }
        return result;
    }

    template <typename U>
    constexpr auto operator-(const Homogeneous<U, N>& rhs) const {
        using ResultType = std::common_type_t<T, U>;
        Homogeneous<ResultType, N> result{};
        for (int i = 0; i < N + 1; ++i) {
            result[i] = static_cast<ResultType>((*this)[i]) - 
                       static_cast<ResultType>(rhs[i]);
        }
        return result;
    }

    template <typename U>
        requires std::is_arithmetic_v<U>
    constexpr auto operator*(const U& scalar) const {
        using ResultType = std::common_type_t<T, U>;
        Homogeneous<ResultType, N> result{};
        for (int i = 0; i < N + 1; ++i) {
            result[i] = static_cast<ResultType>((*this)[i]) * 
                       static_cast<ResultType>(scalar);
        }
        return result;
    }

    template <typename U>
        requires std::is_arithmetic_v<U>
    friend constexpr auto operator*(const U& scalar, const Homogeneous<T, N>& homo) {
        return homo * scalar;
    }

    template <typename U>
        requires std::is_arithmetic_v<U>
    constexpr auto operator/(const U& scalar) const {
        using ResultType = std::common_type_t<T, U>;
        assert_if_ex<std::domain_error>([&scalar]() { return is_equal(scalar, U(0)); }, 
                                       "Division by zero in homogeneous coordinate division");
        Homogeneous<ResultType, N> result{};
        for (int i = 0; i < N + 1; ++i) {
            result[i] = static_cast<ResultType>((*this)[i]) / 
                       static_cast<ResultType>(scalar);
        }
        return result;
    }

    /// @brief Normalize the homogeneous coordinate (make w = 1 if it's a point)
    /// @return Normalized homogeneous coordinate
    constexpr auto standardized() const {
        if (is_vector()) {
            return *this;  // Cannot normalize a vector or w=0
        }
        
        Homogeneous<T, N> result{};
        const T inv_w = T(1) / (*this)[N];
        for (int i = 0; i < N; ++i) {
            result[i] = (*this)[i] * inv_w;
        }
        result[N] = T(1);
        return result;
    }

    /// @brief Normalize the homogeneous coordinate (make w = 1 if it's a point)
    /// @return Normalized homogeneous coordinate
    constexpr auto& standardize() {
        if (is_vector()) {
            return *this;  // Cannot normalize a vector or w=0
        }

        const T inv_w = T(1) / (*this)[N];
        for (int i = 0; i < N; ++i) {
            (*this)[i] *= inv_w;
        }
        (*this)[N] = T(1);
        return *this;
    }

    /// @brief Check if the homogeneous coordinate is normalized (w = 1 for points, w = 0 for vectors)
    constexpr bool is_standardized() const {
        if (is_vector()) {
            return true;  // Vectors are always standardized (w = 0)
        } else {
            return is_equal((*this)[N], T(1));
        }
    }
};

using Homo3 = Homogeneous<Float, 3>;
using Homo2 = Homogeneous<Float, 2>;

}  // namespace pbpt::math