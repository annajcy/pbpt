#pragma once

#include <concepts>
#include <format>
#include <iostream>
#include <stdexcept>
#include <type_traits>

#include "math/global/operator.hpp"
#include "math/global/utils.hpp"
#include "matrix.hpp"
#include "point.hpp"
#include "vector.hpp"
#include "tuple.hpp"

namespace pbpt::math {

template <typename T, int N>
    requires std::is_floating_point_v<T> && (N > 0)
class Homogeneous : public Tuple<Vector, T, N + 1> {
private:
    using Base = Tuple<Vector, T, N + 1>;
    using Base::m_data;

    // Private constexpr helper for zero checking
    static constexpr bool is_zero_constexpr(const T& value) noexcept {
        if constexpr (std::is_floating_point_v<T>) {
            return (value >= -epsilon_v<T>) && (value <= epsilon_v<T>);
        } else {
            return value == 0;
        }
    }

public:
    using Base::Base;
    static std::string name() { 
        return std::format("Homogeneous<{}, {}>", typeid(T).name(), N); 
    }

    // Static factory methods (unified with Vector design)
    static constexpr Homogeneous zeros() noexcept { 
        Homogeneous result;
        for (int i = 0; i < N + 1; ++i)
            result[i] = T(0);
        return result;
    }
    
    static constexpr Homogeneous filled(T value) noexcept {
        Homogeneous result;
        for (int i = 0; i < N + 1; ++i)
            result[i] = value;
        return result;
    }
    
    static constexpr Homogeneous point_at_origin() noexcept { 
        Homogeneous result;
        for (int i = 0; i < N; ++i)
            result[i] = T(0);
        result[N] = T(1);  // w = 1 for point
        return result;
    }
    
    static constexpr Homogeneous zero_vector() noexcept {
        Homogeneous result;
        for (int i = 0; i < N + 1; ++i)
            result[i] = T(0);  // w = 0 for vector
        return result;
    }

    static constexpr Homogeneous from_point(const Point<T, N>& p) noexcept {
        Homogeneous result;
        for (int i = 0; i < N; ++i)
            result[i] = p[i];
        result[N] = T(1);
        return result;
    }

    static constexpr Homogeneous from_vector(const Vector<T, N>& v) noexcept {
        Homogeneous result;
        for (int i = 0; i < N; ++i)
            result[i] = v[i];
        result[N] = T(0);
        return result;
    }

    static constexpr Homogeneous from_raw(const Vector<T, N + 1>& raw_data) noexcept {
        Homogeneous result;
        for (int i = 0; i < N + 1; ++i)
            result[i] = raw_data[i];
        return result;
    }

    // Constructors
    constexpr Homogeneous() noexcept { 
        for (int i = 0; i < N; ++i)
            (*this)[i] = T(0);
        (*this)[N] = T(1);  // Default to point
    }
    
    constexpr explicit Homogeneous(const Vector<T, N + 1>& data) noexcept {
        for (int i = 0; i < N + 1; ++i)
            (*this)[i] = data[i];
    }

    constexpr explicit Homogeneous(const Point<T, N>& p) noexcept {
        for (int i = 0; i < N; ++i)
            (*this)[i] = p[i];
        (*this)[N] = T(1);
    }

    constexpr explicit Homogeneous(const Vector<T, N>& v) noexcept {
        for (int i = 0; i < N; ++i)
            (*this)[i] = v[i];
        (*this)[N] = T(0);
    }

    template <std::convertible_to<T>... Vals>
        requires(sizeof...(Vals) == N + 1)
    constexpr explicit Homogeneous(Vals... vals) noexcept {
        int i = 0;
        (((*this)[i++] = static_cast<T>(vals)), ...);
    }

    // Copy/conversion constructor from different types
    template <typename U>
        requires std::convertible_to<U, T>
    constexpr Homogeneous(const Homogeneous<U, N>& other) noexcept {
        for (int i = 0; i < N + 1; ++i) {
            (*this)[i] = static_cast<T>(other[i]);
        }
    }

    // Accessors with improved error handling
    constexpr const T& w() const noexcept { return (*this)[N]; }
    constexpr T& w() noexcept { return (*this)[N]; }

    constexpr int dims() const noexcept { return N + 1; }
    constexpr int spatial_dims() const noexcept { return N; }

    // Type checking methods
    constexpr bool is_point() const noexcept { 
        if constexpr (std::is_floating_point_v<T>) {
            return ((*this)[N] < -epsilon_v<T>) || ((*this)[N] > epsilon_v<T>);
        } else {
            return (*this)[N] != 0;
        }
    }
    constexpr bool is_vector() const noexcept { 
        if constexpr (std::is_floating_point_v<T>) {
            return ((*this)[N] >= -epsilon_v<T>) && ((*this)[N] <= epsilon_v<T>);
        } else {
            return (*this)[N] == 0;
        }
    }

    /// @brief Check if any element in the homogeneous coordinate is NaN
    /// @return true if any element is NaN, false otherwise
    [[nodiscard]] constexpr bool has_nan() const noexcept {
        return Base::has_nan();
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

    constexpr const Vector<T, N + 1> raw() const noexcept { 
        return Vector<T, N + 1>::from_array(m_data);
    }

    constexpr Vector<T, N + 1>& raw() noexcept {
        // 直接将内部数据包装为 Vector
        return reinterpret_cast<Vector<T, N + 1>&>(m_data);
    }

    // Unified comparison operators
    template <typename U>
    constexpr bool operator==(const Homogeneous<U, N>& rhs) const noexcept {
        for (int i = 0; i < N + 1; ++i) {
            if (!is_equal((*this)[i], static_cast<T>(rhs[i]))) {
                return false;
            }
        }
        return true;
    }

    template <typename U>
    constexpr bool operator!=(const Homogeneous<U, N>& rhs) const noexcept { 
        return !(*this == rhs); 
    }

    // Assignment operators with type safety
    template <typename U>
    constexpr Homogeneous& operator+=(const Homogeneous<U, N>& rhs) noexcept {
        for (int i = 0; i < N + 1; ++i)
            (*this)[i] += static_cast<T>(rhs[i]);
        return *this;
    }

    template <typename U>
    constexpr Homogeneous& operator-=(const Homogeneous<U, N>& rhs) noexcept {
        for (int i = 0; i < N + 1; ++i)
            (*this)[i] -= static_cast<T>(rhs[i]);
        return *this;
    }

    template <typename U>
        requires std::is_arithmetic_v<U>
    constexpr Homogeneous& operator*=(const U& scalar) noexcept {
        for (int i = 0; i < N + 1; ++i)
            (*this)[i] *= static_cast<T>(scalar);
        return *this;
    }

    template <typename U>
        requires std::is_arithmetic_v<U>
    constexpr Homogeneous& operator/=(const U& scalar) noexcept {
        assert_if_ex<std::domain_error>([&scalar]() { return is_equal(scalar, U(0)); }, 
                                       "Division by zero in homogeneous coordinate division");
        for (int i = 0; i < N + 1; ++i)
            (*this)[i] /= static_cast<T>(scalar);
        return *this;
    }

    // Arithmetic operators with type promotion
    constexpr Homogeneous operator-() const noexcept {
        Homogeneous result{};
        for (int i = 0; i < N + 1; ++i)
            result[i] = -(*this)[i];
        return result;
    }

    template <typename U>
    constexpr auto operator+(const Homogeneous<U, N>& rhs) const noexcept {
        using ResultType = std::common_type_t<T, U>;
        Homogeneous<ResultType, N> result{};
        for (int i = 0; i < N + 1; ++i) {
            result[i] = static_cast<ResultType>((*this)[i]) + 
                       static_cast<ResultType>(rhs[i]);
        }
        return result;
    }

    template <typename U>
    constexpr auto operator-(const Homogeneous<U, N>& rhs) const noexcept {
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
    constexpr auto operator*(const U& scalar) const noexcept {
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
    friend constexpr auto operator*(const U& scalar, const Homogeneous<T, N>& homo) noexcept {
        return homo * scalar;
    }

    template <typename U>
        requires std::is_arithmetic_v<U>
    constexpr auto operator/(const U& scalar) const noexcept {
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

    // Matrix transformation
    template <typename U>
    friend constexpr auto operator*(const Matrix<U, N + 1, N + 1>& mat, const Homogeneous<T, N>& homo) noexcept {
        using ResultType = std::common_type_t<T, U>;
        return Homogeneous<ResultType, N>::from_raw(mat * homo.raw());
    }

    // Apply function to each element (unified with Vector design)
    template <std::invocable<T&, int> F>
    constexpr void apply(F&& f) {
        for (int i = 0; i < N + 1; ++i) {
            f((*this)[i], i);
        }
    }

    template <std::invocable<const T&, int> F>
    constexpr void apply(F&& f) const {
        for (int i = 0; i < N + 1; ++i) {
            f((*this)[i], i);
        }
    }

    // Utility methods (unified with Vector design)  
    constexpr bool is_zero_homo() const {
        for (int i = 0; i < N + 1; ++i) {
            if (!is_zero_constexpr((*this)[i]))
                return false;
        }
        return true;
    }

    /// @brief Normalize the homogeneous coordinate (make w = 1 if it's a point)
    /// @return Normalized homogeneous coordinate
    constexpr Homogeneous normalized() const {
        if (is_vector() || is_zero_constexpr((*this)[N])) {
            return *this;  // Cannot normalize a vector or w=0
        }
        
        Homogeneous result{};
        const T inv_w = T(1) / (*this)[N];
        for (int i = 0; i < N; ++i) {
            result[i] = (*this)[i] * inv_w;
        }
        result[N] = T(1);
        return result;
    }

    /// @brief Check if the homogeneous coordinate is normalized (w = 1 for points, w = 0 for vectors)
    constexpr bool is_normalized() const noexcept {
        if (is_vector()) {
            return is_zero_constexpr((*this)[N]);
        } else {
            return is_equal((*this)[N], T(1));
        }
    }

    // Stream output operator
    friend std::ostream& operator<<(std::ostream& os, const Homogeneous& h) {
        os << "Homogeneous<" << N << ">" << (h.is_point() ? "[Point] " : "[Vector] ") << h.raw();
        return os;
    }
};

using Homo3 = Homogeneous<Float, 3>;
using Homo2 = Homogeneous<Float, 2>;

}  // namespace pbpt::math