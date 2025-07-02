#pragma once

#include "../global/type_alias.hpp"
#include "../global/function.hpp"  
#include "../global/operator.hpp"

#include <algorithm>
#include <array>
#include <concepts>
#include <stdexcept>
#include <type_traits>
#include <iostream>
#include <functional>
#include <vector>

/**
 * @file vector.hpp
 * @brief Defines a generic, N-dimensional, constexpr-friendly vector class.
 */

namespace pbpt::math {

/**
 * @class Vector
 * @brief A template class for N-dimensional mathematical vectors.
 * @details This class provides a generic, fixed-size vector implementation suitable for
 * various mathematical and geometric calculations. It is designed to be highly
 * performant and flexible, with extensive use of `constexpr` for compile-time
 * computations. The vector's underlying type is constrained to floating-point
 * types, and its dimension must be positive.
 * @tparam T The underlying floating-point type of the vector's components (e.g., float, double).
 * @tparam N The number of dimensions of the vector.
 */
template<typename T, int N>
requires (N > 0) && (std::is_floating_point_v<T> || std::is_integral_v<T>)
class Vector {
private:
    std::array<T, N> m_data{};

public:
  
    static constexpr Vector filled(T value) noexcept {
        Vector vec;
        vec.m_data.fill(value);
        return vec;
    }

    static constexpr Vector zeros() noexcept { return filled(0.0); }
    static constexpr Vector ones() noexcept { return filled(1.0); }

    constexpr Vector() noexcept = default;

    template<std::convertible_to<T>... Args>
    requires(sizeof...(Args) == N)
    constexpr explicit Vector(Args&&... args) noexcept  {
        m_data = {static_cast<T>(args)...};
    }

    
    constexpr T& x() noexcept requires(N > 0) { return m_data[0]; }
    constexpr T& y() noexcept requires(N > 1) { return m_data[1]; }
    constexpr T& z() noexcept requires(N > 2) { return m_data[2]; }
    constexpr T& w() noexcept requires(N > 3) { return m_data[3]; }

    constexpr const T& x() const noexcept requires(N > 0) { return m_data[0]; }
    constexpr const T& y() const noexcept requires(N > 1) { return m_data[1]; }
    constexpr const T& z() const noexcept requires(N > 2) { return m_data[2]; }
    constexpr const T& w() const noexcept requires(N > 3) { return m_data[3]; }

    constexpr int dims() const noexcept { return N; }

    constexpr const T& operator[](int index) const {
        if (index < 0 || index >= N) {
            if (std::is_constant_evaluated()) {
                throw "Compile-time error: Index out of range";
            } else {
                throw std::out_of_range("Index out of range");
            }
        }
        return m_data[index];
    }

    constexpr T& operator[](int index) {
        if (index < 0 || index >= N) {
            if (std::is_constant_evaluated()) {
                throw "Compile-time error: Index out of range";
            } else {
                throw std::runtime_error("Index out of range");
            }
        }
        return m_data[index];
    }

    constexpr const T& at(int index) const {
        if (index < 0 || index >= N) {
            if (std::is_constant_evaluated()) {
                throw "Compile-time error: Index out of range";
            } else {
                throw std::out_of_range("Index out of range");
            }
        }
        return m_data[index];
    }

    constexpr Vector operator-() const noexcept {
        Vector result{};
        for (int i = 0; i < N; i++) result[i] = -m_data[i];
        return result;
    }

    constexpr Vector& operator+=(const Vector& rhs) noexcept {
        for (int i = 0; i < N; i++) m_data[i] += rhs[i];
        return *this;
    }

    constexpr Vector& operator-=(const Vector& rhs) noexcept {
        for (int i = 0; i < N; i++) m_data[i] -= rhs[i];
        return *this;
    }

    constexpr Vector& operator*=(const T& rhs) noexcept {
        for (int i = 0; i < N; i++) m_data[i] *= rhs;
        return *this;
    }

    template <typename U>
    requires std::convertible_to<U, T>
    constexpr Vector& operator/=(const U& rhs) {
        T rhs_t = static_cast<T>(rhs);
        if (is_equal(rhs_t, T(0.0))) {
            if (std::is_constant_evaluated()) {
                throw "Compile-time error: Division by zero";
            } else {
                throw std::runtime_error("Division by zero");
            }
        }
        T inv = 1 / rhs_t;
        for (int i = 0; i < N; i++) m_data[i] *= inv;
        return *this;
    }

    constexpr bool operator==(const Vector& rhs) const noexcept {
        for (int i = 0; i < N; i++) {
            if (is_not_equal(m_data[i], rhs[i])) return false;
        }
        return true;
    }

    constexpr bool operator!=(const Vector& rhs) const noexcept {
        for (int i = 0; i < N; i++) {
            if (is_equal(m_data[i], rhs[i])) return false;
        }
        return true;
    }

    constexpr bool is_normalized() const {
        return is_equal(length(), 1.0);
    }

    constexpr bool is_zero() const {
        for (int i = 0; i < N; i++) {
            if (!is_equal(m_data[i], 0.0)) return false;
        }
        return true;
    }

    constexpr bool has_nan() const {
        for (int i = 0; i < N; i++) {
            if (std::isnan(m_data[i])) return true;
        }
        return false;
    }

    constexpr T length_squared() const noexcept {
        T result = 0;
        for (int i = 0; i < N; i++) 
            result += m_data[i] * m_data[i];
        return result;
    }

    constexpr T length() const {
        return pbpt::math::sqrt(length_squared());
    }

    constexpr Vector normalized() const {
        Vector result = *this;
        T len = length();
        if (len == 0) {
            if (std::is_constant_evaluated()) {
                throw "Compile-time error: Cannot normalize a zero vector";
            } else {
                throw std::runtime_error("Cannot normalize a zero vector");
            }
        }
        return result * (1.0 / len);
    }

    constexpr Vector& normalize() {
        T len = length();
        if (len == 0) {
            if (std::is_constant_evaluated()) {
                throw "Compile-time error: Cannot normalize a zero vector";
            } else {
                throw std::runtime_error("Cannot normalize a zero vector");
            }
        }
        return *this *= (1.0 / len);
    }
    
    constexpr T dot(const Vector& rhs) const noexcept {
        T result = 0;
        for (int i = 0; i < N; i++) result += m_data[i] * rhs.m_data[i];
        return result;
    }

    constexpr auto cross(const Vector& rhs) const noexcept requires(N == 3) {
        return Vector<T, 3>(
            y() * rhs.z() - z() * rhs.y(),
            z() * rhs.x() - x() * rhs.z(),
            x() * rhs.y() - y() * rhs.x()
        );
    }

    constexpr T product() const noexcept {
        T result = 1;
        for (int i = 0; i < N; i++) result *= m_data[i];
        return result;
    }

    void apply(const std::function<void(T&, int)>& func) {
        for (int i = 0; i < N; ++i) func(m_data[i], i);
    }

    constexpr Vector operator+(const Vector<T, N>& rhs) const noexcept {
        auto result = *this;
        result += rhs;
        return result;
    }

    constexpr Vector operator-(const Vector<T, N>& rhs) const noexcept {
        auto result = *this;
        result -= rhs;
        return result;
    }

    template<typename  U>
    requires std::convertible_to<U, T>
    constexpr Vector operator*(U value) const noexcept {
        auto result = *this;
        result *= static_cast<T>(value);
        return result;
    }

    template<typename  U>
    requires std::convertible_to<U, T>
    friend constexpr Vector operator*(U lhs, const Vector<T, N>& rhs) noexcept {
        return rhs * static_cast<T>(lhs);
    }

    constexpr Vector operator*(const Vector<T, N>& rhs) const noexcept {
        Vector<T, N> result{};
        for (int i = 0; i < N; i++) result[i] = (*this)[i] * rhs[i];
        return result;
    }

    template <typename U>
    requires std::convertible_to<U, T>
    constexpr Vector operator/(U rhs) const {
        Vector result = *this;
        result /= static_cast<T>(rhs);
        return result;
    }

    friend std::ostream& operator<<(std::ostream& os, const Vector& vec) {
        os << "Vec" << N << "(";
        for (int i = 0; i < N; ++i) {
            os << vec[i] << (i == N - 1 ? "" : ", ");
        }
        os << ')';
        return os;
    }

    constexpr int max_dim() const {
        return std::max_element(m_data.begin(), m_data.end()) - m_data.begin();
    }

    constexpr T max() const {
        return *std::max_element(m_data.begin(), m_data.end());
    }

    constexpr int min_dim() const {
        return std::min_element(m_data.begin(), m_data.end()) - m_data.begin();
    }

    constexpr T min() const {
        return *std::min_element(m_data.begin(), m_data.end());
    }

    template<typename ...Args>
    requires (sizeof...(Args) == N)
    constexpr Vector permuted(Args ...args) const {
        Vector result;
        int i = 0;
        ((result[i++] = (*this)[args]), ...);
        return result;
    }

    template<typename ...Args>
    requires (sizeof...(Args) == N)
    constexpr Vector& permute(Args ...args) {
        *this = permuted(args...);
        return *this;
    }

    template<typename U, int M>
    requires std::convertible_to<U, T> && (M > 0)
    constexpr Vector<U, M> cast() const {
        Vector<U, M> result;
        for (int i = 0; i < std::min(N, M); i++) result[i] = static_cast<U>(m_data[i]);
        return result;
    }

    template<typename U>
    requires std::convertible_to<U, T>
    constexpr Vector<U, N> type_cast() const {
        return cast<U, N>();
    }

    template<int M>
    requires (M > 0)
    constexpr Vector<T, M> dim_cast() const {
        return cast<T, M>();
    }
    
};


template<typename T, int N>
constexpr inline std::vector<Vector<T, N>> get_orthogonal_bases(const Vector<T, N>& base) {
     if constexpr (base.is_zero()) {
        if (std::is_constant_evaluated()) {
            throw "Base vector is zero, orthogonal bases are undefined.";
        } else {
            throw std::invalid_argument("Base vector is zero, orthogonal bases are undefined.");
        }
    }
    //TODO
    return {};
}

template<typename T>
constexpr inline std::vector<Vector<T, 2>> get_orthogonal_bases(const Vector<T, 2>& base) {
    if (base.is_zero()) {
        if (std::is_constant_evaluated()) {
            throw "Base vector is zero, orthogonal bases are undefined.";
        } else {
            throw std::invalid_argument("Base vector is zero, orthogonal bases are undefined.");
        }
    }
    Vector<T, 2> u, v;
    u = base.normalized();
    v = Vector<T, 2>(-u.y(), u.x());
    return {u, v};
}

template<typename T>
constexpr inline std::vector<Vector<T, 3>> get_orthogonal_bases(const Vector<T, 3>& base) {
    if (base.is_zero()) {
        if (std::is_constant_evaluated()) {
            throw "Base vector is zero, orthogonal bases are undefined.";
        } else {
            throw std::invalid_argument("Base vector is zero, orthogonal bases are undefined.");
        }
    }
    
    Vector<T, 3> u, v, w;
    u = base.normalized();
    if (std::abs(u.x()) > std::abs(u.y())) {
        T inv_len = T(1) / std::sqrt(u.x() * u.x() + u.z() * u.z());
        v = Vector<T, 3>(-u.z() * inv_len, T(0), u.x() * inv_len);
    } else {
        T inv_len = T(1) / std::sqrt(u.y() * u.y() + u.z() * u.z());
        v = Vector<T, 3>(T(0), u.z() * inv_len, -u.y() * inv_len);
    }
    w = u.cross(v);
    return {u, v, w};
}

using Vec2 = Vector<Float, 2>;
using Vec3 = Vector<Float, 3>;
using Vec4 = Vector<Float, 4>;

using Vec2i = Vector<Int, 2>;
using Vec3i = Vector<Int, 3>;
using Vec4i = Vector<Int, 4>;

template <typename T, int N>
requires (N > 0) && (std::is_floating_point_v<T>)
class Normal : public Vector<T, N> {
public:
    using Vector<T, N>::Vector;
    constexpr Normal(const Vector<T, N>& vec) : Vector<T, N>(vec.normalized()) {}
};

using Normal2 = Normal<Float, 2>;
using Normal3 = Normal<Float, 3>;
using Normal4 = Normal<Float, 4>;

} // namespace math