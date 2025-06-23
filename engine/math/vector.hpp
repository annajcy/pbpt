#pragma once

#include <array>
#include <concepts> 
#include <stdexcept>
#include <type_traits>
#include <iostream>

#include "type_alias.hpp"
#include "function.hpp" 

namespace pbpt {
namespace math {

template<typename T, int N>
class Vec {
    static_assert(N > 0, "Vector dimensions must be positive");
    static_assert(std::is_floating_point_v<T>, "Vector type must be floating point");

private:
    std::array<T, N> m_data{};

public:
    // --- 静态工厂函数 ---
    static constexpr Vec zeros() noexcept { return Vec(0.0); }
    static constexpr Vec ones() noexcept { return Vec(1.0); }

    // --- 构造函数 ---
    constexpr Vec() noexcept = default;
    constexpr explicit Vec(T value) noexcept { m_data.fill(value); }

    template<std::convertible_to<T>... Args>
    constexpr explicit Vec(Args&&... args) noexcept requires(sizeof...(args) == N) {
        m_data = {static_cast<T>(args)...};
    }

    // --- 访问器 ---
    // (保持不变, 已经是 constexpr)
    constexpr T& x() noexcept requires(N > 0) { return m_data[0]; }
    constexpr T& y() noexcept requires(N > 1) { return m_data[1]; }
    constexpr T& z() noexcept requires(N > 2) { return m_data[2]; }
    constexpr T& w() noexcept requires(N > 3) { return m_data[3]; }

    constexpr T x() const noexcept requires(N > 0) { return m_data[0]; }
    constexpr T y() const noexcept requires(N > 1) { return m_data[1]; }
    constexpr T z() const noexcept requires(N > 2) { return m_data[2]; }
    constexpr T w() const noexcept requires(N > 3) { return m_data[3]; }
    constexpr int dims() const noexcept { return N; }

    // --- 下标访问 ---
    constexpr T operator[](int index) const {
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
                throw std::out_of_range("Index out of range");
            }
        }
        return m_data[index];
    }
    
    // --- 一元运算符 ---
    constexpr Vec operator-() const noexcept {
        Vec result{};
        for (int i = 0; i < N; i++) result[i] = -m_data[i];
        return result;
    }

    // --- 复合赋值运算符 ---
    constexpr Vec& operator+=(const Vec& rhs) noexcept {
        for (int i = 0; i < N; i++) m_data[i] += rhs[i];
        return *this;
    }
    constexpr Vec& operator-=(const Vec& rhs) noexcept {
        for (int i = 0; i < N; i++) m_data[i] -= rhs[i];
        return *this;
    }
    constexpr Vec& operator*=(const T& rhs) noexcept {
        for (int i = 0; i < N; i++) m_data[i] *= rhs;
        return *this;
    }

    // --- 向量数学运算 ---
    constexpr T length_squared() const noexcept {
        T result = 0;
        for (int i = 0; i < N; i++) 
            result += m_data[i] * m_data[i];
        return result;
    }

    constexpr T length() const {
        return pbpt::math::sqrt(length_squared());
    }

    constexpr Vec normalized() const {
        Vec result = *this;
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

    constexpr Vec& normalize() {
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
    
    constexpr T dot(const Vec& rhs) const noexcept {
        T result = 0;
        for (int i = 0; i < N; i++) result += m_data[i] * rhs.m_data[i];
        return result;
    }

    constexpr Vec cross(const Vec& rhs) const noexcept requires(N == 3) {
        return Vec(
            y() * rhs.z() - z() * rhs.y(),
            z() * rhs.x() - x() * rhs.z(),
            x() * rhs.y() - y() * rhs.x()
        );
    }
    
    friend std::ostream& operator<<(std::ostream& os, const Vec& vec) {
        os << '(';
        for (int i = 0; i < N; ++i) {
            os << vec.m_data[i] << (i == N - 1 ? "" : ", ");
        }
        os << ')';
        return os;
    }
};

// --- 全局运算符 ---
template<typename T, int N>
constexpr Vec<T, N> operator+(const Vec<T, N>& lhs, const Vec<T, N>& rhs) noexcept {
    auto result = lhs;
    result += rhs;
    return result;
}

template<typename T, int N>
constexpr Vec<T, N> operator-(const Vec<T, N>& lhs, const Vec<T, N>& rhs) noexcept {
    auto result = lhs;
    result -= rhs;
    return result;
}

template<typename T, int N, std::convertible_to<T> U>
constexpr Vec<T, N> operator*(const Vec<T, N>& lhs, U rhs) noexcept {
    auto result = lhs;
    result *= static_cast<T>(rhs);
    return result;
}

template<typename T, int N, std::convertible_to<T> U>
constexpr Vec<T, N> operator*(U lhs, const Vec<T, N>& rhs) noexcept {
    return rhs * static_cast<T>(lhs);
}

template<typename T, int N>
constexpr Vec<T, N> operator*(const Vec<T, N>& lhs, const Vec<T, N>& rhs) noexcept {
    Vec<T, N> result{};
    for (int i = 0; i < N; i++) result[i] = lhs[i] * rhs[i];
    return result;
}

// --- 类型别名 ---
using Vec2 = Vec<Float, 2>;
using Vec3 = Vec<Float, 3>;
using Vec4 = Vec<Float, 4>;

} // namespace math
} // namespace pbpt