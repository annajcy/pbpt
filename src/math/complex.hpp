#pragma once

#include <cmath>
#include "math/vector.hpp"

namespace pbpt::math {

template<typename T>
class Complex {
private:
    T m_real;
    T m_imag;

public:
    static Complex<T> from_vector(const math::Vector<T, 2>& vec) {
        return Complex<T>(vec.x(), vec.y());
    }

    static Complex<T> from_polar(T magnitude, T angle) {
        return Complex<T>(
            magnitude * std::cos(angle),
            magnitude * std::sin(angle)
        );
    }

public:

    constexpr Complex() noexcept : m_real(0), m_imag(0) {}
    constexpr Complex(T r, T i) noexcept : m_real(r), m_imag(i) {}

    constexpr const T& real() const noexcept { return m_real; }
    constexpr const T& imag() const noexcept { return m_imag; }
    constexpr T& real() noexcept { return m_real; }
    constexpr T& imag() noexcept { return m_imag; }

    constexpr Complex<T>& operator+=(const Complex<T>& other) noexcept {
        m_real += other.m_real;
        m_imag += other.m_imag;
        return *this;
    }

    constexpr Complex<T>& operator-=(const Complex<T>& other) noexcept {
        m_real -= other.m_real;
        m_imag -= other.m_imag;
        return *this;
    }

    constexpr Complex<T>& operator*=(const Complex<T>& other) noexcept {
        T real_part = m_real * other.m_real - m_imag * other.m_imag;
        T imag_part = m_real * other.m_imag + m_imag * other.m_real;
        m_real = real_part;
        m_imag = imag_part;
        return *this;
    }

    constexpr Complex<T>& operator*=(T scalar) noexcept {
        m_real *= scalar;
        m_imag *= scalar;
        return *this;
    }

    constexpr Complex<T>& operator/=(const Complex<T>& other) noexcept {
        T denom = other.m_real * other.m_real + other.m_imag * other.m_imag;
        T real_part = (m_real * other.m_real + m_imag * other.m_imag) / denom;
        T imag_part = (m_imag * other.m_real - m_real * other.m_imag) / denom;
        m_real = real_part;
        m_imag = imag_part;
        return *this;
    }

    constexpr Complex<T>& operator/=(T scalar) noexcept {
        m_real /= scalar;
        m_imag /= scalar;
        return *this;
    }

    constexpr Complex<T> operator-() const noexcept {
        return Complex<T>(-m_real, -m_imag);
    }

    constexpr Complex<T> operator+(const Complex<T>& other) const noexcept {
        Complex<T> res = *this;
        res += other;
        return res;
    }

    constexpr Complex<T> operator-(const Complex<T>& other) const noexcept {
        Complex<T> res = *this;
        res -= other;
        return res;
    }

    constexpr Complex<T> operator*(const Complex<T>& other) const noexcept {
        Complex<T> res = *this;
        res *= other;
        return res;
    }

    constexpr Complex<T> operator/(const Complex<T>& other) const noexcept {
        Complex<T> res = *this;
        res /= other;
        return res;
    }

    constexpr Complex<T> operator*(T scalar) const noexcept {
        Complex<T> res = *this;
        res *= scalar;
        return res;
    }

    constexpr Complex<T> operator/(T scalar) const noexcept {
        Complex<T> res = *this;
        res /= scalar;
        return res;
    }

    constexpr Complex<T> sqrt() const noexcept {
        // 如果是零
        if (m_real == 0 && m_imag == 0) return Complex<T>(0, 0);

        T x = m_real;
        T y = m_imag;
        T r = std::hypot(x, y); // 获取模长 |z|

        // 公式: sqrt(x+iy) = +/- (sqrt((r+x)/2) + i * sgn(y) * sqrt((r-x)/2))
        // 我们通常取实部为正的主值
        T u = std::sqrt((r + x) / T(2));
        T v = std::sqrt((r - x) / T(2));
        
        if (y < 0) v = -v; // 处理符号

        return Complex<T>(u, v);
    }

    constexpr T length() const noexcept {
        return std::hypot(m_real, m_imag);
    }

    constexpr T squared_length() const noexcept {
        return m_real * m_real + m_imag * m_imag;
    }

    constexpr math::Vector<T, 2> to_vector() const noexcept {
        return math::Vector<T, 2>(m_real, m_imag);
    }

    constexpr Complex<T> conjugate() const noexcept {
        return Complex<T>(m_real, -m_imag);
    }

    //eulur's formula e^(a + bi) = e^a * (cos(b) + i sin(b))    
    constexpr Complex<T> exp() const noexcept {
        T exp_real = std::exp(m_real);
        return Complex<T>(
            exp_real * std::cos(m_imag),
            exp_real * std::sin(m_imag)
        );
    }

    friend constexpr Complex<T> operator*(T scalar, const Complex<T>& complex) noexcept {
        return complex * scalar;
    }

    friend Complex<T> operator/(T lhs, const Complex<T>& rhs) {
        T denom = rhs.m_real * rhs.m_real + rhs.m_imag * rhs.m_imag;
        return Complex<T>(
            (lhs * rhs.m_real) / denom,
            (-lhs * rhs.m_imag) / denom
        );
    }

    friend Complex<T> operator-(T lhs, const Complex<T>& rhs) {
        return Complex<T>(lhs - rhs.m_real, -rhs.m_imag);
    }

    friend Complex<T> operator+(T lhs, const Complex<T>& rhs) {
        return Complex<T>(lhs + rhs.m_real, rhs.m_imag);
    }

};

};