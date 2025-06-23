#pragma once

#include "vector.hpp"

namespace pbpt {
namespace math {

template<typename T, int N>
class Point {
private:
    Vec<T, N> m_coords{};

public:

    constexpr static Point zeros() noexcept { return Point(Vec<T, N>::zeros()); }
    constexpr static Point ones() noexcept { return Point(Vec<T, N>::ones()); }
    
    // --- 构造函数 ---
    constexpr Point() noexcept : m_coords(Vec<T, N>::zeros()) {}
    constexpr explicit Point(T value) noexcept : m_coords(value) {}
    constexpr explicit Point(const Vec<T, N>& vec) noexcept : m_coords(vec) {}

    template<std::convertible_to<T>... Args>
    constexpr explicit Point(Args&&... args) noexcept requires(sizeof...(args) == N)
        : m_coords(std::forward<Args>(args)...) {}

    // --- 访问器 ---
    constexpr T& x() noexcept requires(N > 0) { return m_coords.x(); }
    constexpr T& y() noexcept requires(N > 1) { return m_coords.y(); }
    constexpr T& z() noexcept requires(N > 2) { return m_coords.z(); }
    constexpr T& w() noexcept requires(N > 3) { return m_coords.w(); }

    constexpr T x() const noexcept requires(N > 0) { return m_coords.x(); }
    constexpr T y() const noexcept requires(N > 1) { return m_coords.y(); }
    constexpr T z() const noexcept requires(N > 2) { return m_coords.z(); }
    constexpr T w() const noexcept requires(N > 3) { return m_coords.w(); }

    constexpr int dims() const noexcept { return N; }

    // --- 下标访问 ---
    constexpr T operator[](int index) const { return m_coords[index]; }
    constexpr T& operator[](int index) { return m_coords[index]; }

    // --- 显式转换 ---
    // 允许显式地将 Point 转换为其底层的坐标向量（即从原点到该点的向量）
    constexpr explicit operator Vec<T, N>() const noexcept { return m_coords; }

    // --- 复合赋值运算符 (Point 与 Vector 运算) ---
    constexpr Point& operator+=(const Vec<T, N>& rhs) noexcept {
        m_coords += rhs;
        return *this;
    }

    constexpr Point& operator-=(const Vec<T, N>& rhs) noexcept {
        m_coords -= rhs;
        return *this;
    }
    
    // --- 流输出 ---
    friend std::ostream& operator<<(std::ostream& os, const Point& point) {
        os << static_cast<Vec<T, N>>(point);
        return os;
    }
};

// --- 全局二元运算符 ---
// 这里定义了点与向量的代数运算规则

// 规则 1: Point - Point = Vector
template<typename T, int N>
constexpr Vec<T, N> operator-(const Point<T, N>& lhs, const Point<T, N>& rhs) noexcept {
    // 两个点的坐标向量相减，得到位移向量
    return static_cast<Vec<T, N>>(lhs) - static_cast<Vec<T, N>>(rhs);
}

// 规则 2: Point + Vector = Point
template<typename T, int N>
constexpr Point<T, N> operator+(const Point<T, N>& lhs, const Vec<T, N>& rhs) noexcept {
    auto result = lhs;
    result += rhs;
    return result;
}

// 交换律: Vector + Point = Point
template<typename T, int N>
constexpr Point<T, N> operator+(const Vec<T, N>& lhs, const Point<T, N>& rhs) noexcept {
    return rhs + lhs;
}

// 规则 3: Point - Vector = Point
template<typename T, int N>
constexpr Point<T, N> operator-(const Point<T, N>& lhs, const Vec<T, N>& rhs) noexcept {
    auto result = lhs;
    result -= rhs;
    return result;
}

template<typename T, int N>
constexpr Point<T, N> mid_point(const Point<T, N>& lhs, const Point<T, N>& rhs) noexcept {
    return Point<T, N>((static_cast<Vec<T, N>>(lhs) + static_cast<Vec<T, N>>(rhs)) * 0.5);
}

// --- 类型别名 ---
using Point2 = Point<Float, 2>;
using Point3 = Point<Float, 3>;
using Point4 = Point<Float, 4>;

} // namespace math
} // namespace pbpt