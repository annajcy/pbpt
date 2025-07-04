#pragma once

#include "vector.hpp"

/**
 * @file point.hpp
 * @brief Defines a generic, N-dimensional Point class and its geometric operations.
 * @details This file provides a `Point` class that represents a location in N-dimensional
 * space. It is designed to work in tandem with the `Vec` class, enforcing a strict
 * conceptual separation between points (locations) and vectors (displacements).
 * The operators are overloaded to reflect valid affine geometry operations, such as
 * `Point - Point = Vector` and `Point + Vector = Point`.
 */

namespace pbpt::math {
/**
 * @class Point
 * @brief A template class for an N-dimensional point in space.
 * @details Represents a specific location in an N-dimensional affine space. Unlike a
 * `Vec`, which represents direction and magnitude (a displacement), a `Point`
 * represents a coordinate.
 *
 * The class's interface enforces these geometric distinctions through its
 * operator overloads:
 * - You can subtract two points to get the vector between them (`Point - Point = Vec`).
 * - You can add or subtract a vector to a point to get a new, translated point
 * (`Point +/- Vec = Point`).
 * - Operations like adding two points are deliberately not defined, as they are
 * geometrically meaningless in most contexts.
 *
 * @tparam T The underlying floating-point type of the point's coordinates.
 * @tparam N The number of dimensions.
 * @see Vec
 */
template<typename T, int N>
requires (N > 0) && (std::is_floating_point_v<T> || std::is_integral_v<T>)
class Point {
private:
    Vector<T, N> m_data{};

public:

    constexpr static Point filled(T value) noexcept { return Point(Vector<T, N>::filled(value)); }
    constexpr static Point zeros() noexcept { return Point(Vector<T, N>::zeros()); }
    constexpr static Point ones() noexcept { return Point(Vector<T, N>::ones()); }

    constexpr Point() noexcept : m_data(Vector<T, N>::zeros()) {}
    constexpr explicit Point(const Vector<T, N>& vec) noexcept : m_data(vec) {}

    template<std::convertible_to<T>... Args>
    requires(sizeof...(Args) == N)
    constexpr explicit Point(Args&&... args) noexcept : m_data(std::forward<Args>(args)...) {}

    constexpr T& x() noexcept requires(N > 0) { return m_data.x(); }
    constexpr T& y() noexcept requires(N > 1) { return m_data.y(); }
    constexpr T& z() noexcept requires(N > 2) { return m_data.z(); }
    constexpr T& w() noexcept requires(N > 3) { return m_data.w(); }

    constexpr T x() const noexcept requires(N > 0) { return m_data.x(); }
    constexpr T y() const noexcept requires(N > 1) { return m_data.y(); }
    constexpr T z() const noexcept requires(N > 2) { return m_data.z(); }
    constexpr T w() const noexcept requires(N > 3) { return m_data.w(); }
    constexpr int dims() const noexcept { return N; }

    constexpr const T& operator[](int index) const { return m_data[index]; }
    constexpr T& operator[](int index) { return m_data[index]; }
    constexpr const T& at(int index) const { return m_data.at(index); }

    constexpr Vector<T, N> to_vector() const noexcept { return m_data; }

    constexpr Point& operator+=(const Vector<T, N>& rhs) noexcept {
        m_data += rhs;
        return *this;
    }

    constexpr bool operator<(const Point& rhs) const {
        for (int i = 0; i < N; i ++) {
            if (is_greater_equal(m_data[i], rhs.m_data[i]))
                return false;
        }
        return true;
    }

    constexpr bool operator<=(const Point& rhs) const {
        for (int i = 0; i < N; i ++) {
            if (is_greater(m_data[i], rhs.m_data[i]))
                return false;
        }
        return true;
    }

    constexpr bool operator>(const Point& rhs) const {
        for (int i = 0; i < N; i ++) {
            if (is_less_equal(m_data[i], rhs.m_data[i]))
                return false;
        }
        return true;
    }

    constexpr bool operator>=(const Point& rhs) const {
        for (int i = 0; i < N; i ++) {
            if (is_less(m_data[i], rhs.m_data[i]))
                return false;
        }
        return true;
    }

    constexpr bool operator==(const Point& rhs) const {
        for (int i = 0; i < N; i ++) {
            if (is_not_equal(m_data[i], rhs.m_data[i]))
                return false;
        }
        return true;
    }

    constexpr bool operator!=(const Point& rhs) const {
        for (int i = 0; i < N; i ++) {
            if (is_equal(m_data[i], rhs.m_data[i]))
                return false;
        }
        return true;
    }

    constexpr Point& operator-=(const Vector<T, N>& rhs) noexcept {
        m_data -= rhs;
        return *this;
    }
    
    friend std::ostream& operator<<(std::ostream& os, const Point& point) {
        os << "Point" << N << "(";
        for (int i = 0; i < N; ++i) {
            os << point[i] << (i == N - 1 ? "" : ", ");
        }
        os << ")";
        return os;
    }

    constexpr Vector<T, N> operator-(const Point& rhs) const noexcept {
        return this->to_vector() - rhs.to_vector();
    }

    constexpr Point operator+(const Vector<T, N>& rhs) const noexcept {
        auto result = *this;
        result += rhs;
        return result;
    }

    constexpr Point operator-(const Vector<T, N>& rhs) const noexcept {
        auto result = *this;
        result -= rhs;
        return result;
    }

    constexpr Point mid(const Point<T, N>& rhs) const noexcept {
        return Point<T, N>((this->to_vector() + rhs.to_vector()) * 0.5);
    }

    static constexpr Point mid(const std::vector<Point<T, N>>& points) noexcept {
        Vector<T, N> mean = Vector<T, N>::zeros();
        for (const auto& p : points) {
            mean += p.to_vector();
        }
        mean /= points.size();
        return Point<T, N>(mean);
    }
};

using Pt1 = Point<Float, 1>;
using Pt2 = Point<Float, 2>;
using Pt3 = Point<Float, 3>;
using Pt4 = Point<Float, 4>;

using Pt1i = Point<Int, 1>;
using Pt2i = Point<Int, 2>;
using Pt3i = Point<Int, 3>;
using Pt4i = Point<Int, 4>;


} // namespace math
