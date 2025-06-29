#pragma once

#include "math/function.hpp"
#include "vector.hpp" // Presumed to define Vec, Vec2, Vec3, Vec4, Float

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
class Point {
private:
    /**
     * @brief The internal coordinates of the point, stored as a vector from the origin.
     */
    Vector<T, N> m_coords{};

public:
    // --- 静态工厂函数 (Static Factory Functions) ---

    /**
     * @brief Creates a point filled with a specified value.
     * @param value The value to fill all coordinates with.
     * @return A new Point instance filled with the given value.
     */
    constexpr static Point filled(T value) noexcept { return Point(Vector<T, N>::filled(value)); }

    /**
     * @brief Creates a point at the origin (0, 0, ...).
     * @return A new Point instance at the origin.
     */
    constexpr static Point zeros() noexcept { return Point(Vector<T, N>::zeros()); }

    /**
     * @brief Creates a point with all coordinates set to one.
     * @return A new Point instance with all coordinates set to 1.0.
     */
    constexpr static Point ones() noexcept { return Point(Vector<T, N>::ones()); }
    
    // --- 构造函数 (Constructors) ---

    /**
     * @brief Default constructor. Initializes the point at the origin.
     */
    constexpr Point() noexcept : m_coords(Vector<T, N>::zeros()) {}

    /**
     * @brief Constructs a point from a coordinate vector.
     * @param vec The vector representing the point's coordinates from the origin.
     */
    constexpr explicit Point(const Vector<T, N>& vec) noexcept : m_coords(vec) {}

    /**
     * @brief Constructs a point from a list of individual coordinate values.
     * @details The number of arguments must exactly match the point's dimension N.
     * @tparam Args A parameter pack of types convertible to T.
     * @param args The coordinate values.
     * @note The number of arguments `sizeof...(args)` must be equal to `N`.
     */
    template<std::convertible_to<T>... Args>
    requires(sizeof...(Args) == N)
    constexpr explicit Point(Args&&... args) noexcept 
        : m_coords(std::forward<Args>(args)...) {}

    // --- 访问器 (Accessors) ---

    /** @brief Accesses the first coordinate (x-axis). @note N > 0. */
    constexpr T& x() noexcept requires(N > 0) { return m_coords.x(); }
    /** @brief Accesses the second coordinate (y-axis). @note N > 1. */
    constexpr T& y() noexcept requires(N > 1) { return m_coords.y(); }
    /** @brief Accesses the third coordinate (z-axis). @note N > 2. */
    constexpr T& z() noexcept requires(N > 2) { return m_coords.z(); }
    /** @brief Accesses the fourth coordinate (w-axis). @note N > 3. */
    constexpr T& w() noexcept requires(N > 3) { return m_coords.w(); }

    /** @brief Const access to the first coordinate (x-axis). @note N > 0. */
    constexpr T x() const noexcept requires(N > 0) { return m_coords.x(); }
    /** @brief Const access to the second coordinate (y-axis). @note N > 1. */
    constexpr T y() const noexcept requires(N > 1) { return m_coords.y(); }
    /** @brief Const access to the third coordinate (z-axis). @note N > 2. */
    constexpr T z() const noexcept requires(N > 2) { return m_coords.z(); }
    /** @brief Const access to the fourth coordinate (w-axis). @note N > 3. */
    constexpr T w() const noexcept requires(N > 3) { return m_coords.w(); }

    /** @brief Returns the number of dimensions of the point. */
    constexpr int dims() const noexcept { return N; }

    // --- 下标访问 (Subscript Access) ---

    /** @brief Provides const access to the point's coordinates by index. */
    constexpr const T& operator[](int index) const { return m_coords[index]; }
    /** @brief Provides mutable access to the point's coordinates by index. */
    constexpr T& operator[](int index) { return m_coords[index]; }
    /** @brief Provides safe const access to the point's coordinates by index.*/
    constexpr const T& at(int index) const { return m_coords.at(index); }

    // --- 显式转换 (Explicit Conversion) ---

    /**
     * @brief Explicitly converts the point to its underlying coordinate vector.
     * @details This allows a point to be treated as a vector from the origin
     * when an explicit cast is used.
     * @return The `Vec<T, N>` representing the point's coordinates.
     */
    constexpr Vector<T, N> to_vector() const noexcept { return m_coords; }

    // --- 复合赋值运算符 (Compound Assignment Operators) ---

    /**
     * @brief Translates the point by adding a vector.
     * @param rhs The displacement vector to add.
     * @return A reference to the modified point.
     */
    constexpr Point& operator+=(const Vector<T, N>& rhs) noexcept {
        m_coords += rhs;
        return *this;
    }

    /*** @brief less than */
    constexpr bool operator<(const Point& rhs) const {
        for (int i = 0; i < N; i ++) {
            if (!is_less(m_coords[i], rhs.m_coords[i]))
                return false;
        }
        return true;
    }

    /*** @brief less than or equal*/
    constexpr bool operator<=(const Point& rhs) const {
        for (int i = 0; i < N; i ++) {
            if (is_greater(m_coords[i], rhs.m_coords[i]))
                return false;
        }
        return true;
    }

    /**
     * @brief greater than
     */
    constexpr bool operator>(const Point& rhs) const {
        for (int i = 0; i < N; i ++) {
            if (!is_greater(m_coords[i], rhs.m_coords[i]))
                return false;
        }
        return true;
    }

    /**
     * @brief greater than or equal
     */
    constexpr bool operator>=(const Point& rhs) const {
        for (int i = 0; i < N; i ++) {
            if (is_less(m_coords[i], rhs.m_coords[i]))
                return false;
        }
        return true;
    }

    /**
     * @brief equal
     * 
     * @param rhs 
     * @return true 
     * @return false 
     */
    constexpr bool operator==(const Point& rhs) const {
        for (int i = 0; i < N; i ++) {
            if (!is_equal(m_coords[i], rhs.m_coords[i]))
                return false;
        }
        return true;
    }

    /**
     * @brief Translates the point by subtracting a vector.
     * @param rhs The displacement vector to subtract.
     * @return A reference to the modified point.
     */
    constexpr Point& operator-=(const Vector<T, N>& rhs) noexcept {
        m_coords -= rhs;
        return *this;
    }
    
    // --- 流输出 (Stream Output) ---
    
    /**
     * @brief Stream insertion operator for printing the point's coordinates.
     * @param os The output stream.
     * @param point The point to print.
     * @return A reference to the output stream.
     */
    friend std::ostream& operator<<(std::ostream& os, const Point& point) {
        os << "Point" << N << "(";
        for (int i = 0; i < N; ++i) {
            os << point[i] << (i == N - 1 ? "" : ", ");
        }
        os << ")";
        return os;
    }

    /**
    * @brief Rule 1: Calculates the displacement vector between two points (`Point - Point = Vector`).
    * @param rhs The starting point.
    * @return The `Vec<T, N>` that goes from `rhs` to `lhs`.
    */
    constexpr Vector<T, N> operator-(const Point& rhs) const noexcept {
        return this->to_vector() - rhs.to_vector();
    }

    /**
    * @brief Rule 2: Translates a point by a vector, resulting in a new point (`Point + Vector = Point`).
    * @param rhs The displacement vector.
    * @return The new, translated point.
    */
    constexpr Point operator+(const Vector<T, N>& rhs) const noexcept {
        auto result = *this;
        result += rhs;
        return result;
    }

    /**
    * @brief Rule 3: Translates a point by the negative of a vector (`Point - Vector = Point`).
    * @param rhs The displacement vector to subtract.
    * @return The new, translated point.
    */
    constexpr Point operator-(const Vector<T, N>& rhs) const noexcept {
        auto result = *this;
        result -= rhs;
        return result;
    }

    /**
    * @brief Calculates the midpoint between two points.
    * @param rhs The second point.
    * @return The point that lies exactly halfway between `lhs` and `rhs`.
    */
    constexpr Point mid(const Point<T, N>& rhs) const noexcept {
        // Conceptually: start at lhs, and move halfway towards rhs.
        // (lhs + (rhs - lhs) * 0.5) which simplifies to (lhs + rhs) * 0.5
        return Point<T, N>((this->to_vector() + rhs.to_vector()) * 0.5);
    }
};

// --- 类型别名 (Type Aliases) ---
/** @brief A 2-dimensional point of type `Float`. */
using Pt2 = Point<Float, 2>;
/** @brief A 3-dimensional point of type `Float`. */
using Pt3 = Point<Float, 3>;
/** @brief A 4-dimensional point of type `Float`. */
using Pt4 = Point<Float, 4>;

} // namespace math
