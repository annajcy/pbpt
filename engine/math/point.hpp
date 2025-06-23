#pragma once

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

namespace pbpt {
namespace math {

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
    Vec<T, N> m_coords{};

public:
    /**
     * @brief Creates a point at the origin (0, 0, ...).
     * @return A new Point instance at the origin.
     */
    constexpr static Point zeros() noexcept { return Point(Vec<T, N>::zeros()); }

    /**
     * @brief Creates a point with all coordinates set to one.
     * @return A new Point instance with all coordinates set to 1.0.
     */
    constexpr static Point ones() noexcept { return Point(Vec<T, N>::ones()); }
    
    // --- 构造函数 (Constructors) ---

    /**
     * @brief Default constructor. Initializes the point at the origin.
     */
    constexpr Point() noexcept : m_coords(Vec<T, N>::zeros()) {}

    /**
     * @brief Constructs a point with all coordinates set to a single scalar value.
     * @param value The value to assign to all coordinates.
     */
    constexpr explicit Point(T value) noexcept : m_coords(value) {}

    /**
     * @brief Constructs a point from a coordinate vector.
     * @param vec The vector representing the point's coordinates from the origin.
     */
    constexpr explicit Point(const Vec<T, N>& vec) noexcept : m_coords(vec) {}

    /**
     * @brief Constructs a point from a list of individual coordinate values.
     * @details The number of arguments must exactly match the point's dimension N.
     * @tparam Args A parameter pack of types convertible to T.
     * @param args The coordinate values.
     * @requires The number of arguments `sizeof...(args)` must be equal to `N`.
     */
    template<std::convertible_to<T>... Args>
    constexpr explicit Point(Args&&... args) noexcept requires(sizeof...(args) == N)
        : m_coords(std::forward<Args>(args)...) {}

    // --- 访问器 (Accessors) ---

    /** @brief Accesses the first coordinate (x-axis). @requires N > 0. */
    constexpr T& x() noexcept requires(N > 0) { return m_coords.x(); }
    /** @brief Accesses the second coordinate (y-axis). @requires N > 1. */
    constexpr T& y() noexcept requires(N > 1) { return m_coords.y(); }
    /** @brief Accesses the third coordinate (z-axis). @requires N > 2. */
    constexpr T& z() noexcept requires(N > 2) { return m_coords.z(); }
    /** @brief Accesses the fourth coordinate (w-axis). @requires N > 3. */
    constexpr T& w() noexcept requires(N > 3) { return m_coords.w(); }

    /** @brief Const access to the first coordinate (x-axis). @requires N > 0. */
    constexpr T x() const noexcept requires(N > 0) { return m_coords.x(); }
    /** @brief Const access to the second coordinate (y-axis). @requires N > 1. */
    constexpr T y() const noexcept requires(N > 1) { return m_coords.y(); }
    /** @brief Const access to the third coordinate (z-axis). @requires N > 2. */
    constexpr T z() const noexcept requires(N > 2) { return m_coords.z(); }
    /** @brief Const access to the fourth coordinate (w-axis). @requires N > 3. */
    constexpr T w() const noexcept requires(N > 3) { return m_coords.w(); }

    /** @brief Returns the number of dimensions of the point. */
    constexpr int dims() const noexcept { return N; }

    // --- 下标访问 (Subscript Access) ---

    /** @brief Provides const access to the point's coordinates by index. */
    constexpr T operator[](int index) const { return m_coords[index]; }
    /** @brief Provides mutable access to the point's coordinates by index. */
    constexpr T& operator[](int index) { return m_coords[index]; }

    // --- 显式转换 (Explicit Conversion) ---

    /**
     * @brief Explicitly converts the point to its underlying coordinate vector.
     * @details This allows a point to be treated as a vector from the origin
     * when an explicit cast is used.
     * @return The `Vec<T, N>` representing the point's coordinates.
     */
    constexpr explicit operator Vec<T, N>() const noexcept { return m_coords; }

    // --- 复合赋值运算符 (Compound Assignment Operators) ---

    /**
     * @brief Translates the point by adding a vector.
     * @param rhs The displacement vector to add.
     * @return A reference to the modified point.
     */
    constexpr Point& operator+=(const Vec<T, N>& rhs) noexcept {
        m_coords += rhs;
        return *this;
    }

    /**
     * @brief Translates the point by subtracting a vector.
     * @param rhs The displacement vector to subtract.
     * @return A reference to the modified point.
     */
    constexpr Point& operator-=(const Vec<T, N>& rhs) noexcept {
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
        // A point prints just like its coordinate vector.
        os << static_cast<Vec<T, N>>(point);
        return os;
    }
};

// --- 全局二元运算符 (Global Binary Operators) ---
// These operators define the fundamental rules of affine geometry.

/**
 * @brief Rule 1: Calculates the displacement vector between two points (`Point - Point = Vector`).
 * @param lhs The destination point.
 * @param rhs The starting point.
 * @return The `Vec<T, N>` that goes from `rhs` to `lhs`.
 */
template<typename T, int N>
constexpr Vec<T, N> operator-(const Point<T, N>& lhs, const Point<T, N>& rhs) noexcept {
    return static_cast<Vec<T, N>>(lhs) - static_cast<Vec<T, N>>(rhs);
}

/**
 * @brief Rule 2: Translates a point by a vector, resulting in a new point (`Point + Vector = Point`).
 * @param lhs The starting point.
 * @param rhs The displacement vector.
 * @return The new, translated point.
 */
template<typename T, int N>
constexpr Point<T, N> operator+(const Point<T, N>& lhs, const Vec<T, N>& rhs) noexcept {
    auto result = lhs;
    result += rhs;
    return result;
}

/**
 * @brief Rule 2 (Commutative): Translates a point by a vector (`Vector + Point = Point`).
 * @param lhs The displacement vector.
 * @param rhs The starting point.
 * @return The new, translated point.
 */
template<typename T, int N>
constexpr Point<T, N> operator+(const Vec<T, N>& lhs, const Point<T, N>& rhs) noexcept {
    return rhs + lhs;
}

/**
 * @brief Rule 3: Translates a point by the negative of a vector (`Point - Vector = Point`).
 * @param lhs The starting point.
 * @param rhs The displacement vector to subtract.
 * @return The new, translated point.
 */
template<typename T, int N>
constexpr Point<T, N> operator-(const Point<T, N>& lhs, const Vec<T, N>& rhs) noexcept {
    auto result = lhs;
    result -= rhs;
    return result;
}

/**
 * @brief Calculates the midpoint between two points.
 * @param lhs The first point.
 * @param rhs The second point.
 * @return The point that lies exactly halfway between `lhs` and `rhs`.
 */
template<typename T, int N>
constexpr Point<T, N> mid_point(const Point<T, N>& lhs, const Point<T, N>& rhs) noexcept {
    // Conceptually: start at lhs, and move halfway towards rhs.
    // (lhs + (rhs - lhs) * 0.5) which simplifies to (lhs + rhs) * 0.5
    return Point<T, N>((static_cast<Vec<T, N>>(lhs) + static_cast<Vec<T, N>>(rhs)) * 0.5);
}

// --- 类型别名 (Type Aliases) ---

/** @brief A 2-dimensional point of type `Float`. */
using Point2 = Point<Float, 2>;
/** @brief A 3-dimensional point of type `Float`. */
using Point3 = Point<Float, 3>;
/** @brief A 4-dimensional point of type `Float`. */
using Point4 = Point<Float, 4>;

} // namespace math
} // namespace pbpt