#pragma once

#include "point.hpp"
#include <concepts>
#include <stdexcept>
#include <type_traits>
#include <iostream>

/**
 * @file homogeneous.hpp
 * @brief Defines a class for N-dimensional homogeneous coordinates.
 * @details This file provides the `Homo` class, which is essential for performing
 * 3D transformations in a unified manner. It allows points and vectors to be
 * treated consistently when multiplied by transformation matrices. The key idea
 * is to represent an N-dimensional concept using an (N+1)-dimensional vector.
 */

namespace pbpt::math {

/**
 * @class Homogeneous
 * @brief A template class for an N-dimensional homogeneous coordinate.
 * @details This class unifies the mathematical treatment of points and vectors
 * for use in projective geometry, particularly for 3D transformations. It
 * internally stores an (N+1)-dimensional vector.
 *
 * The distinction between a point and a vector is encoded in the last component, `w`:
 * - A **Point** `(x, y, z)` is represented in homogeneous coordinates as `(x, y, z, 1)`.
 * - A **Vector** `(vx, vy, vz)` is represented as `(vx, vy, vz, 0)`.
 *
 * This class provides a type-safe way to create these coordinates from `Point` and
 * `Vec` objects, and to convert them back, including performing the necessary
 * perspective divide for points.
 *
 * @tparam T The underlying floating-point type of the coordinate's components.
 * @tparam N The original number of dimensions (e.g., 3 for 3D space). The internal
 * storage will have N+1 dimensions.
 * @see Point
 * @see Vec
 * @see Matrix
 */
template<typename T, int N>
class Homogeneous {
private:
    /**
     * @brief Internal storage for the N+1 homogeneous coordinates.
     */
    Vector<T, N + 1> m_data{};

public:
    // --- 构造函数 (Constructors) ---

    /**
     * @brief Default constructor.
     * @details Initializes a homogeneous coordinate representing the origin POINT (0, ..., 0, 1).
     */
    constexpr Homogeneous() noexcept { m_data[N] = 1; }

    /**
     * @brief Constructs a homogeneous coordinate from a Point.
     * @details Copies the point's coordinates and sets the w-component to 1.
     * @param p The point to convert.
     */
    constexpr explicit Homogeneous(const Point<T, N>& p) noexcept {
        for (int i = 0; i < N; ++i) m_data[i] = p[i];
        m_data[N] = 1;
    }

    /**
     * @brief Constructs a homogeneous coordinate from a Vec (vector).
     * @details Copies the vector's components and sets the w-component to 0.
     * @param v The vector to convert.
     */
    constexpr explicit Homogeneous(const Vector<T, N>& v) noexcept {
        for (int i = 0; i < N; ++i) m_data[i] = v[i];
        m_data[N] = 0;
    }

    /**
     * @brief Constructs a homogeneous coordinate from a list of values.
     * @details The number of values must match the number of dimensions (N + 1).
     * @tparam Vals The types of the values, must be convertible to `T`.
     * @param vals The values to initialize the homogeneous coordinate.
     */

    template<std::convertible_to<T>... Vals>
    constexpr explicit Homogeneous(Vals... vals) noexcept requires(sizeof...(Vals) == N + 1) {
        int i = 0;
        ((m_data[i++] = vals), ...);
    }

    /**
     * @brief Explicitly constructs from a raw (N+1)-dimensional vector.
     * @details This is an advanced constructor, primarily used internally after a
     * matrix transformation has produced a new raw homogeneous coordinate vector.
     * @param data The raw `Vec<T, N + 1>` of homogeneous coordinates.
     */
    constexpr explicit Homogeneous(const Vector<T, N + 1>& data) noexcept : m_data(data) {}

    /**
     * @brief Returns the w-component of the homogeneous coordinate.
     * @return The w-component, which is 1 for points and 0 for vectors.
     */
    constexpr T w() const { return m_data[N]; }

    /**
     * @brief Returns a reference to the w-component of the homogeneous coordinate.
     * @return A reference to the w-component, which can be used to modify it.
     */
    constexpr T& w() { return m_data[N]; }

    // 访问运算符
    constexpr T operator[](int index) const { return m_data[index]; }
    constexpr T& operator[](int index) { return m_data[index]; }
    constexpr T at(int index) const { return m_data.at(index); }
    constexpr T& at(int index) { return m_data.at(index); }


    // --- 状态检查 (State Checks) ---

    /**
     * @brief Checks if this homogeneous coordinate represents a point.
     * @return `true` if the w-component is not zero, `false` otherwise.
     * @note For floating-point types, a check against a small epsilon might be
     * more robust in some runtime applications, but `w != 0` is standard.
     */
    constexpr bool is_point() const noexcept { return m_data[N] != 0; }

    /**
     * @brief Checks if this homogeneous coordinate represents a vector.
     * @return `true` if the w-component is exactly zero, `false` otherwise.
     */
    constexpr bool is_vector() const noexcept { return m_data[N] == 0; }
    
    // --- 转换 (Conversions) ---

    /**
     * @brief Converts the homogeneous coordinate back to a `Point`.
     * @details This method performs the crucial "perspective divide" operation by
     * dividing the x, y, z components by the w component.
     * @return The resulting `Point<T, N>` in Cartesian space.
     * @throw std::runtime_error Throws at runtime if this coordinate represents a
     * vector (w=0), as this conversion is mathematically undefined.
     * @note In a `constexpr` context, an invalid conversion will result in a compile-time error.
     */
    constexpr Point<T, N> to_point() const {
        if (is_vector()) {
            if (std::is_constant_evaluated()) throw "Compile-time error: Cannot convert a homogeneous vector (w=0) to a Point.";
            else throw std::runtime_error("Cannot convert a homogeneous vector (w=0) to a Point.");
        }
        Vector<T, N> result_coords;
        const T inv_w = 1.0 / m_data[N];
        for (int i = 0; i < N; ++i) result_coords[i] = m_data[i] * inv_w;
        return Point<T, N>(result_coords);
    }

    /**
     * @brief Converts the homogeneous coordinate back to a `Vec`.
     * @details This method returns the first N components, assuming w is 0.
     * @return The resulting `Vec<T, N>` in Cartesian space.
     * @throw std::runtime_error Throws at runtime if this coordinate represents a
     * point (w!=0), as this usually indicates a logical error in a transformation pipeline.
     * @note In a `constexpr` context, an invalid conversion will result in a compile-time error.
     */
    constexpr Vector<T, N> to_vector() const {
        if (is_point()) {
            if (std::is_constant_evaluated()) throw "Compile-time error: Cannot convert a homogeneous point (w!=0) to a Vec.";
            else throw std::runtime_error("Logical error: Cannot convert a homogeneous point (w!=0) to a Vec.");
        }
        Vector<T, N> result_coords;
        for (int i = 0; i < N; ++i) result_coords[i] = m_data[i];
        return result_coords;
    }

    // --- 访问器 (Accessors) ---

    /**
     * @brief Provides read-only access to the underlying raw (N+1)-dimensional vector.
     * @return A const reference to the internal `Vec<T, N + 1>`.
     */
    constexpr const Vector<T, N + 1>& raw() const noexcept { return m_data; }

    /**
     * @brief Provides mutable access to the underlying raw (N+1)-dimensional vector.
     * @return A mutable reference to the internal `Vec<T, N + 1>`.
     */
    constexpr Vector<T, N + 1>& raw() noexcept { return m_data; }
    
    // --- 流输出 (Stream Output) ---

    /**
     * @brief Stream insertion operator for printing the homogeneous coordinate.
     * @details Prints a `[P]` or `[V]` tag to distinguish points from vectors,
     * followed by the raw underlying vector data.
     * @param os The output stream.
     * @param h The `Homo` object to print.
     * @return A reference to the output stream.
     */
    friend std::ostream& operator<<(std::ostream& os, const Homogeneous& h) {
        os << "HCoord" << N << (h.is_point() ? "[P] " : "[V] ") << h.raw();
        return os;
    }
};

// --- 类型别名 (Type Aliases) ---

/**
 * @brief A 3-dimensional homogeneous coordinate using the library's default `Float` type.
 * @details This is a convenient alias for `Homo<Float, 3>`, which internally uses a `Vec4`.
 */
using Homo3 = Homogeneous<Float, 3>;

} // namespace homogeneous