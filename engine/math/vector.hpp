#pragma once

#include <algorithm>
#include <array>
#include <concepts>
#include <stdexcept>
#include <type_traits>
#include <iostream>
#include <functional>
#include <vector>

#include "global.hpp" 
#include "function.hpp"  

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
requires (N > 0) && std::is_floating_point_v<T>
class Vector {
private:
    std::array<T, N> m_data{};

public:
    // --- 静态工厂函数 (Static Factory Functions) ---

    /**
     * @brief Creates a vector with all components set to a single scalar value.
     * @param value The value to assign to all components.
     */
    static constexpr Vector filled(T value) noexcept {
        Vector vec;
        vec.m_data.fill(value);
        return vec;
    }

    /**
     * @brief Creates a vector with all components set to zero.
     * @return A new Vec instance with all components initialized to 0.0.
     */
    static constexpr Vector zeros() noexcept { return filled(0.0); }

    /**
     * @brief Creates a vector with all components set to one.
     * @return A new Vec instance with all components initialized to 1.0.
     */
    static constexpr Vector ones() noexcept { return filled(1.0); }

    // --- 构造函数 (Constructors) ---

    /**
     * @brief Default constructor. Initializes all components to zero.
     */
    constexpr Vector() noexcept = default;

    /**
     * @brief Constructs a vector from a list of individual components.
     * @details This constructor allows for initialization like `Vec<float, 3>(1.0f, 2.0f, 3.0f)`.
     * The number of arguments must exactly match the vector's dimension N.
     * @tparam Args A parameter pack of types convertible to T.
     * @param args The component values.
     * @note The number of arguments `sizeof...(args)` must be equal to `N`.
     */
    template<std::convertible_to<T>... Args>
    requires(sizeof...(Args) == N)
    constexpr explicit Vector(Args&&... args) noexcept  {
        m_data = {static_cast<T>(args)...};
    }

    // --- 访问器 (Accessors) ---

    /** @brief Accesses the first component (x-axis). @note N > 0. */
    constexpr T& x() noexcept requires(N > 0) { return m_data[0]; }
    /** @brief Accesses the second component (y-axis). @note N > 1. */
    constexpr T& y() noexcept requires(N > 1) { return m_data[1]; }
    /** @brief Accesses the third component (z-axis). @note N > 2. */
    constexpr T& z() noexcept requires(N > 2) { return m_data[2]; }
    /** @brief Accesses the fourth component (w-axis). @note N > 3. */
    constexpr T& w() noexcept requires(N > 3) { return m_data[3]; }

    /** @brief Const access to the first component (x-axis). @note N > 0. */
    constexpr const T& x() const noexcept requires(N > 0) { return m_data[0]; }
    /** @brief Const access to the second component (y-axis). @note N > 1. */
    constexpr const T& y() const noexcept requires(N > 1) { return m_data[1]; }
    /** @brief Const access to the third component (z-axis). @note N > 2. */
    constexpr const T& z() const noexcept requires(N > 2) { return m_data[2]; }
    /** @brief Const access to the fourth component (w-axis). @note N > 3. */
    constexpr const T& w() const noexcept requires(N > 3) { return m_data[3]; }

    /** @brief Returns the number of dimensions of the vector. */
    constexpr int dims() const noexcept { return N; }

    // --- 下标访问 (Subscript Access) ---

    /**
     * @brief Provides const access to the vector's components by index.
     * @param index The zero-based index of the component to access.
     * @return A const reference to the component.
     * @throw std::out_of_range If `index` is out of bounds [0, N-1] at runtime.
     * @note In a `constexpr` context, out-of-bounds access will result in a compile-time error.
     */
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

    /**
     * @brief Provides mutable access to the vector's components by index.
     * @param index The zero-based index of the component to access.
     * @return A mutable reference to the component.
     * @throw std::out_of_range If `index` is out of bounds [0, N-1] at runtime.
     * @note In a `constexpr` context, out-of-bounds access will result in a compile-time error.
     */
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

    /**
     * @brief Provides const access to the vector's components by index.
     * @param index The zero-based index of the component to access.
     * @return A const reference to the component.
     * @throw std::out_of_range If `index` is out of bounds [0, N-1] at runtime.
     * @note In a `constexpr` context, out-of-bounds access will result in a compile-time error.
     */
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

    // --- 一元运算符 (Unary Operators) ---

    /**
     * @brief Negates the vector.
     * @return A new vector where each component is the negation of the original.
     */
    constexpr Vector operator-() const noexcept {
        Vector result{};
        for (int i = 0; i < N; i++) result[i] = -m_data[i];
        return result;
    }

    // --- 复合赋值运算符 (Compound Assignment Operators) ---
    
    /** @brief Adds another vector to this one component-wise. */
    constexpr Vector& operator+=(const Vector& rhs) noexcept {
        for (int i = 0; i < N; i++) m_data[i] += rhs[i];
        return *this;
    }
    /** @brief Subtracts another vector from this one component-wise. */
    constexpr Vector& operator-=(const Vector& rhs) noexcept {
        for (int i = 0; i < N; i++) m_data[i] -= rhs[i];
        return *this;
    }
    /** @brief Multiplies this vector by a scalar. */
    constexpr Vector& operator*=(const T& rhs) noexcept {
        for (int i = 0; i < N; i++) m_data[i] *= rhs;
        return *this;
    }

    /**
     * @brief operator== ,compare two vector component by component
     */
    constexpr bool operator==(const Vector& rhs) const noexcept {
        for (int i = 0; i < N; i++) {
            if (!is_equal(m_data[i], rhs[i])) return false;
        }
        return true;
    }

    /** @brief operator!= */
    constexpr bool operator!=(const Vector& rhs) const noexcept {
        return !(*this == rhs);
    }

    /**
     * @brief Check if the vector is normalized.
     * @details A normalized vector has a length of 1.0.
     * @return true If the vector is normalized.
     * @return false If the vector is not normalized.
     */
    constexpr bool is_normalized() const {
        return is_equal(length(), 1.0);
    }

    /**
     * @brief Check if all components of the vector are zero.
     * 
     * @return true If all components are zero.
     * @return false If any component is non-zero.
     */
    constexpr bool is_zero() const {
        for (int i = 0; i < N; i++) {
            if (!is_equal(m_data[i], 0.0)) return false;
        }
        return true;
    }


    /**
     * @brief Check if any component of the vector is NaN (Not a Number).
     * 
     * @return true If any component is NaN.
     * @return false If no component is NaN.
     */
    constexpr bool has_nan() const {
        for (int i = 0; i < N; i++) {
            if (std::isnan(m_data[i])) return true;
        }
        return false;
    }

    /**
     * @brief Divides this vector by a scalar.
     */
    template<std::convertible_to<T> U>
    constexpr Vector operator/=(const U& value) const noexcept {
        if (value == 0) {
            if (std::is_constant_evaluated()) {
                throw "Compile-time error: Division by zero";
            } else {
                throw std::runtime_error("Division by zero");
            }
        }
        Float inv = 1.0 / value;
        for (int i = 0; i < N; i++) m_data[i] *= inv;
        return *this;
    }

    // --- 向量数学运算 (Vector Math Operations) ---

    /**
     * @brief Calculates the squared length (magnitude) of the vector.
     * @details This is faster than `length()` as it avoids a square root.
     * It is often sufficient for comparing vector lengths.
     * @return The squared length of the vector.
     */
    constexpr T length_squared() const noexcept {
        T result = 0;
        for (int i = 0; i < N; i++) 
            result += m_data[i] * m_data[i];
        return result;
    }

    /**
     * @brief Calculates the length (Euclidean norm) of the vector.
     * @see length_squared()
     * @return The length of the vector.
     */
    constexpr T length() const {
        return pbpt::math::sqrt(length_squared());
    }

    /**
     * @brief Returns a new vector that is a normalized version of this one.
     * @details The new vector will have a length of 1.
     * @return A new unit vector.
     * @throw std::runtime_error If the vector's length is zero.
     * @note In a `constexpr` context, normalizing a zero vector will result in a compile-time error.
     */
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

    /**
     * @brief Normalizes this vector in-place, making its length 1.
     * @return A reference to this modified vector.
     * @throw std::runtime_error If the vector's length is zero.
     * @note In a `constexpr` context, normalizing a zero vector will result in a compile-time error.
     */
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
    
    /**
     * @brief Calculates the dot product of this vector and another.
     * @param rhs The other vector.
     * @return The scalar dot product.
     */
    constexpr T dot(const Vector& rhs) const noexcept {
        T result = 0;
        for (int i = 0; i < N; i++) result += m_data[i] * rhs.m_data[i];
        return result;
    }

    /**
     * @brief Calculates the cross product of this vector and another.
     * @param rhs The other vector.
     * @return The resulting vector perpendicular to the two input vectors.
     * @note This operation is only defined for 3D vectors (N == 3).
     */
    constexpr Vector cross(const Vector& rhs) const noexcept requires(N == 3) {
        return Vector(
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

    /**
     * @brief Applies a function to each element of the vector.
     * @param func The function to apply.
     */
    void apply(const std::function<void(T&, int)>& func) {
        for (int i = 0; i < N; ++i) func(m_data[i], i);
    }

    /** @brief Adds two vectors component-wise. */
    constexpr Vector operator+(const Vector<T, N>& rhs) const noexcept {
        auto result = *this;
        result += rhs;
        return result;
    }

    /** @brief Subtracts one vector from another component-wise. */
    constexpr Vector operator-(const Vector<T, N>& rhs) const noexcept {
        auto result = *this;
        result -= rhs;
        return result;
    }

    /** @brief Multiplies a vector by a scalar. */
    template<std::convertible_to<T> U>
    constexpr Vector operator*(U value) const noexcept {
        auto result = *this;
        result *= static_cast<T>(value);
        return result;
    }

    /** @brief Multiplies a scalar by a vector. */
    template<std::convertible_to<T> U>
    friend constexpr Vector operator*(U lhs, const Vector<T, N>& rhs) noexcept {
        return rhs * static_cast<T>(lhs);
    }

    /** * @brief Multiplies two vectors component-wise (Hadamard product).
    * @warning This is NOT a dot product or cross product.
    */
    constexpr Vector operator*(const Vector<T, N>& rhs) const noexcept {
        Vector<T, N> result{};
        for (int i = 0; i < N; i++) result[i] = (*this)[i] * rhs[i];
        return result;
    }

    /**
    * @brief Divides a vector by a scalar.
    */
    template <std::convertible_to<T> U>
    constexpr Vector operator/(U rhs) const {
        Vector result = *this;
        result /= static_cast<T>(rhs);
        return result;
    }

    /**
    * @brief Stream insertion operator for printing the vector.
    * @details Prints the vector in the format (c1, c2, ..., cN).
    * @param os The output stream.
    * @param vec The vector to print.
    * @return A reference to the output stream.
    */
    friend std::ostream& operator<<(std::ostream& os, const Vector& vec) {
        os << "Vec" << N << "(";
        for (int i = 0; i < N; ++i) {
            os << vec[i] << (i == N - 1 ? "" : ", ");
        }
        os << ')';
        return os;
    }

    /**
     * @brief Returns the dimension with the maximum value.
     * @return The dimension index with the maximum value.
     */
    constexpr int max_dim() const {
        return std::max_element(m_data.begin(), m_data.end()) - m_data.begin();
    }

    /**
     * @brief Returns the maximum value in the vector.
     * @return The maximum value.
     */
    constexpr T max() const {
        return *std::max_element(m_data.begin(), m_data.end());
    }

    /**
     * @brief Returns the dimension with the minimum value.
     * @return The dimension index with the minimum value.
     */
    constexpr int min_dim() const {
        return std::min_element(m_data.begin(), m_data.end()) - m_data.begin();
    }

    /**
     * @brief Returns the minimum value in the vector.
     * @return The minimum value.
     */
    constexpr T min() const {
        return *std::min_element(m_data.begin(), m_data.end());
    }

    /**
     * @brief Returns a new vector with the specified dimensions permuted.
     * @details The dimensions are specified by their indices, starting from 0.
     * @tparam Args The dimension indices to permute.
     * @return A new vector with the specified dimensions permuted.
     */
    template<typename ...Args>
    requires (sizeof...(Args) == N)
    constexpr Vector permuted(Args ...args) const {
        Vector result;
        int i = 0;
        ((result[i++] = (*this)[args]), ...);
        return result;
    }

    /**
     * @brief Permutes the dimensions of the vector.
     * @details The dimensions are specified by their indices, starting from 0.
     * @tparam Args The dimension indices to permute.
     * @return A reference to this vector with the dimensions permuted.
     */
    template<typename ...Args>
    requires (sizeof...(Args) == N)
    constexpr Vector& permute(Args ...args) {
        *this = permuted(args...);
        return *this;
    }
    
    
};

/**
* @brief Returns orthogonal bases for a given vector.
* @tparam Args The types of the orthogonal vectors.
* @param base The input vector.
* @return A vector of orthogonal vectors.
*/

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

/**
* @brief Returns orthogonal bases for a given vector when N == 2.
* @tparam Args The types of the orthogonal vectors.
* @param u The input vector.
* @return A vector of orthogonal vectors.
*/
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

/**
* @brief Returns orthogonal bases for a given vector when N == 3.
* @tparam Args The types of the orthogonal vectors.
* @param base The input vector.
* @return A vector of orthogonal vectors.
*/
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

// --- 类型别名 (Type Aliases) ---

/** @brief A 2-dimensional vector of type `Float`. */
using Vec2 = Vector<Float, 2>;
/** @brief A 3-dimensional vector of type `Float`. */
using Vec3 = Vector<Float, 3>;
/** @brief A 4-dimensional vector of type `Float`. */
using Vec4 = Vector<Float, 4>;

template <typename T, int N>
class Normal : public Vector<T, N> {
public:
    using Vector<T, N>::Vector;
    constexpr Normal(const Vector<T, N>& vec) : Vector<T, N>(vec.normalized()) {}
};

/** @brief A 2-dimensional vector of type `Float`. */
using Normal2 = Normal<Float, 2>;
/** @brief A 3-dimensional vector of type `Float`. */
using Normal3 = Normal<Float, 3>;
/** @brief A 4-dimensional vector of type `Float`. */
using Normal4 = Normal<Float, 4>;


} // namespace math