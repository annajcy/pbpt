#pragma once

#include <concepts>
#include <iostream>
#include <stdexcept>
#include <type_traits>

#include "matrix.hpp"
#include "point.hpp"

/**
 * @file homogeneous.hpp
 * @brief Defines a class for N-dimensional homogeneous coordinates.
 * @details This file provides the `Homo` class, which is essential for
 * performing 3D transformations in a unified manner. It allows points and
 * vectors to be treated consistently when multiplied by transformation
 * matrices. The key idea is to represent an N-dimensional concept using an
 * (N+1)-dimensional vector.
 */

namespace pbpt::math {

/**
 * @class Homogeneous
 * @brief A template class for an N-dimensional homogeneous coordinate.
 * @details This class unifies the mathematical treatment of points and vectors
 * for use in projective geometry, particularly for 3D transformations. It
 * internally stores an (N+1)-dimensional vector.
 *
 * The distinction between a point and a vector is encoded in the last
 * component, `w`:
 * - A **Point** `(x, y, z)` is represented in homogeneous coordinates as `(x,
 * y, z, 1)`.
 * - A **Vector** `(vx, vy, vz)` is represented as `(vx, vy, vz, 0)`.
 *
 * This class provides a type-safe way to create these coordinates from `Point`
 * and `Vec` objects, and to convert them back, including performing the
 * necessary perspective divide for points.
 *
 * @tparam T The underlying floating-point type of the coordinate's components.
 * @tparam N The original number of dimensions (e.g., 3 for 3D space). The
 * internal storage will have N+1 dimensions.
 * @see Point
 * @see Vec
 * @see Matrix
 */
template <typename T, int N>
class Homogeneous {
private:
    Vector<T, N + 1> m_data{};

public:
    constexpr Homogeneous() noexcept { m_data[N] = 1; }
    constexpr explicit Homogeneous(const Vector<T, N + 1>& data) noexcept : m_data(data) {}

    constexpr explicit Homogeneous(const Point<T, N>& p) noexcept {
        for (int i = 0; i < N; ++i)
            m_data[i] = p[i];
        m_data[N] = 1;
    }

    constexpr explicit Homogeneous(const Vector<T, N>& v) noexcept {
        for (int i = 0; i < N; ++i)
            m_data[i] = v[i];
        m_data[N] = 0;
    }

    template <std::convertible_to<T>... Vals>
    constexpr explicit Homogeneous(Vals... vals) noexcept
        requires(sizeof...(Vals) == N + 1)
    {
        int i = 0;
        ((m_data[i++] = vals), ...);
    }

    constexpr const T& w() const { return m_data[N]; }
    constexpr T&       w() { return m_data[N]; }

    constexpr const T& operator[](int index) const { return m_data[index]; }
    constexpr T&       operator[](int index) { return m_data[index]; }
    constexpr const T& at(int index) const { return m_data.at(index); }

    constexpr bool is_point() const noexcept { return m_data[N] != 0; }
    constexpr bool is_vector() const noexcept { return m_data[N] == 0; }

    constexpr Point<T, N> to_point() const {
        if (is_vector()) {
            if (std::is_constant_evaluated())
                throw "Compile-time error: Cannot convert a homogeneous vector "
                      "(w=0) to a Point.";
            else
                throw std::runtime_error("Cannot convert a homogeneous vector (w=0) to a Point.");
        }
        Vector<T, N> result_coords;
        const T      inv_w = 1.0 / m_data[N];
        for (int i = 0; i < N; ++i)
            result_coords[i] = m_data[i] * inv_w;
        return Point<T, N>::from_vector(result_coords);
    }

    constexpr Vector<T, N> to_vector() const {
        if (is_point()) {
            if (std::is_constant_evaluated())
                throw "Compile-time error: Cannot convert a homogeneous point "
                      "(w!=0) to a Vec.";
            else
                throw std::runtime_error(
                    "Logical error: Cannot convert a "
                    "homogeneous point (w!=0) to a "
                    "Vec.");
        }
        Vector<T, N> result_coords;
        for (int i = 0; i < N; ++i)
            result_coords[i] = m_data[i];
        return result_coords;
    }

    constexpr const Vector<T, N + 1>& raw() const noexcept { return m_data; }
    constexpr Vector<T, N + 1>&       raw() noexcept { return m_data; }

    friend std::ostream& operator<<(std::ostream& os, const Homogeneous& h) {
        os << "HCoord" << N << (h.is_point() ? "[P] " : "[V] ") << h.raw();
        return os;
    }

    friend Homogeneous operator*(const Matrix<T, N + 1, N + 1>& mat, const Homogeneous& h) {
        return Homogeneous(mat * h.raw());
    }
};

using Homo3 = Homogeneous<Float, 3>;
using Homo2 = Homogeneous<Float, 2>;

}  // namespace pbpt::math