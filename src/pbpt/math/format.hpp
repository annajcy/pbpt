/**
 * @file
 * @brief Formatting helpers for math types (vectors, points, normals, matrices).
 */
#pragma once

#include <string>
#include <sstream>
#include <iomanip>
#include <format>

#include "pbpt/math/matrix.hpp"
#include "pbpt/math/normal.hpp"
#include "pbpt/math/octahedral.hpp"
#include "pbpt/math/tuple.hpp"
#include "pbpt/math/complex.hpp"

namespace pbpt::math {

// =============================================================================
// Tuple-based types (Vector, Point, Normal, Homogeneous)
// =============================================================================

/**
 * @brief Returns a readable name for tuple-based math types.
 *
 * Distinguishes between Vector, Point, Normal and Homogeneous instantiations
 * to produce a string such as `Vector<float, 3>`.
 */
template <template <typename, int> typename Derived, typename T, int N>
std::string tuple_name() {
    if constexpr (std::is_same_v<Derived<T, N>, Vector<T, N>>) {
        return std::format("Vector<{}, {}>", typeid(T).name(), N);
    } else if constexpr (std::is_same_v<Derived<T, N>, Point<T, N>>) {
        return std::format("Point<{}, {}>", typeid(T).name(), N);
    } else if constexpr (std::is_same_v<Derived<T, N>, Normal<T, N>>) {
        return std::format("Normal<{}, {}>", typeid(T).name(), N);
    } else if constexpr (std::is_same_v<Derived<T, N>, Homogeneous<T, N>>) {
        return std::format("Homogeneous<{}, {}>", typeid(T).name(), N);
    } else {
        return "Unknown";
    }
}

/**
 * @brief Format a tuple-based math type with type info and fixed precision.
 * @return String of the form `Vector<T, N>(v0, v1, ...)`.
 */
template <template <typename, int> typename Derived, typename T, int N>
std::string to_string(const Tuple<Derived, T, N>& t) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);
    oss << tuple_name<Derived, T, N>() << "(";
    for (int i = 0; i < N; ++i) {
        oss << t[i] << (i == N - 1 ? "" : ", ");
    }
    oss << ")";
    return oss.str();
}

// =============================================================================
// Matrix types
// =============================================================================

/**
 * @brief Format a matrix with type info and multi-line layout.
 */
template <typename T, int R, int C>
std::string to_string(const Matrix<T, R, C>& m) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);
    oss << std::format("Matrix<{}, {}, {}>(", typeid(T).name(), R, C);
    oss << "\n";
    for (int r = 0; r < R; ++r) {
        oss << "\t[";
        for (int c = 0; c < C; ++c) {
            if (c > 0)
                oss << ", ";
            oss << m[r][c];
        }
        oss << "]\n";
    }
    oss << ")";
    return oss.str();
}

// =============================================================================
// OctahedralVector type
// =============================================================================

/**
 * @brief Format an octahedral vector and its decoded XYZ components.
 */
template <typename T>
std::string to_string(const OctahedralVector<T>& oct) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);
    oss << std::format("OctahedralVector<{}>(", typeid(T).name());
    Vector<T, 3> decoded = oct.decode();
    oss << "x=" << decoded[0] << ", y=" << decoded[1] << ", z=" << decoded[2];
    oss << ")";
    return oss.str();
}

// =============================================================================
// Complex type
// =============================================================================

/**
 * @brief Format a complex number.
 */
template <typename T>
std::string to_string(const Complex<T>& c) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);
    oss << std::format("Complex<{}>(", typeid(T).name());
    oss << c.real() << ", " << c.imag() << ")";
    return oss.str();
}

// =============================================================================
// Extended formatting functions with custom precision
// =============================================================================

/**
 * @brief Format a tuple with caller-provided decimal precision.
 */
template <template <typename, int> typename Derived, typename T, int N>
std::string to_string_precision(const Tuple<Derived, T, N>& t, int precision) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision);
    oss << Derived<T, N>::name() << "(";
    for (int i = 0; i < N; ++i) {
        oss << t[i] << (i == N - 1 ? "" : ", ");
    }
    oss << ")";
    return oss.str();
}

/**
 * @brief Format a matrix with caller-provided decimal precision.
 */
template <typename T, int R, int C>
std::string to_string_precision(const Matrix<T, R, C>& m, int precision) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision);
    oss << std::format("Matrix<{}, {}, {}>(", typeid(T).name(), R, C);

    for (int r = 0; r < R; ++r) {
        if (r > 0)
            oss << ", ";
        oss << "[";
        for (int c = 0; c < C; ++c) {
            if (c > 0)
                oss << ", ";
            oss << m[r][c];
        }
        oss << "]";
    }
    oss << ")";
    return oss.str();
}

// =============================================================================
// Compact formatting functions (without type information)
// =============================================================================

/**
 * @brief Compact tuple formatter without type information.
 */
template <template <typename, int> typename Derived, typename T, int N>
std::string to_string_compact(const Tuple<Derived, T, N>& t) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    oss << "(";
    for (int i = 0; i < N; ++i) {
        oss << t[i] << (i == N - 1 ? "" : ", ");
    }
    oss << ")";
    return oss.str();
}

/**
 * @brief Compact single-line matrix formatter.
 */
template <typename T, int R, int C>
std::string to_string_compact(const Matrix<T, R, C>& m) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    oss << "[";

    for (int r = 0; r < R; ++r) {
        if (r > 0)
            oss << "; ";
        for (int c = 0; c < C; ++c) {
            if (c > 0)
                oss << " ";
            oss << m[r][c];
        }
    }
    oss << "]";
    return oss.str();
}

/**
 * @brief Stream insertion for tuple-based math types.
 */
template <template <typename, int> typename Derived, typename T, int N>
std::ostream& operator<<(std::ostream& os, const Tuple<Derived, T, N>& t) {
    return os << to_string(t);
}

/**
 * @brief Stream insertion for matrices.
 */
template <typename T, int R, int C>
std::ostream& operator<<(std::ostream& os, const Matrix<T, R, C>& m) {
    return os << to_string(m);
}

/**
 * @brief Stream insertion for octahedral vectors.
 */
template <typename T>
std::ostream& operator<<(std::ostream& os, const OctahedralVector<T>& oct) {
    return os << to_string(oct);
}

/**
 * @brief Stream insertion for complex numbers.
 */
template <typename T>
std::ostream& operator<<(std::ostream& os, const Complex<T>& c) {
    return os << to_string(c);
}

}  // namespace pbpt::math
