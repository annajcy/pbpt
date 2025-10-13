#pragma once

#include <string>
#include <sstream>
#include <iomanip>
#include <format>

#include "math/matrix.hpp"
#include "math/normal.hpp"
#include "math/octahedral.hpp"
#include "math/tuple.hpp"

namespace pbpt::math {

// =============================================================================
// Tuple-based types (Vector, Point, Normal, Homogeneous)
// =============================================================================

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

template <typename T, int R, int C>
std::string to_string(const Matrix<T, R, C>& m) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);
    oss << std::format("Matrix<{}, {}, {}>(", typeid(T).name(), R, C);
    oss << "\n";
    for (int r = 0; r < R; ++r) {
        oss << "\t[";
        for (int c = 0; c < C; ++c) {
            if (c > 0) oss << ", ";
            oss << m[r][c];
        }
        oss << "]\n";
    }
    oss << ")";
    return oss.str();
}

// =============================================================================
// Interval type
// =============================================================================

template <typename T>
std::string to_string(const Interval<T>& interval) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);
    oss << std::format("Interval<{}>(", typeid(T).name());
    
    if (interval.is_empty()) {
        oss << "empty";
    } else if (interval.is_point()) {
        oss << interval.m_low;
    } else {
        oss << "[" << interval.m_low << ", " << interval.m_high << "]";
    }
    oss << ")";
    return oss.str();
}

// =============================================================================
// OctahedralVector type
// =============================================================================

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
// Extended formatting functions with custom precision
// =============================================================================

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

template <typename T, int R, int C>
std::string to_string_precision(const Matrix<T, R, C>& m, int precision) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision);
    oss << std::format("Matrix<{}, {}, {}>(", typeid(T).name(), R, C);
    
    for (int r = 0; r < R; ++r) {
        if (r > 0) oss << ", ";
        oss << "[";
        for (int c = 0; c < C; ++c) {
            if (c > 0) oss << ", ";
            oss << m[r][c];
        }
        oss << "]";
    }
    oss << ")";
    return oss.str();
}

template <typename T>
std::string to_string_precision(const Interval<T>& interval, int precision) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision);
    oss << std::format("Interval<{}>(", typeid(T).name());
    
    if (interval.is_empty()) {
        oss << "empty";
    } else if (interval.is_point()) {
        oss << interval.m_low;
    } else {
        oss << "[" << interval.m_low << ", " << interval.m_high << "]";
    }
    oss << ")";
    return oss.str();
}

// =============================================================================
// Compact formatting functions (without type information)
// =============================================================================

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

template <typename T, int R, int C>
std::string to_string_compact(const Matrix<T, R, C>& m) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    oss << "[";
    
    for (int r = 0; r < R; ++r) {
        if (r > 0) oss << "; ";
        for (int c = 0; c < C; ++c) {
            if (c > 0) oss << " ";
            oss << m[r][c];
        }
    }
    oss << "]";
    return oss.str();
}

template <typename T>
std::string to_string_compact(const Interval<T>& interval) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    
    if (interval.is_empty()) {
        oss << "Empty";
    } else if (interval.is_point()) {
        oss << interval.m_low;
    } else {
        oss << "[" << interval.m_low << ", " << interval.m_high << "]";
    }
    return oss.str();
}

template <template <typename, int> typename Derived, typename T, int N>
std::ostream& operator<<(std::ostream& os, const Tuple<Derived, T, N>& t) {
    return os << to_string(t);
}

template <typename T, int R, int C>
std::ostream& operator<<(std::ostream& os, const Matrix<T, R, C>& m) {
    return os << to_string(m);
}

template <typename T>
std::ostream& operator<<(std::ostream& os, const Interval<T>& interval) {
    return os << to_string(interval);
}

template <typename T>
std::ostream& operator<<(std::ostream& os, const OctahedralVector<T>& oct) {
    return os << to_string(oct);
}

}  // namespace pbpt::math
