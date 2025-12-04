#pragma once

#include <array>
#include <type_traits>
#include <vector>
#include <cmath>

#include "type_alias.hpp"
#include "tuple.hpp"
#include "vector.hpp"

namespace pbpt::math {

/**
 * @brief Geometric point in N-dimensional space.
 *
 * A `Point` behaves like a `Tuple` of coordinates but has point
 * semantics: adding a vector translates the point, and subtracting
 * two points yields a displacement vector.
 *
 * @tparam T Scalar type.
 * @tparam N Dimension.
 */
template <typename T, int N>
class Point : public Tuple<Point, T, N> {
private:
    using Base = Tuple<Point, T, N>;
    using Base::m_data;

public:
    using Base::Base;

    /// Constructs a point from a vector of coordinates.
    /**
     * @brief Constructs a point from a vector of coordinates.
     *
     * This simply copies the components of @p vec into the point.
     */
    static constexpr auto from_vector(const Vector<T, N>& vec) {
        Point<T, N> p;
        for (int i = 0; i < N; ++i)
            p[i] = vec[i];
        return p;
    }

    /// Converts this point to a `Vector<T,N>` with the same coordinates.
    constexpr auto to_vector() const {
        return Vector<T, N>::from_array(this->to_array());
    }

    /// Translates the point by a vector and returns the result.
    template <typename U>
    constexpr auto operator+(const Vector<U, N>& v) const {
        using R = std::common_type_t<T, U>;
        Point<R, N> out;
        for (int i = 0; i < N; ++i)
            out[i] = static_cast<R>(this->m_data[i]) + static_cast<R>(v[i]);
        return out;
    }

    /// Subtracts a vector from the point and returns the result.
    template <typename U>
    constexpr auto operator-(const Vector<U, N>& v) const {
        using R = std::common_type_t<T, U>;
        Point<R, N> out;
        for (int i = 0; i < N; ++i)
            out[i] = static_cast<R>(this->m_data[i]) - static_cast<R>(v[i]);
        return out;
    }

    /// Returns the displacement vector from another point to this point.
    template <typename U>
    /// Translates this point in-place by a vector.
    constexpr auto operator-(const Point<U, N>& p) const {
        using R = std::common_type_t<T, U>;
        Vector<R, N> v;
        for (int i = 0; i < N; ++i)
            v[i] = static_cast<R>(this->m_data[i]) - static_cast<R>(p[i]);
        return v;
    }

    template <typename U>
    /// Subtracts a vector from this point in-place.
    constexpr auto& operator+=(const Vector<U, N>& v) {
        for (int i = 0; i < N; ++i)
            this->m_data[i] += static_cast<T>(v[i]);
        return *this;
    }

    template <typename U>
    /// Squared Euclidean distance to another point.
    constexpr auto& operator-=(const Vector<U, N>& v) {
        for (int i = 0; i < N; ++i)
            this->m_data[i] -= static_cast<T>(v[i]);
        return *this;
    }

    template <typename U>
    /// Euclidean distance to another point.
    constexpr auto distance_squared(const Point<U, N>& other) const {
        using R = std::common_type_t<T, U>;
        R sum   = R(0);
        for (int i = 0; i < N; ++i) {
            R d = static_cast<R>(this->m_data[i]) - static_cast<R>(other[i]);
            sum += d * d;
        }
        return sum;
    }

    template <typename U>
    /// Midpoint between this point and another point.
    constexpr auto distance(const Point<U, N>& other) const {
        using R = std::common_type_t<T, U>;
        return std::sqrt(static_cast<promote_int_to_float_t<R>>(distance_squared(other)));
    }

    /// Midpoint between this point and another point.
    template <typename U>
    constexpr auto mid(const Point<U, N>& other) const {
        using R = std::common_type_t<T, U>;
        Point<R, N> out;
        for (int i = 0; i < N; ++i)
            out[i] = (static_cast<R>(this->m_data[i]) + static_cast<R>(other[i])) * R(0.5);
        return out;
    }

    template<typename U>
    /// Arithmetic midpoint of a non-empty set of points.
    static constexpr auto mid(const std::vector<Point<U, N>>& points) {
        using R = std::common_type_t<T, U>;
        assert_if([&points]() { return points.empty(); }, "Cannot compute midpoint of empty point array");
        std::array<R, N> sum{};
        for (const auto& p : points)
            for (int i = 0; i < N; ++i)
                sum[i] += static_cast<R>(p[i]);
        for (int i = 0; i < N; ++i)
            sum[i] /= static_cast<R>(points.size());
        return Point<R, N>::from_array(sum);
    }
};

// Vector + Point = Point
template <typename U, typename T, int N>
    requires(N > 0)
constexpr auto operator+(const Vector<U, N>& v, const Point<T, N>& p) {
    return p + v;
}

// Aliases
using Pt1  = Point<Float, 1>;
using Pt2  = Point<Float, 2>;
using Pt3  = Point<Float, 3>;
using Pt4  = Point<Float, 4>;
using Pt1i = Point<Int, 1>;
using Pt2i = Point<Int, 2>;
using Pt3i = Point<Int, 3>;
using Pt4i = Point<Int, 4>;

} // namespace pbpt::math
