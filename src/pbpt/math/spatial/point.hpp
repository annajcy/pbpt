/**
 * @file
 * @brief Geometric point type built on top of Tuple and Vector.
 */
#pragma once

#include <array>
#include <cmath>
#include <type_traits>
#include <vector>

#include "pbpt/math/basic/algebra_concepts.hpp"
#include "pbpt/math/spatial/tuple.hpp"
#include "pbpt/math/basic/type_alias.hpp"
#include "pbpt/math/spatial/vector.hpp"

namespace pbpt::math {

/**
 * @brief Geometric point in N-dimensional space.
 *
 * Satisfies `AffineSpace<Point, Vector, T>`:
 *   - `Point + Vector -> Point`   (translation)
 *   - `Point - Vector -> Point`   (inverse translation)
 *   - `Point - Point  -> Vector`  (displacement)
 *   - `lerp(p, q, t)` and `lerp(a, b, c, t1, t2, t3)` (affine combinations)
 *   - `distance(p, q)`, `distance_squared(p, q)`
 *
 * Does NOT support:
 *   - `Point + Point`   (no additive structure)
 *   - `scalar * Point`  (no scaling)
 *
 * @tparam T  Scalar type.
 * @tparam N  Dimension.
 */
template <typename T, int N>
class Point : public Tuple<Point<T, N>, T, N> {
private:
    using Base = Tuple<Point<T, N>, T, N>;
    using Base::m_data;

public:
    using Base::Base;

    // -----------------------------------------------------------------------
    // Construction / conversion
    // -----------------------------------------------------------------------

    static constexpr Point from_vector(const Vector<T, N>& vec) {
        Point p;
        for (int i = 0; i < N; ++i)
            p[i] = vec[i];
        return p;
    }

    constexpr Vector<T, N> to_vector() const {
        return Vector<T, N>::from_array(this->to_array());
    }

    // -----------------------------------------------------------------------
    // Affine arithmetic
    // -----------------------------------------------------------------------

    /// Point + Vector = Point  (translation).
    template <typename U>
    constexpr auto operator+(const Vector<U, N>& v) const {
        using R = std::common_type_t<T, U>;
        Point<R, N> out;
        for (int i = 0; i < N; ++i)
            out[i] = static_cast<R>(m_data[i]) + static_cast<R>(v[i]);
        return out;
    }

    /// Point - Vector = Point  (inverse translation).
    template <typename U>
    constexpr auto operator-(const Vector<U, N>& v) const {
        using R = std::common_type_t<T, U>;
        Point<R, N> out;
        for (int i = 0; i < N; ++i)
            out[i] = static_cast<R>(m_data[i]) - static_cast<R>(v[i]);
        return out;
    }

    /// Point - Point = Vector  (displacement).
    template <typename U>
    constexpr auto operator-(const Point<U, N>& p) const {
        using R = std::common_type_t<T, U>;
        Vector<R, N> v;
        for (int i = 0; i < N; ++i)
            v[i] = static_cast<R>(m_data[i]) - static_cast<R>(p[i]);
        return v;
    }

    template <typename U>
    constexpr Point& operator+=(const Vector<U, N>& v) {
        for (int i = 0; i < N; ++i)
            m_data[i] += static_cast<T>(v[i]);
        return *this;
    }

    template <typename U>
    constexpr Point& operator-=(const Vector<U, N>& v) {
        for (int i = 0; i < N; ++i)
            m_data[i] -= static_cast<T>(v[i]);
        return *this;
    }

    // -----------------------------------------------------------------------
    // Distance metrics
    // -----------------------------------------------------------------------

    template <typename U>
    constexpr auto distance_squared(const Point<U, N>& other) const {
        using R = std::common_type_t<T, U>;
        R sum = R(0);
        for (int i = 0; i < N; ++i) {
            R d = static_cast<R>(m_data[i]) - static_cast<R>(other[i]);
            sum += d * d;
        }
        return sum;
    }

    template <typename U>
    constexpr auto distance(const Point<U, N>& other) const {
        using R = std::common_type_t<T, U>;
        return std::sqrt(static_cast<promote_int_to_float_t<R>>(distance_squared(other)));
    }

    // -----------------------------------------------------------------------
    // Midpoint helpers
    // -----------------------------------------------------------------------

    template <typename U>
    constexpr auto mid(const Point<U, N>& other) const {
        using R = std::common_type_t<T, U>;
        Point<R, N> out;
        for (int i = 0; i < N; ++i)
            out[i] = (static_cast<R>(m_data[i]) + static_cast<R>(other[i])) * R(0.5);
        return out;
    }

    template <typename U>
    static constexpr auto mid(const std::vector<Point<U, N>>& points) {
        using R = std::common_type_t<T, U>;
        assert_if([&points]() { return points.empty(); },
                  "Cannot compute midpoint of empty point array");
        std::array<R, N> sum{};
        for (const auto& p : points)
            for (int i = 0; i < N; ++i)
                sum[i] += static_cast<R>(p[i]);
        for (int i = 0; i < N; ++i)
            sum[i] /= static_cast<R>(points.size());
        return Point<R, N>::from_array(sum);
    }
};

// ---------------------------------------------------------------------------
// Vector + Point = Point  (commutative convenience overload)
// ---------------------------------------------------------------------------

template <typename U, typename T, int N>
    requires(N > 0)
constexpr auto operator+(const Vector<U, N>& v, const Point<T, N>& p) {
    return p + v;
}

// ---------------------------------------------------------------------------
// rebind_trait specialization
// ---------------------------------------------------------------------------

template <typename T, int N>
struct rebind_trait<Point<T, N>> {
    template <typename U, int M>
    using type = Point<U, M>;
};

// ---------------------------------------------------------------------------
// algebra_traits specialization
// ---------------------------------------------------------------------------

template <typename T, int N>
struct algebra_traits<Point<T, N>> {
    using scalar = T;
    static constexpr int dim = N;
};

// NOTE: zero_v<Point<T,N>> is deliberately NOT provided.
// Points do not form an additive monoid; this enforces !AdditiveMonoid<Point>
// in the AffineSpace concept.

// ---------------------------------------------------------------------------
// Affine free functions (required by AffineSpace concept for ADL lookup)
// ---------------------------------------------------------------------------

/// Linear interpolation between two points: (1-t)*p + t*q.
template <typename T, int N>
constexpr Point<T, N> lerp(const Point<T, N>& p, const Point<T, N>& q, T t) {
    Point<T, N> result;
    for (int i = 0; i < N; ++i)
        result[i] = p[i] * (T(1) - t) + q[i] * t;
    return result;
}

/// Barycentric combination of three points: t1*a + t2*b + t3*c.
template <typename T, int N>
constexpr Point<T, N> lerp(
    const Point<T, N>& a, const Point<T, N>& b, const Point<T, N>& c,
    T t1, T t2, T t3) {
    Point<T, N> result;
    for (int i = 0; i < N; ++i)
        result[i] = a[i] * t1 + b[i] * t2 + c[i] * t3;
    return result;
}

/// Euclidean distance between two points.
template <typename T, int N>
constexpr auto distance(const Point<T, N>& p, const Point<T, N>& q) {
    return p.distance(q);
}

/// Squared Euclidean distance between two points.
template <typename T, int N>
constexpr auto distance_squared(const Point<T, N>& p, const Point<T, N>& q) {
    return p.distance_squared(q);
}

// ---------------------------------------------------------------------------
// Type aliases
// ---------------------------------------------------------------------------

using Pt1 = Point<Float, 1>;
using Pt2 = Point<Float, 2>;
using Pt3 = Point<Float, 3>;
using Pt4 = Point<Float, 4>;

using Pt1i = Point<Int, 1>;
using Pt2i = Point<Int, 2>;
using Pt3i = Point<Int, 3>;
using Pt4i = Point<Int, 4>;

}  // namespace pbpt::math
