#pragma once

#include <array>
#include <type_traits>
#include <vector>
#include <cmath>

#include "interval.hpp"
#include "type_alias.hpp"
#include "tuple.hpp"
#include "vector.hpp"

namespace pbpt::math {

template <typename T, int N>
class Point : public Tuple<Point, T, N> {
private:
    using Base = Tuple<Point, T, N>;
    using Base::m_data;

public:
    using Base::Base;

    static constexpr auto from_vector(const Vector<T, N>& vec) {
        Point<T, N> p;
        for (int i = 0; i < N; ++i)
            p[i] = vec[i];
        return p;
    }

    constexpr auto to_vector() const {
        return Vector<T, N>::from_array(this->to_array());
    }

    // Point + Vector = Point
    template <typename U>
    constexpr auto operator+(const Vector<U, N>& v) const {
        using R = std::common_type_t<T, U>;
        Point<R, N> out;
        for (int i = 0; i < N; ++i)
            out[i] = static_cast<R>(this->m_data[i]) + static_cast<R>(v[i]);
        return out;
    }

    // Point - Vector = Point
    template <typename U>
    constexpr auto operator-(const Vector<U, N>& v) const {
        using R = std::common_type_t<T, U>;
        Point<R, N> out;
        for (int i = 0; i < N; ++i)
            out[i] = static_cast<R>(this->m_data[i]) - static_cast<R>(v[i]);
        return out;
    }

    // Point - Point = Vector
    template <typename U>
    constexpr auto operator-(const Point<U, N>& p) const {
        using R = std::common_type_t<T, U>;
        Vector<R, N> v;
        for (int i = 0; i < N; ++i)
            v[i] = static_cast<R>(this->m_data[i]) - static_cast<R>(p[i]);
        return v;
    }

    template <typename U>
    constexpr auto& operator+=(const Vector<U, N>& v) {
        for (int i = 0; i < N; ++i)
            this->m_data[i] += static_cast<T>(v[i]);
        return *this;
    }

    template <typename U>
    constexpr auto& operator-=(const Vector<U, N>& v) {
        for (int i = 0; i < N; ++i)
            this->m_data[i] -= static_cast<T>(v[i]);
        return *this;
    }

    template <typename U>
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
    constexpr auto distance(const Point<U, N>& other) const {
        using R = std::common_type_t<T, U>;
        return std::sqrt(static_cast<promote_int_to_float_t<R>>(distance_squared(other)));
    }

    template <typename U>
    constexpr auto mid(const Point<U, N>& other) const {
        using R = std::common_type_t<T, U>;
        Point<R, N> out;
        for (int i = 0; i < N; ++i)
            out[i] = (static_cast<R>(this->m_data[i]) + static_cast<R>(other[i])) * R(0.5);
        return out;
    }

    template<typename U>
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

    template <typename U>
    constexpr Point clamp(const Point<U, N>& low, const Point<U, N>& high) const {
        Point out;
        for (int i = 0; i < N; ++i) {
            auto v = this->m_data[i];
            auto l = static_cast<T>(low[i]);
            auto h = static_cast<T>(high[i]);
            out[i] = v < l ? l : (v > h ? h : v);
        }
        return out;
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

template <std::floating_point T, int N>
using PointInterval = Point<Interval<T>, N>;

using Pt3Interv = PointInterval<float, 3>;

template <std::floating_point T, int N>
inline constexpr Point<T, N> to_point(const PointInterval<T, N>& pi) {
    Point<T, N> p;
    for (int i = 0; i < N; ++i) {
        auto& interv = pi[i];
        p[i] = interv.midpoint();
    }
    return p;
}

} // namespace pbpt::math
