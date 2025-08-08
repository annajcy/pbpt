#pragma once

#include "tuple.hpp"

#include <cmath>
#include <concepts>
#include <format>
#include <type_traits>

namespace pbpt::math {

template <typename T, int N>
    requires(N > 0) && (std::is_floating_point_v<T> || std::is_integral_v<T>)
class Vector : public Tuple<Vector, T, N> {
private:
    using Base = Tuple<Vector, T, N>;
    using Base::Base;
    using Base::m_data;

public:
    using Base::operator[];
    using Base::at;
    using Base::to_array;
    using Base::from_array;
    using Base::cast;
    using Base::type_cast;
    using Base::dim_cast;
    using Base::x;
    using Base::y;
    using Base::z;
    using Base::w;

    static std::string name() { return std::format("Vector<{}, {}>", typeid(T).name(), N); }

    constexpr auto operator-() const {
        Vector<T, N> result{};
        for (int i = 0; i < N; i++)
            result[i] = -(*this)[i];
        return result;
    }

    template <typename U>
    constexpr auto& operator+=(const Vector<U, N>& rhs) {
        for (int i = 0; i < N; i++)
            (*this)[i] += static_cast<T>(rhs[i]);
        return *this;
    }

    template <typename U>
    constexpr auto& operator-=(const Vector<U, N>& rhs) {
        for (int i = 0; i < N; i++)
            (*this)[i] -= static_cast<T>(rhs[i]);
        return *this;
    }

    template <typename U>
    constexpr auto& operator*=(const U& rhs) {
        for (int i = 0; i < N; i++)
            (*this)[i] *= static_cast<T>(rhs);
        return *this;
    }

    template <typename U>
    constexpr auto& operator*=(const Vector<U, N>& rhs) {
        for (int i = 0; i < N; i++)
            (*this)[i] *= static_cast<T>(rhs[i]);
        return *this;
    }

    template <typename U>
    constexpr auto& operator/=(const Vector<U, N>& rhs) {
        for (int i = 0; i < N; i++) {
            assert_if([&rhs, i]() { return is_equal(rhs[i], 0.0); }, "Division by zero in vector division");
            (*this)[i] /= static_cast<T>(rhs[i]);
        }
        return *this;
    }

    template <typename U>
    constexpr bool operator==(const Vector<U, N>& rhs) const {
        for (int i = 0; i < N; i++) {
            if (is_not_equal(m_data[i], rhs[i]))
                return false;
        }
        return true;
    }

    template <typename U>
    constexpr bool operator!=(const Vector<U, N>& rhs) const {
        for (int i = 0; i < N; i++) {
            if (is_equal(m_data[i], rhs[i]))
                return false;
        }
        return true;
    }

    constexpr bool is_zero() const {
        for (int i = 0; i < N; i++) {
            if (!is_equal(m_data[i], 0.0))
                return false;
        }
        return true;
    }

    constexpr bool has_nan() const {
        if constexpr (std::is_floating_point_v<T>) {
            for (int i = 0; i < N; ++i)
                if (std::isnan(m_data[i]))
                    return true;
        }
        return false;
    }

    constexpr auto length_squared() const {
        auto result = T(0);
        for (int i = 0; i < N; i++)
            result += m_data[i] * m_data[i];
        return result;
    }

    constexpr auto length() const { return std::sqrt(static_cast<promote_int_to_float_t<T>>(length_squared())); }

    constexpr auto normalized() const {
        using R             = promote_int_to_float_t<T>;
        Vector<R, N> result = *this;
        auto         len    = length();
        assert_if([&len]() { return is_equal(len, 0.0); }, "Cannot normalize a zero vector");
        for (int i = 0; i < N; i++)
            result[i] /= static_cast<R>(len);
        return result;
    }

    constexpr bool is_normalized() const { return is_equal(length(), T(1.0)); }

    template <typename U>
    constexpr auto dot(const Vector<U, N>& rhs) const {
        using R  = std::common_type_t<T, U>;
        R result = 0;
        for (int i = 0; i < N; i++)
            result += static_cast<R>((*this)[i]) * static_cast<R>(rhs[i]);
        return result;
    }

    constexpr T product() const {
        T result = 1;
        for (int i = 0; i < N; i++)
            result *= (*this)[i];
        return result;
    }

    template <std::invocable<T&, int> F>
    constexpr void apply(F&& f) {
        for (int i = 0; i < N; ++i)
            f((*this)[i], i);
    }

    template <typename U>
    constexpr auto operator+(const Vector<U, N>& rhs) const {
        using R = std::common_type_t<T, U>;
        Vector<R, N> result{};
        for (int i = 0; i < N; i++) {
            result[i] = static_cast<R>((*this)[i]) + static_cast<R>(rhs[i]);
        }
        return result;
    }

    template <typename U>
    constexpr auto operator-(const Vector<U, N>& rhs) const {
        using R = std::common_type_t<T, U>;
        Vector<R, N> result{};
        for (int i = 0; i < N; i++) {
            result[i] = static_cast<R>((*this)[i]) - static_cast<R>(rhs[i]);
        }
        return result;
    }

    template <typename U>
        requires std::is_arithmetic_v<U>
    friend constexpr auto operator*(U value, const Vector<T, N>& rhs) {
        using R = std::common_type_t<T, U>;
        Vector<R, N> result{};
        for (int i = 0; i < N; i++) {
            result[i] = static_cast<R>(value) * static_cast<R>(rhs.m_data[i]);
        }
        return result;
    }

    template <typename U>
    constexpr auto operator*(U value) const {
        using R = std::common_type_t<T, U>;
        Vector<R, N> result{};
        for (int i = 0; i < N; i++) {
            result[i] = static_cast<R>(this->m_data[i]) * static_cast<R>(value);
        }
        return result;
    }

    template <typename U>
    constexpr auto operator/(U value) const {
        using R = std::common_type_t<T, U>;
        Vector<R, N> result{};
        for (int i = 0; i < N; i++) {
            assert_if([&value]() { return is_equal(value, 0.0); }, "Division by zero in vector division");
            result[i] = static_cast<R>(this->m_data[i]) / static_cast<R>(value);
        }
        return result;
    }

    template <typename U>
    constexpr auto operator*(const Vector<U, N>& rhs) const {
        using R = std::common_type_t<T, U>;
        Vector<R, N> result{};
        for (int i = 0; i < N; i++) {
            result[i] = static_cast<R>((*this)[i]) * static_cast<R>(rhs[i]);
        }
        return result;
    }

    template <typename U>
    constexpr auto operator/(const Vector<U, N>& rhs) const {
        using R = std::common_type_t<T, U>;
        Vector<R, N> result{};
        for (int i = 0; i < N; i++) {
            result[i] = static_cast<R>((*this)[i]) / static_cast<R>(rhs[i]);
        }
        return result;
    }

    constexpr auto inv() const {
        using R = promote_int_to_float_t<T>;
        Vector<R, N> result{};
        for (int i = 0; i < N; i++) {
            assert_if([&]() { return is_equal((*this)[i], 0.0); }, "Cannot invert zero component");
            result[i] = static_cast<R>(1) / (*this)[i];
        }
        return result;
    }

    constexpr auto abs() const {
        Vector<T, N> result{};
        for (int i = 0; i < N; i++) {
            result[i] = pbpt::math::abs((*this)[i]);
        }
        return result;
    }

    int max_dim() const {
        int max_i = 0;
        for (int i = 1; i < N; ++i)
            if (is_greater((*this)[i], (*this)[max_i]))
                max_i = i;
        return max_i;
    }

    T max() const {
        T max_v = (*this)[0];
        for (int i = 1; i < N; ++i)
            if (is_greater((*this)[i], max_v))
                max_v = (*this)[i];
        return max_v;
    }

    int min_dim() const {
        int min_i = 0;
        for (int i = 1; i < N; ++i)
            if (is_less((*this)[i], (*this)[min_i]))
                min_i = i;
        return min_i;
    }

    T min() const {
        T min_v = (*this)[0];
        for (int i = 1; i < N; ++i)
            if (is_less((*this)[i], min_v))
                min_v = (*this)[i];
        return min_v;
    }

    template <typename... Args>
        requires(sizeof...(Args) == N)
    constexpr auto permuted(Args... args) const {
        Vector<T, N> result;
        int          i = 0;
        ((result[i++] = (*this)[args]), ...);
        return result;
    }

    template <typename... Args>
        requires(sizeof...(Args) == N)
    constexpr auto& permute(Args... args) {
        *this = permuted(args...);
        return *this;
    }
};

template <typename T>
constexpr auto cross(const Vector<T, 3>& lhs, const Vector<T, 3>& rhs) {
    return Vector<T, 3>(
        std::fma(lhs.y(), rhs.z(), -lhs.z() * rhs.y()), 
        std::fma(lhs.z(), rhs.x(), -lhs.x() * rhs.z()),
        std::fma(lhs.x(), rhs.y(), -lhs.y() * rhs.x())
    );
}

template <typename T>
constexpr promote_int_to_float_t<T> angle_between(const Vector<T, 3>& v1, const Vector<T, 3>& v2) {
    assert_if([&v1, &v2]() { return v1.is_zero() || v2.is_zero(); }, "Cannot compute angle between zero vectors");
    assert_if([&v1, &v2]() { return !v1.is_normalized() || !v2.is_normalized(); },
              "Vectors must be normalized to compute angle between them");
    if (v1.dot(v2) < 0) {
        return pi_v<T> - 2 * safe_asin((v1 + v2).length() / 2);
    } else {
        return 2 * safe_asin((v1 - v2).length() / 2);
    }
}

template <std::floating_point T>
constexpr std::pair<Vector<T, 3>, Vector<T, 3>> coordinate_system(const Vector<T, 3>& v1) {
    assert_if([&v1]() { return !v1.is_normalized(); }, "Input vector to coordinate_system_stable() must be normalized");

    const T sign = std::copysign(T(1), v1.z());
    const T a    = T(-1) / (sign + v1.z());
    const T b    = v1.x() * v1.y() * a;

    Vector<T, 3> v2(T(1) + sign * v1.x() * v1.x() * a, sign * b, -sign * v1.x());

    Vector<T, 3> v3(b, sign + v1.y() * v1.y() * a, -v1.y());

    return {v2.normalized(), v3.normalized()};
}

using Vec1 = Vector<Float, 1>;
using Vec2 = Vector<Float, 2>;
using Vec3 = Vector<Float, 3>;
using Vec4 = Vector<Float, 4>;

using Vec1i = Vector<Int, 1>;
using Vec2i = Vector<Int, 2>;
using Vec3i = Vector<Int, 3>;
using Vec4i = Vector<Int, 4>;

}  // namespace pbpt::math