/**
 * @file
 * @brief Fixed-size vector type and geometric helpers (cross product, bases).
 */
#pragma once

#include <cmath>
#include <concepts>
#include <type_traits>

#include "pbpt/math/basic/algebra_concepts.hpp"
#include "pbpt/math/basic/function.hpp"
#include "pbpt/math/spatial/tuple.hpp"

namespace pbpt::math {

/**
 * @brief Fixed-size mathematical vector with arithmetic operations.
 *
 * Inherits storage and utilities from Tuple.  All arithmetic is defined here
 * as member functions; free-function wrappers (`dot`, `length`, `normalized`)
 * in `ops/arithmetic.hpp` allow concept satisfaction.
 *
 * @tparam T  Scalar type.
 * @tparam N  Dimension.
 */
template <typename T, int N>
class Vector : public Tuple<Vector<T, N>, T, N> {
private:
    using Base = Tuple<Vector<T, N>, T, N>;

protected:
    using Base::m_data;

public:
    using Base::Base;
    using Base::operator-;

    /// Extra constructor: Vec4 from Vec3 + scalar w.
    template <typename U>
        requires(N == 4 && std::is_convertible_v<U, T>)
    constexpr Vector(const Vector<U, 3>& xyz, U w)
        : Base(static_cast<T>(xyz.x()), static_cast<T>(xyz.y()), static_cast<T>(xyz.z()), static_cast<T>(w)) {}

    // -----------------------------------------------------------------------
    // Compound assignment
    // -----------------------------------------------------------------------

    template <typename U>
    constexpr auto& operator+=(const Vector<U, N>& rhs) {
        return this->add_assign_impl(rhs);
    }

    template <typename U>
    constexpr auto& operator-=(const Vector<U, N>& rhs) {
        return this->sub_assign_impl(rhs);
    }

    template <typename U>
        requires std::is_arithmetic_v<U>
    constexpr auto& operator*=(U rhs) {
        return this->mul_scalar_impl(rhs);
    }

    template <typename U>
        requires std::is_arithmetic_v<U>
    constexpr auto& operator/=(U rhs) {
        return this->div_scalar_impl(rhs);
    }

    template <typename U>
    constexpr auto& operator*=(const Vector<U, N>& rhs) {
        return this->mul_vec_impl(rhs);
    }

    template <typename U>
    constexpr auto& operator/=(const Vector<U, N>& rhs) {
        return this->div_vec_impl(rhs);
    }

    // -----------------------------------------------------------------------
    // Binary arithmetic (return new object)
    // -----------------------------------------------------------------------

    template <typename U>
    constexpr auto operator+(const Vector<U, N>& rhs) const {
        using R = std::common_type_t<T, U>;
        Vector<R, N> result{};
        for (int i = 0; i < N; ++i)
            result[i] = static_cast<R>((*this)[i]) + static_cast<R>(rhs[i]);
        return result;
    }

    template <typename U>
    constexpr auto operator-(const Vector<U, N>& rhs) const {
        using R = std::common_type_t<T, U>;
        Vector<R, N> result{};
        for (int i = 0; i < N; ++i)
            result[i] = static_cast<R>((*this)[i]) - static_cast<R>(rhs[i]);
        return result;
    }

    template <typename U>
        requires std::is_arithmetic_v<U>
    constexpr auto operator*(U value) const {
        using R = std::common_type_t<T, U>;
        Vector<R, N> result{};
        for (int i = 0; i < N; ++i)
            result[i] = static_cast<R>(m_data[i]) * static_cast<R>(value);
        return result;
    }

    /// Left scalar multiply (friend non-member).
    template <typename U>
        requires std::is_arithmetic_v<U>
    friend constexpr auto operator*(U value, const Vector<T, N>& rhs) {
        return rhs * value;
    }

    template <typename U>
    constexpr auto operator/(U value) const {
        using R = std::common_type_t<T, U>;
        Vector<R, N> result{};
        for (int i = 0; i < N; ++i) {
            assert_if([&value]() { return is_equal(value, 0.0); }, "Division by zero in vector division");
            result[i] = static_cast<R>(m_data[i]) / static_cast<R>(value);
        }
        return result;
    }

    template <typename U>
    constexpr auto operator*(const Vector<U, N>& rhs) const {
        using R = std::common_type_t<T, U>;
        Vector<R, N> result{};
        for (int i = 0; i < N; ++i)
            result[i] = static_cast<R>((*this)[i]) * static_cast<R>(rhs[i]);
        return result;
    }

    template <typename U>
    constexpr auto operator/(const Vector<U, N>& rhs) const {
        using R = std::common_type_t<T, U>;
        Vector<R, N> result{};
        for (int i = 0; i < N; ++i)
            result[i] = static_cast<R>((*this)[i]) / static_cast<R>(rhs[i]);
        return result;
    }

    // -----------------------------------------------------------------------
    // Length / normalisation
    // -----------------------------------------------------------------------

    constexpr auto length_squared() const {
        T result = T(0);
        for (int i = 0; i < N; ++i)
            result += m_data[i] * m_data[i];
        return result;
    }

    constexpr auto length() const { return std::sqrt(static_cast<promote_int_to_float_t<T>>(length_squared())); }

    constexpr auto normalized() const {
        using R = promote_int_to_float_t<T>;
        Vector<R, N> result{};
        auto len = length();
        assert_if([&len]() { return is_equal(len, 0.0); }, "Cannot normalize a zero vector");
        for (int i = 0; i < N; ++i)
            result[i] = static_cast<R>((*this)[i]) / static_cast<R>(len);
        return result;
    }

    constexpr bool is_normalized() const { return is_equal(length(), T(1.0)); }

    // -----------------------------------------------------------------------
    // Dot product / component product
    // -----------------------------------------------------------------------

    template <typename U>
    constexpr auto dot(const Vector<U, N>& rhs) const {
        using R = std::common_type_t<T, U>;
        R result = R(0);
        for (int i = 0; i < N; ++i)
            result += static_cast<R>((*this)[i]) * static_cast<R>(rhs[i]);
        return result;
    }

    constexpr T product() const {
        T result = T(1);
        for (int i = 0; i < N; ++i)
            result *= (*this)[i];
        return result;
    }

    /// Component-wise reciprocal with float promotion.
    constexpr auto inv() const {
        using R = promote_int_to_float_t<T>;
        Vector<R, N> result{};
        for (int i = 0; i < N; ++i) {
            assert_if([&]() { return is_equal((*this)[i], 0.0); }, "Cannot invert zero component");
            result[i] = static_cast<R>(1) / static_cast<R>((*this)[i]);
        }
        return result;
    }
};

// ---------------------------------------------------------------------------
// rebind_trait specialization
// ---------------------------------------------------------------------------

template <typename T, int N>
struct rebind_trait<Vector<T, N>> {
    template <typename U, int M>
    using type = Vector<U, M>;
};

// ---------------------------------------------------------------------------
// algebra_traits specialization
// ---------------------------------------------------------------------------

template <typename T, int N>
struct algebra_traits<Vector<T, N>> {
    using scalar = T;
    static constexpr int dim = N;
};

// ---------------------------------------------------------------------------
// zero_v specialization (required by AdditiveIdentity / VectorSpace)
// ---------------------------------------------------------------------------

template <typename T, int N>
inline constexpr Vector<T, N> zero_v<Vector<T, N>> = Vector<T, N>::zeros();

// ---------------------------------------------------------------------------
// Free functions
// ---------------------------------------------------------------------------

/// length(v) — required by Normed<Vector> and GLM-style compat.
template <typename T, int N>
constexpr auto length(const Vector<T, N>& v) {
    return v.length();
}

/// length_squared(v) — required by Normed<Vector>.
template <typename T, int N>
constexpr auto length_squared(const Vector<T, N>& v) {
    return v.length_squared();
}

/// normalized(v) — required by Normed<Vector>.
template <typename T, int N>
constexpr auto normalized(const Vector<T, N>& v) {
    return v.normalized();
}

/// normalize(v) — GLM-style alias for normalized().
template <typename T, int N>
constexpr auto normalize(const Vector<T, N>& v) {
    return v.normalized();
}

/// dot(v, u) — required by InnerProduct<Vector, S>. Supports cross-scalar types.
template <typename T, typename U, int N>
constexpr auto dot(const Vector<T, N>& lhs, const Vector<U, N>& rhs) {
    return lhs.dot(rhs);
}

/**
 * @brief 3D cross product between two vectors.
 */
template <typename T>
constexpr auto cross(const Vector<T, 3>& lhs, const Vector<T, 3>& rhs) {
    return Vector<T, 3>(std::fma(lhs.y(), rhs.z(), -lhs.z() * rhs.y()), std::fma(lhs.z(), rhs.x(), -lhs.x() * rhs.z()),
                        std::fma(lhs.x(), rhs.y(), -lhs.y() * rhs.x()));
}

/**
 * @brief Numerically stable angle between two normalized 3D vectors.
 */
template <typename T>
constexpr promote_int_to_float_t<T> angle_between(const Vector<T, 3>& v1, const Vector<T, 3>& v2) {
    assert_if([&v1, &v2]() { return v1.is_all_zero() || v2.is_all_zero(); },
              "Cannot compute angle between zero vectors");
    assert_if([&v1, &v2]() { return !v1.is_normalized() || !v2.is_normalized(); },
              "Vectors must be normalized to compute angle between them");
    if (v1.dot(v2) < 0)
        return pi_v<T> - 2 * safe_asin((v1 + v2).length() / 2);
    else
        return 2 * safe_asin((v1 - v2).length() / 2);
}

/**
 * @brief Builds an orthonormal basis {v2, v3} around a unit vector v1.
 */
template <std::floating_point T>
constexpr std::pair<Vector<T, 3>, Vector<T, 3>> coordinate_system(const Vector<T, 3>& v1) {
    assert_if([&v1]() { return !v1.is_normalized(); }, "Input vector to coordinate_system must be normalized");
    const T sign = std::copysign(T(1), v1.z());
    const T a = T(-1) / (sign + v1.z());
    const T b = v1.x() * v1.y() * a;
    Vector<T, 3> v2(T(1) + sign * v1.x() * v1.x() * a, sign * b, -sign * v1.x());
    Vector<T, 3> v3(b, sign + v1.y() * v1.y() * a, -v1.y());
    return {v2.normalized(), v3.normalized()};
}

// ---------------------------------------------------------------------------
// Type aliases
// ---------------------------------------------------------------------------

using Vec1 = Vector<Float, 1>;
using Vec2 = Vector<Float, 2>;
using Vec3 = Vector<Float, 3>;
using Vec4 = Vector<Float, 4>;

using Vec1i = Vector<Int, 1>;
using Vec2i = Vector<Int, 2>;
using Vec3i = Vector<Int, 3>;
using Vec4i = Vector<Int, 4>;

}  // namespace pbpt::math
