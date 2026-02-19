/**
 * @file
 * @brief Fixed-size vector type and geometric helpers (cross product, bases).
 */
#pragma once

#include "function.hpp"
#include "vector_base.hpp"

#include <cmath>
#include <concepts>
#include <type_traits>

namespace pbpt::math {

/**
 * @brief Fixed-size mathematical vector with arithmetic operations.
 *
 * `Vector` is a thin wrapper over `VectorOps` that provides a
 * convenient alias for numeric vectors in N dimensions.
 *
 * @tparam T Scalar type.
 * @tparam N Dimension.
 */
template <typename T, int N>
class Vector : public VectorBase<Vector, T, N> {
private:
    using Base = VectorBase<Vector, T, N>;

public:
    using Base::Base;

    template <typename U>
        requires(N == 4 && std::is_convertible_v<U, T>)
    constexpr Vector(const Vector<U, 3>& xyz, U w)
        : Base(static_cast<T>(xyz.x()), static_cast<T>(xyz.y()), static_cast<T>(xyz.z()), static_cast<T>(w)) {}
};

/**
 * @brief 3D cross product between two vectors.
 */
template <typename T>
constexpr auto cross(const Vector<T, 3>& lhs, const Vector<T, 3>& rhs) {
    return Vector<T, 3>(std::fma(lhs.y(), rhs.z(), -lhs.z() * rhs.y()), std::fma(lhs.z(), rhs.x(), -lhs.x() * rhs.z()),
                        std::fma(lhs.x(), rhs.y(), -lhs.y() * rhs.x()));
}

/**
 * @brief Angle between two normalized 3D vectors.
 *
 * Uses a numerically stable formulation based on the length of
 * v1 ± v2 rather than directly calling arccos on the dot product.
 * Both inputs must be normalized.
 */
template <typename T>
constexpr promote_int_to_float_t<T> angle_between(const Vector<T, 3>& v1, const Vector<T, 3>& v2) {
    assert_if([&v1, &v2]() { return v1.is_all_zero() || v2.is_all_zero(); },
              "Cannot compute angle between zero vectors");
    assert_if([&v1, &v2]() { return !v1.is_normalized() || !v2.is_normalized(); },
              "Vectors must be normalized to compute angle between them");
    if (v1.dot(v2) < 0) {
        return pi_v<T> - 2 * safe_asin((v1 + v2).length() / 2);
    } else {
        return 2 * safe_asin((v1 - v2).length() / 2);
    }
}

/**
 * @brief Builds an orthonormal basis {v2, v3} around a unit vector v1.
 *
 * Given a normalized direction v1, this function returns two
 * perpendicular unit vectors v2 and v3 that, together with v1,
 * form a right-handed coordinate system.
 */
template <std::floating_point T>
constexpr std::pair<Vector<T, 3>, Vector<T, 3>> coordinate_system(const Vector<T, 3>& v1) {
    assert_if([&v1]() { return !v1.is_normalized(); }, "Input vector to coordinate_system_stable() must be normalized");

    const T sign = std::copysign(T(1), v1.z());
    const T a = T(-1) / (sign + v1.z());
    const T b = v1.x() * v1.y() * a;

    Vector<T, 3> v2(T(1) + sign * v1.x() * v1.x() * a, sign * b, -sign * v1.x());
    Vector<T, 3> v3(b, sign + v1.y() * v1.y() * a, -v1.y());

    return {v2.normalized(), v3.normalized()};
}

/// @brief 1D vector using the project's default Float type.
using Vec1 = Vector<Float, 1>;
/// @brief 2D vector using the project's default Float type.
using Vec2 = Vector<Float, 2>;
/// @brief 3D vector using the project's default Float type.
using Vec3 = Vector<Float, 3>;
/// @brief 4D vector using the project's default Float type.
using Vec4 = Vector<Float, 4>;

/// @brief 1D vector using the project's default Int type.
using Vec1i = Vector<Int, 1>;
/// @brief 2D vector using the project's default Int type.
using Vec2i = Vector<Int, 2>;
/// @brief 3D vector using the project's default Int type.
using Vec3i = Vector<Int, 3>;
/// @brief 4D vector using the project's default Int type.
using Vec4i = Vector<Int, 4>;

// Lower-case aliases for migration from GLM-style naming.
using vec1 = Vec1;
using vec2 = Vec2;
using vec3 = Vec3;
using vec4 = Vec4;
using vec1i = Vec1i;
using vec2i = Vec2i;
using vec3i = Vec3i;
using vec4i = Vec4i;

}  // namespace pbpt::math
