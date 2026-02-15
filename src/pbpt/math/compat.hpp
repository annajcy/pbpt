/**
 * @file
 * @brief Convenience free functions and aliases for GLM-style migration.
 */
#pragma once

#include <algorithm>
#include <cmath>

#include "function.hpp"
#include "matrix.hpp"
#include "point.hpp"
#include "quaternion.hpp"
#include "vector.hpp"

namespace pbpt::math {

template <typename T>
constexpr T radians(T degree) {
    return deg2rad(degree);
}

template <typename T>
constexpr T degrees(T radian) {
    return rad2deg(radian);
}

template <typename T>
constexpr T clamp(T value, T min_value, T max_value) {
    return std::clamp(value, min_value, max_value);
}

template <typename T>
constexpr T max(T lhs, T rhs) {
    return std::max(lhs, rhs);
}

template <typename T>
constexpr T min(T lhs, T rhs) {
    return std::min(lhs, rhs);
}

template <typename T>
constexpr T asin(T value) {
    return std::asin(value);
}

template <typename T>
constexpr T atan2(T y, T x) {
    return std::atan2(y, x);
}

template <typename T, int N>
constexpr auto length(const Vector<T, N>& v) {
    return v.length();
}

template <typename T, int N>
constexpr auto normalize(const Vector<T, N>& v) {
    return v.normalized();
}

template <typename T, int N>
constexpr auto dot(const Vector<T, N>& lhs, const Vector<T, N>& rhs) {
    return lhs.dot(rhs);
}

template <typename T>
constexpr auto normalize(const Quaternion<T>& q) {
    return q.normalized();
}

template <typename T>
constexpr Matrix<T, 3, 3> mat3_cast(const Quaternion<T>& q) {
    return q.to_mat3();
}

template <typename T>
constexpr Matrix<T, 4, 4> mat4_cast(const Quaternion<T>& q) {
    return q.to_mat4();
}

template <typename T>
constexpr Quaternion<T> quat_cast(const Matrix<T, 3, 3>& m) {
    return Quaternion<T>::from_mat3(m);
}

template <typename T>
constexpr Quaternion<T> angleAxis(T angle_rad, const Vector<T, 3>& axis) {
    return Quaternion<T>::from_axis_angle(angle_rad, axis);
}

template <typename T>
constexpr Vector<T, 3> eulerAngles(const Quaternion<T>& q) {
    return q.to_euler_xyz();
}

template <typename T>
constexpr Quaternion<T> rotation(const Vector<T, 3>& from, const Vector<T, 3>& to) {
    return Quaternion<T>::rotation(from, to);
}

template <typename T>
constexpr Quaternion<T> rotate(const Quaternion<T>& q, T angle_rad, const Vector<T, 3>& axis) {
    return Quaternion<T>::from_axis_angle(angle_rad, axis) * q;
}

template <typename T, int R, int C>
constexpr Matrix<T, C, R> transpose(const Matrix<T, R, C>& m) {
    return m.transposed();
}

template <typename T, int N>
constexpr Matrix<T, N, N> inverse(const Matrix<T, N, N>& m) {
    return m.inversed();
}

template <typename T>
constexpr Matrix<T, 4, 4> translate(const Matrix<T, 4, 4>& m, const Vector<T, 3>& t) {
    Matrix<T, 4, 4> tr = Matrix<T, 4, 4>::identity();
    tr[0][3] = t.x();
    tr[1][3] = t.y();
    tr[2][3] = t.z();
    return m * tr;
}

template <typename T>
constexpr Matrix<T, 4, 4> scale(const Matrix<T, 4, 4>& m, const Vector<T, 3>& s) {
    Matrix<T, 4, 4> sm = Matrix<T, 4, 4>::identity();
    sm[0][0] = s.x();
    sm[1][1] = s.y();
    sm[2][2] = s.z();
    return m * sm;
}

template <typename T>
constexpr Matrix<T, 4, 4> lookAt(const Vector<T, 3>& eye, const Vector<T, 3>& target, const Vector<T, 3>& up) {
    const Vector<T, 3> f = (target - eye).normalized();
    const Vector<T, 3> s = cross(f, up).normalized();
    const Vector<T, 3> u = cross(s, f);

    return Matrix<T, 4, 4>(
        s.x(), s.y(), s.z(), -s.dot(eye),
        u.x(), u.y(), u.z(), -u.dot(eye),
        -f.x(), -f.y(), -f.z(), f.dot(eye),
        T(0), T(0), T(0), T(1)
    );
}

template <typename T>
constexpr Matrix<T, 4, 4> perspective(T fov_y_rad, T aspect_xy, T near, T far) {
    const T tan_half = std::tan(fov_y_rad / T(2));
    Matrix<T, 4, 4> out{};
    out[0][0] = T(1) / (aspect_xy * tan_half);
    out[1][1] = T(1) / tan_half;
    out[2][2] = -(far + near) / (far - near);
    out[2][3] = -(T(2) * far * near) / (far - near);
    out[3][2] = -T(1);
    return out;
}

template <typename T>
constexpr Matrix<T, 4, 4> ortho(T left, T right, T bottom, T top, T near, T far) {
    Matrix<T, 4, 4> out = Matrix<T, 4, 4>::identity();
    out[0][0] = T(2) / (right - left);
    out[1][1] = T(2) / (top - bottom);
    out[2][2] = -T(2) / (far - near);
    out[0][3] = -(right + left) / (right - left);
    out[1][3] = -(top + bottom) / (top - bottom);
    out[2][3] = -(far + near) / (far - near);
    return out;
}

template <typename T>
constexpr T identity() {
    return T::identity();
}

template <>
constexpr Quat identity<Quat>() {
    return Quat::identity();
}

}  // namespace pbpt::math
