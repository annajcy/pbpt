/**
 * @file
 * @brief Quaternion math utilities for 3D rotations.
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <type_traits>

#include "function.hpp"
#include "matrix.hpp"
#include "operator.hpp"
#include "vector.hpp"

namespace pbpt::math {

template <typename T>
class Quaternion {
private:
    T m_w{T(1)};
    T m_x{T(0)};
    T m_y{T(0)};
    T m_z{T(0)};

public:
    constexpr Quaternion() = default;
    constexpr Quaternion(T w, T x, T y, T z) : m_w(w), m_x(x), m_y(y), m_z(z) {}

    template <typename U>
        requires std::is_convertible_v<U, T>
    constexpr explicit Quaternion(const Vector<U, 3>& euler_xyz_rad) {
        *this = from_euler_xyz(Vector<T, 3>(static_cast<T>(euler_xyz_rad.x()), static_cast<T>(euler_xyz_rad.y()),
                                            static_cast<T>(euler_xyz_rad.z())));
    }

    static constexpr Quaternion identity() { return Quaternion(T(1), T(0), T(0), T(0)); }

    constexpr T w() const { return m_w; }
    constexpr T x() const { return m_x; }
    constexpr T y() const { return m_y; }
    constexpr T z() const { return m_z; }

    constexpr T& w() { return m_w; }
    constexpr T& x() { return m_x; }
    constexpr T& y() { return m_y; }
    constexpr T& z() { return m_z; }

    constexpr T length_squared() const { return m_w * m_w + m_x * m_x + m_y * m_y + m_z * m_z; }
    constexpr T length() const { return std::sqrt(length_squared()); }

    constexpr Quaternion normalized() const {
        const T len = length();
        assert_if([&]() { return is_equal(len, T(0)); }, "Cannot normalize zero quaternion");
        return Quaternion(m_w / len, m_x / len, m_y / len, m_z / len);
    }

    constexpr Quaternion conjugated() const { return Quaternion(m_w, -m_x, -m_y, -m_z); }

    constexpr Quaternion inversed() const {
        const T ls = length_squared();
        assert_if([&]() { return is_equal(ls, T(0)); }, "Cannot invert zero quaternion");
        return conjugated() * (T(1) / ls);
    }

    constexpr Quaternion operator*(const Quaternion& rhs) const {
        return Quaternion(m_w * rhs.m_w - m_x * rhs.m_x - m_y * rhs.m_y - m_z * rhs.m_z,
                          m_w * rhs.m_x + m_x * rhs.m_w + m_y * rhs.m_z - m_z * rhs.m_y,
                          m_w * rhs.m_y - m_x * rhs.m_z + m_y * rhs.m_w + m_z * rhs.m_x,
                          m_w * rhs.m_z + m_x * rhs.m_y - m_y * rhs.m_x + m_z * rhs.m_w);
    }

    constexpr Quaternion operator*(T rhs) const { return Quaternion(m_w * rhs, m_x * rhs, m_y * rhs, m_z * rhs); }

    constexpr Quaternion operator/(T rhs) const {
        assert_if([&]() { return is_equal(rhs, T(0)); }, "Quaternion division by zero");
        return Quaternion(m_w / rhs, m_x / rhs, m_y / rhs, m_z / rhs);
    }

    constexpr Quaternion& operator*=(const Quaternion& rhs) {
        *this = (*this) * rhs;
        return *this;
    }

    constexpr bool operator==(const Quaternion& rhs) const {
        return is_equal(m_w, rhs.m_w) && is_equal(m_x, rhs.m_x) && is_equal(m_y, rhs.m_y) && is_equal(m_z, rhs.m_z);
    }

    constexpr bool operator!=(const Quaternion& rhs) const { return !(*this == rhs); }

    constexpr Vector<T, 3> rotate_vector(const Vector<T, 3>& v) const {
        const Quaternion qv(T(0), v.x(), v.y(), v.z());
        const Quaternion qr = (*this) * qv * this->inversed();
        return Vector<T, 3>(qr.x(), qr.y(), qr.z());
    }

    constexpr Vector<T, 3> operator*(const Vector<T, 3>& v) const { return rotate_vector(v); }

    constexpr Matrix<T, 3, 3> to_mat3() const {
        const Quaternion q = normalized();
        const T xx = q.x() * q.x();
        const T yy = q.y() * q.y();
        const T zz = q.z() * q.z();
        const T xy = q.x() * q.y();
        const T xz = q.x() * q.z();
        const T yz = q.y() * q.z();
        const T wx = q.w() * q.x();
        const T wy = q.w() * q.y();
        const T wz = q.w() * q.z();

        return Matrix<T, 3, 3>(T(1) - T(2) * (yy + zz), T(2) * (xy - wz), T(2) * (xz + wy), T(2) * (xy + wz),
                               T(1) - T(2) * (xx + zz), T(2) * (yz - wx), T(2) * (xz - wy), T(2) * (yz + wx),
                               T(1) - T(2) * (xx + yy));
    }

    constexpr Matrix<T, 4, 4> to_mat4() const {
        Matrix<T, 4, 4> out = Matrix<T, 4, 4>::identity();
        const Matrix<T, 3, 3> m3 = to_mat3();
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                out[r][c] = m3[r][c];
            }
        }
        return out;
    }

    static Quaternion from_axis_angle(T angle_rad, const Vector<T, 3>& axis) {
        const auto axis_len = axis.length();
        assert_if([&]() { return is_equal(axis_len, T(0)); }, "Axis must be non-zero");
        const Vector<T, 3> n = axis.normalized();
        const T half = angle_rad * T(0.5);
        const T s = std::sin(half);
        return Quaternion(std::cos(half), n.x() * s, n.y() * s, n.z() * s).normalized();
    }

    static Quaternion from_mat3(const Matrix<T, 3, 3>& m) {
        const T trace = m[0][0] + m[1][1] + m[2][2];
        Quaternion q;
        if (trace > T(0)) {
            const T s = std::sqrt(trace + T(1)) * T(2);
            q.w() = T(0.25) * s;
            q.x() = (m[2][1] - m[1][2]) / s;
            q.y() = (m[0][2] - m[2][0]) / s;
            q.z() = (m[1][0] - m[0][1]) / s;
        } else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) {
            const T s = std::sqrt(T(1) + m[0][0] - m[1][1] - m[2][2]) * T(2);
            q.w() = (m[2][1] - m[1][2]) / s;
            q.x() = T(0.25) * s;
            q.y() = (m[0][1] + m[1][0]) / s;
            q.z() = (m[0][2] + m[2][0]) / s;
        } else if (m[1][1] > m[2][2]) {
            const T s = std::sqrt(T(1) + m[1][1] - m[0][0] - m[2][2]) * T(2);
            q.w() = (m[0][2] - m[2][0]) / s;
            q.x() = (m[0][1] + m[1][0]) / s;
            q.y() = T(0.25) * s;
            q.z() = (m[1][2] + m[2][1]) / s;
        } else {
            const T s = std::sqrt(T(1) + m[2][2] - m[0][0] - m[1][1]) * T(2);
            q.w() = (m[1][0] - m[0][1]) / s;
            q.x() = (m[0][2] + m[2][0]) / s;
            q.y() = (m[1][2] + m[2][1]) / s;
            q.z() = T(0.25) * s;
        }
        return q.normalized();
    }

    static Quaternion from_euler_xyz(const Vector<T, 3>& euler_xyz_rad) {
        const Quaternion qx = from_axis_angle(euler_xyz_rad.x(), Vector<T, 3>(T(1), T(0), T(0)));
        const Quaternion qy = from_axis_angle(euler_xyz_rad.y(), Vector<T, 3>(T(0), T(1), T(0)));
        const Quaternion qz = from_axis_angle(euler_xyz_rad.z(), Vector<T, 3>(T(0), T(0), T(1)));
        return (qz * qy * qx).normalized();
    }

    constexpr Vector<T, 3> to_euler_xyz() const {
        const Quaternion q = normalized();

        const T sinr_cosp = T(2) * (q.w() * q.x() + q.y() * q.z());
        const T cosr_cosp = T(1) - T(2) * (q.x() * q.x() + q.y() * q.y());
        const T roll_x = std::atan2(sinr_cosp, cosr_cosp);

        const T sinp = T(2) * (q.w() * q.y() - q.z() * q.x());
        const T pitch_y = (std::abs(sinp) >= T(1)) ? std::copysign(pi_v<T> / T(2), sinp) : std::asin(sinp);

        const T siny_cosp = T(2) * (q.w() * q.z() + q.x() * q.y());
        const T cosy_cosp = T(1) - T(2) * (q.y() * q.y() + q.z() * q.z());
        const T yaw_z = std::atan2(siny_cosp, cosy_cosp);

        return Vector<T, 3>(roll_x, pitch_y, yaw_z);
    }

    static Quaternion rotation(const Vector<T, 3>& from, const Vector<T, 3>& to) {
        const Vector<T, 3> f = from.normalized();
        const Vector<T, 3> t = to.normalized();
        const T cos_theta = f.dot(t);

        if (cos_theta > T(1) - epsilon_v<T>) {
            return identity();
        }

        if (cos_theta < T(-1) + epsilon_v<T>) {
            Vector<T, 3> axis = cross(Vector<T, 3>(T(1), T(0), T(0)), f);
            if (axis.length() < epsilon_v<T>) {
                axis = cross(Vector<T, 3>(T(0), T(1), T(0)), f);
            }
            return from_axis_angle(pi_v<T>, axis.normalized());
        }

        const Vector<T, 3> axis = cross(f, t);
        const T s = std::sqrt((T(1) + cos_theta) * T(2));
        const T inv_s = T(1) / s;
        return Quaternion(s * T(0.5), axis.x() * inv_s, axis.y() * inv_s, axis.z() * inv_s).normalized();
    }
};

template <typename T>
constexpr Quaternion<T> operator*(T lhs, const Quaternion<T>& rhs) {
    return rhs * lhs;
}

using Quat = Quaternion<Float>;
using quat = Quat;

}  // namespace pbpt::math
