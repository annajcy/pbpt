#pragma once

#include <cmath>
#include <iostream>

#include "vector.hpp"

namespace pbpt::math {

template <typename T = Float>
    requires std::is_floating_point_v<T>
class Quaternion {
private:
    T m_w, m_x, m_y, m_z;  // w + xi + yj + zk
public:
    // ========== Constructors ==========

    /// Default constructor - creates identity quaternion
    constexpr Quaternion() noexcept : m_w(1), m_x(0), m_y(0), m_z(0) {}

    /// Construct from components (w, x, y, z)
    constexpr Quaternion(T w, T x, T y, T z) noexcept : m_w(w), m_x(x), m_y(y), m_z(z) {}

    /// Construct from scalar and vector parts
    constexpr Quaternion(T scalar, const Vector<T, 3>& vector) noexcept
        : m_w(scalar), m_x(vector.x()), m_y(vector.y()), m_z(vector.z()) {}

    /// Construct from axis-angle representation
    constexpr Quaternion(const Vector<T, 3>& axis, T angle) noexcept {
        T            half_angle      = angle * T(0.5);
        T            sin_half        = std::sin(half_angle);
        Vector<T, 3> normalized_axis = axis.normalized();

        m_w = std::cos(half_angle);
        m_x = normalized_axis.x() * sin_half;
        m_y = normalized_axis.y() * sin_half;
        m_z = normalized_axis.z() * sin_half;
    }

    /// Construct rotation from one vector to another
    constexpr Quaternion(const Vector<T, 3>& from, const Vector<T, 3>& to) noexcept {
        Vector<T, 3> from_norm = from.normalized();
        Vector<T, 3> to_norm   = to.normalized();

        T dot_product = from_norm.dot(to_norm);

        if (dot_product >= T(1) - epsilon_v<T>) {
            // Vectors are the same
            *this = identity();
            return;
        }

        if (dot_product <= T(-1) + epsilon_v<T>) {
            // Vectors are opposite - find any perpendicular axis
            Vector<T, 3> axis = cross(from_norm, Vector<T, 3>(1, 0, 0));
            if (axis.length_squared() < epsilon_v<T>) {
                axis = cross(from_norm, Vector<T, 3>(0, 1, 0));
            }
            *this = Quaternion(axis.normalized(), T(M_PI));
            return;
        }

        Vector<T, 3> cross_product = cross(from_norm, to_norm);
        m_w                        = T(1) + dot_product;
        m_x                        = cross_product.x();
        m_y                        = cross_product.y();
        m_z                        = cross_product.z();

        *this = normalized();
    }

    // ========== Static Factory Methods ==========

    /// Create identity quaternion
    static constexpr Quaternion identity() noexcept { return Quaternion(T(1), T(0), T(0), T(0)); }

    /// Create quaternion from Euler angles (roll, pitch, yaw) in radians
    static constexpr Quaternion from_euler(T roll, T pitch, T yaw) noexcept {
        T cr = std::cos(roll * T(0.5));
        T sr = std::sin(roll * T(0.5));
        T cp = std::cos(pitch * T(0.5));
        T sp = std::sin(pitch * T(0.5));
        T cy = std::cos(yaw * T(0.5));
        T sy = std::sin(yaw * T(0.5));

        return Quaternion(cr * cp * cy + sr * sp * sy,  // w
                          sr * cp * cy - cr * sp * sy,  // x
                          cr * sp * cy + sr * cp * sy,  // y
                          cr * cp * sy - sr * sp * cy   // z
        );
    }

    /// Create quaternion from rotation matrix (3x3)
    template <typename MatrixType>
    static constexpr Quaternion from_matrix(const MatrixType& m) noexcept {
        T trace = m(0, 0) + m(1, 1) + m(2, 2);

        if (trace > 0) {
            T s = std::sqrt(trace + T(1)) * T(2);       // s = 4 * qw
            return Quaternion(T(0.25) * s,              // w
                              (m(2, 1) - m(1, 2)) / s,  // x
                              (m(0, 2) - m(2, 0)) / s,  // y
                              (m(1, 0) - m(0, 1)) / s   // z
            );
        } else if (m(0, 0) > m(1, 1) && m(0, 0) > m(2, 2)) {
            T s = std::sqrt(T(1) + m(0, 0) - m(1, 1) - m(2, 2)) * T(2);  // s = 4 * qx
            return Quaternion((m(2, 1) - m(1, 2)) / s,                   // w
                              T(0.25) * s,                               // x
                              (m(0, 1) + m(1, 0)) / s,                   // y
                              (m(0, 2) + m(2, 0)) / s                    // z
            );
        } else if (m(1, 1) > m(2, 2)) {
            T s = std::sqrt(T(1) + m(1, 1) - m(0, 0) - m(2, 2)) * T(2);  // s = 4 * qy
            return Quaternion((m(0, 2) - m(2, 0)) / s,                   // w
                              (m(0, 1) + m(1, 0)) / s,                   // x
                              T(0.25) * s,                               // y
                              (m(1, 2) + m(2, 1)) / s                    // z
            );
        } else {
            T s = std::sqrt(T(1) + m(2, 2) - m(0, 0) - m(1, 1)) * T(2);  // s = 4 * qz
            return Quaternion((m(1, 0) - m(0, 1)) / s,                   // w
                              (m(0, 2) + m(2, 0)) / s,                   // x
                              (m(1, 2) + m(2, 1)) / s,                   // y
                              T(0.25) * s                                // z
            );
        }
    }

    // ========== Accessors ==========

    constexpr T w() const noexcept { return m_w; }
    constexpr T x() const noexcept { return m_x; }
    constexpr T y() const noexcept { return m_y; }
    constexpr T z() const noexcept { return m_z; }

    constexpr T& w() noexcept { return m_w; }
    constexpr T& x() noexcept { return m_x; }
    constexpr T& y() noexcept { return m_y; }
    constexpr T& z() noexcept { return m_z; }

    /// Get scalar part
    constexpr T scalar() const noexcept { return m_w; }

    /// Get vector part
    constexpr Vector<T, 3> vector() const noexcept { return Vector<T, 3>(m_x, m_y, m_z); }

    /// Array-style access [0]=w, [1]=x, [2]=y, [3]=z
    constexpr const T& operator[](int index) const {
        switch (index) {
            case 0:
                return m_w;
            case 1:
                return m_x;
            case 2:
                return m_y;
            case 3:
                return m_z;
            default:
                if (std::is_constant_evaluated()) {
                    throw "Compile-time error: Quaternion index out of range";
                } else {
                    throw std::out_of_range("Quaternion index out of range");
                }
        }
    }

    constexpr T& operator[](int index) {
        switch (index) {
            case 0:
                return m_w;
            case 1:
                return m_x;
            case 2:
                return m_y;
            case 3:
                return m_z;
            default:
                if (std::is_constant_evaluated()) {
                    throw "Compile-time error: Quaternion index out of range";
                } else {
                    throw std::out_of_range("Quaternion index out of range");
                }
        }
    }

    // ========== Properties ==========

    /// Calculate squared magnitude
    constexpr T length_squared() const noexcept { return m_w * m_w + m_x * m_x + m_y * m_y + m_z * m_z; }

    /// Calculate magnitude
    constexpr T length() const noexcept { return std::sqrt(length_squared()); }

    /// Check if quaternion is normalized
    constexpr bool is_normalized(T epsilon = epsilon_v<T>) const noexcept {
        return abs(length_squared() - T(1)) < epsilon;
    }

    /// Check if quaternion is identity
    constexpr bool is_identity(T epsilon = epsilon_v<T>) const noexcept {
        return abs(m_w - T(1)) < epsilon && abs(m_x) < epsilon && abs(m_y) < epsilon && abs(m_z) < epsilon;
    }

    // ========== Operations ==========

    /// Normalize quaternion
    constexpr Quaternion normalized() const noexcept {
        T len = length();
        if (len < epsilon_v<T>) {
            return identity();
        }
        T inv_len = T(1) / len;
        return Quaternion(m_w * inv_len, m_x * inv_len, m_y * inv_len, m_z * inv_len);
    }

    /// Normalize quaternion in-place
    constexpr Quaternion& normalize() noexcept {
        *this = normalized();
        return *this;
    }

    /// Conjugate quaternion (negate vector part)
    constexpr Quaternion conjugate() const noexcept { return Quaternion(m_w, -m_x, -m_y, -m_z); }

    /// Inverse quaternion
    constexpr Quaternion inverse() const noexcept {
        T len_sq = length_squared();
        if (len_sq < epsilon_v<T>) {
            return identity();
        }
        T inv_len_sq = T(1) / len_sq;
        return Quaternion(m_w * inv_len_sq, -m_x * inv_len_sq, -m_y * inv_len_sq, -m_z * inv_len_sq);
    }

    /// Dot product with another quaternion
    constexpr T dot(const Quaternion& other) const noexcept {
        return m_w * other.m_w + m_x * other.m_x + m_y * other.m_y + m_z * other.m_z;
    }

    // ========== Rotation Operations ==========

    /// Rotate a vector by this quaternion
    constexpr Vector<T, 3> rotate(const Vector<T, 3>& v) const noexcept {
        // Optimized version: v' = v + 2 * cross(q.xyz, cross(q.xyz, v) + q.w *
        // v)
        Vector<T, 3> qvec(m_x, m_y, m_z);
        Vector<T, 3> cross1 = cross(qvec, v);
        Vector<T, 3> cross2 = cross(qvec, cross1 + m_w * v);
        return v + T(2) * cross2;
    }

    /// Get rotation axis and angle
    constexpr std::pair<Vector<T, 3>, T> to_axis_angle() const noexcept {
        Quaternion q = normalized();

        // Handle identity quaternion
        if (abs(q.m_w) >= T(1) - epsilon_v<T>) {
            return {Vector<T, 3>(0, 0, 1), T(0)};
        }

        T angle          = T(2) * std::acos(abs(q.m_w));
        T sin_half_angle = std::sqrt(T(1) - q.m_w * q.m_w);

        Vector<T, 3> axis;
        if (sin_half_angle < epsilon_v<T>) {
            axis = Vector<T, 3>(1, 0, 0);
        } else {
            T inv_sin = T(1) / sin_half_angle;
            axis      = Vector<T, 3>(q.m_x * inv_sin, q.m_y * inv_sin, q.m_z * inv_sin);
        }

        return {axis, angle};
    }

    /// Convert to Euler angles (roll, pitch, yaw) in radians
    constexpr Vector<T, 3> to_euler() const noexcept {
        Quaternion q = normalized();

        // Roll (x-axis rotation)
        T sinr_cosp = T(2) * (q.m_w * q.m_x + q.m_y * q.m_z);
        T cosr_cosp = T(1) - T(2) * (q.m_x * q.m_x + q.m_y * q.m_y);
        T roll      = std::atan2(sinr_cosp, cosr_cosp);

        // Pitch (y-axis rotation)
        T sinp = T(2) * (q.m_w * q.m_y - q.m_z * q.m_x);
        T pitch;
        if (abs(sinp) >= T(1)) {
            pitch = std::copysign(T(M_PI) / T(2),
                                  sinp);  // Use 90 degrees if out of range
        } else {
            pitch = std::asin(sinp);
        }

        // Yaw (z-axis rotation)
        T siny_cosp = T(2) * (q.m_w * q.m_z + q.m_x * q.m_y);
        T cosy_cosp = T(1) - T(2) * (q.m_y * q.m_y + q.m_z * q.m_z);
        T yaw       = std::atan2(siny_cosp, cosy_cosp);

        return Vector<T, 3>(roll, pitch, yaw);
    }

    // ========== Arithmetic Operators ==========

    constexpr Quaternion operator+(const Quaternion& rhs) const noexcept {
        return Quaternion(m_w + rhs.m_w, m_x + rhs.m_x, m_y + rhs.m_y, m_z + rhs.m_z);
    }

    constexpr Quaternion operator-(const Quaternion& rhs) const noexcept {
        return Quaternion(m_w - rhs.m_w, m_x - rhs.m_x, m_y - rhs.m_y, m_z - rhs.m_z);
    }

    constexpr Quaternion operator-() const noexcept { return Quaternion(-m_w, -m_x, -m_y, -m_z); }

    /// Quaternion multiplication (composition of rotations)
    constexpr Quaternion operator*(const Quaternion& rhs) const noexcept {
        return Quaternion(m_w * rhs.m_w - m_x * rhs.m_x - m_y * rhs.m_y - m_z * rhs.m_z,  // w
                          m_w * rhs.m_x + m_x * rhs.m_w + m_y * rhs.m_z - m_z * rhs.m_y,  // x
                          m_w * rhs.m_y - m_x * rhs.m_z + m_y * rhs.m_w + m_z * rhs.m_x,  // y
                          m_w * rhs.m_z + m_x * rhs.m_y - m_y * rhs.m_x + m_z * rhs.m_w   // z
        );
    }

    /// Scalar multiplication
    constexpr Quaternion operator*(T scalar) const noexcept {
        return Quaternion(m_w * scalar, m_x * scalar, m_y * scalar, m_z * scalar);
    }

    friend constexpr Quaternion operator*(T scalar, const Quaternion& q) noexcept { return q * scalar; }

    /// Scalar division
    constexpr Quaternion operator/(T scalar) const {
        if (abs(scalar) < epsilon_v<T>) {
            if (std::is_constant_evaluated()) {
                throw "Compile-time error: Division by zero";
            } else {
                throw std::runtime_error("Division by zero");
            }
        }
        T inv_scalar = T(1) / scalar;
        return Quaternion(m_w * inv_scalar, m_x * inv_scalar, m_y * inv_scalar, m_z * inv_scalar);
    }

    // ========== Assignment Operators ==========

    constexpr Quaternion& operator+=(const Quaternion& rhs) noexcept {
        m_w += rhs.m_w;
        m_x += rhs.m_x;
        m_y += rhs.m_y;
        m_z += rhs.m_z;
        return *this;
    }

    constexpr Quaternion& operator-=(const Quaternion& rhs) noexcept {
        m_w -= rhs.m_w;
        m_x -= rhs.m_x;
        m_y -= rhs.m_y;
        m_z -= rhs.m_z;
        return *this;
    }

    constexpr Quaternion& operator*=(const Quaternion& rhs) noexcept {
        *this = *this * rhs;
        return *this;
    }

    constexpr Quaternion& operator*=(T scalar) noexcept {
        m_w *= scalar;
        m_x *= scalar;
        m_y *= scalar;
        m_z *= scalar;
        return *this;
    }

    constexpr Quaternion& operator/=(T scalar) {
        *this = *this / scalar;
        return *this;
    }

    // ========== Comparison Operators ==========

    constexpr bool operator==(const Quaternion& rhs) const noexcept {
        return is_equal(m_w, rhs.m_w) && is_equal(m_x, rhs.m_x) && is_equal(m_y, rhs.m_y) && is_equal(m_z, rhs.m_z);
    }

    constexpr bool operator!=(const Quaternion& rhs) const noexcept { return !(*this == rhs); }

    // ========== Interpolation ==========

    /// Linear interpolation (not recommended for rotations)
    static constexpr Quaternion lerp(const Quaternion& a, const Quaternion& b, T t) noexcept {
        return ((T(1) - t) * a + t * b).normalized();
    }

    /// Spherical linear interpolation (recommended for smooth rotation
    /// interpolation)
    static constexpr Quaternion slerp(const Quaternion& a, const Quaternion& b, T t) noexcept {
        Quaternion qa = a.normalized();
        Quaternion qb = b.normalized();

        T dot = qa.dot(qb);

        // If dot product is negative, slerp won't take the shorter path.
        // Note that v1 and -v1 are equivalent when the represent rotations.
        if (dot < T(0)) {
            qb  = -qb;
            dot = -dot;
        }

        // If the inputs are too close for comfort, linearly interpolate
        if (dot > T(0.9995)) {
            return lerp(qa, qb, t);
        }

        T theta_0     = std::acos(abs(dot));
        T theta       = theta_0 * t;
        T sin_theta   = std::sin(theta);
        T sin_theta_0 = std::sin(theta_0);

        T s0 = std::cos(theta) - dot * sin_theta / sin_theta_0;
        T s1 = sin_theta / sin_theta_0;

        return s0 * qa + s1 * qb;
    }

    // ========== Stream Output ==========

    friend std::ostream& operator<<(std::ostream& os, const Quaternion& q) {
        os << "Quat(" << q.m_w << ", " << q.m_x << ", " << q.m_y << ", " << q.m_z << ")";
        return os;
    }
};

// ========== Type Aliases ==========

using Quat  = Quaternion<Float>;
using Quatf = Quaternion<float>;
using Quatd = Quaternion<double>;

}  // namespace pbpt::math