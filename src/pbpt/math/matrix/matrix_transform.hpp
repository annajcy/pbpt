#pragma once

#include "pbpt/math/complex/quaternion.hpp"
#include "pbpt/math/matrix/matrix.hpp"
#include "pbpt/math/spatial/point.hpp"

namespace pbpt::math {

template <typename T>
constexpr Matrix<T, 4, 4> translate(const Vector<T, 3>& t) {
    return Matrix<T, 4, 4>(T(1), T(0), T(0), t.x(), T(0), T(1), T(0), t.y(), T(0), T(0), T(1), t.z(), T(0), T(0),
                           T(0), T(1));
}

template <typename T>
constexpr Matrix<T, 4, 4> scale(const Vector<T, 3>& s) {
    return Matrix<T, 4, 4>(s.x(), T(0), T(0), T(0), T(0), s.y(), T(0), T(0), T(0), T(0), s.z(), T(0), T(0), T(0),
                           T(0), T(1));
}

template <typename T>
constexpr Matrix<T, 4, 4> scale(T s) {
    return scale(Vector<T, 3>(s, s, s));
}

template <typename T>
constexpr Matrix<T, 4, 4> rotate_x(T angle_rad) {
    const T s = std::sin(angle_rad);
    const T c = std::cos(angle_rad);
    return Matrix<T, 4, 4>(T(1), T(0), T(0), T(0), T(0), c, -s, T(0), T(0), s, c, T(0), T(0), T(0), T(0), T(1));
}

template <typename T>
constexpr Matrix<T, 4, 4> rotate_y(T angle_rad) {
    const T s = std::sin(angle_rad);
    const T c = std::cos(angle_rad);
    return Matrix<T, 4, 4>(c, T(0), s, T(0), T(0), T(1), T(0), T(0), -s, T(0), c, T(0), T(0), T(0), T(0), T(1));
}

template <typename T>
constexpr Matrix<T, 4, 4> rotate_z(T angle_rad) {
    const T s = std::sin(angle_rad);
    const T c = std::cos(angle_rad);
    return Matrix<T, 4, 4>(c, -s, T(0), T(0), s, c, T(0), T(0), T(0), T(0), T(1), T(0), T(0), T(0), T(0), T(1));
}

template <typename T>
constexpr Matrix<T, 4, 4> rotate(T angle_rad, const Vector<T, 3>& axis) {
    const Vector<T, 3> a = axis.normalized();
    const T            s = std::sin(angle_rad);
    const T            c = std::cos(angle_rad);
    const T            omc = T(1) - c;
    const T            ax = a.x();
    const T            ay = a.y();
    const T            az = a.z();

    return Matrix<T, 4, 4>(c + ax * ax * omc, ax * ay * omc - az * s, ax * az * omc + ay * s, T(0),
                           ay * ax * omc + az * s, c + ay * ay * omc, ay * az * omc - ax * s, T(0),
                           az * ax * omc - ay * s, az * ay * omc + ax * s, c + az * az * omc, T(0), T(0), T(0),
                           T(0), T(1));
}

template <typename T>
constexpr Matrix<T, 4, 4> rotate(const Quaternion<T>& q) {
    return q.to_mat4();
}

template <typename T>
constexpr Matrix<T, 4, 4> compose_trs(const Vector<T, 3>& translation, const Quaternion<T>& rotation,
                                      const Vector<T, 3>& scaling) {
    return translate(translation) * rotate(rotation) * scale(scaling);
}

template <typename T>
constexpr Matrix<T, 4, 4> from_mat3x3(const Matrix<T, 3, 3>& m) {
    return Matrix<T, 4, 4>(m.at(0, 0), m.at(0, 1), m.at(0, 2), T(0), m.at(1, 0), m.at(1, 1), m.at(1, 2), T(0),
                           m.at(2, 0), m.at(2, 1), m.at(2, 2), T(0), T(0), T(0), T(0), T(1));
}

template <typename T>
constexpr Vector<T, 3> extract_translation(const Matrix<T, 4, 4>& m) {
    return Vector<T, 3>(m[0][3], m[1][3], m[2][3]);
}

template <typename T>
constexpr Vector<T, 3> extract_scale(const Matrix<T, 4, 4>& m) {
    return Vector<T, 3>(length(Vector<T, 3>(m[0][0], m[1][0], m[2][0])), length(Vector<T, 3>(m[0][1], m[1][1], m[2][1])),
                        length(Vector<T, 3>(m[0][2], m[1][2], m[2][2])));
}

template <typename T>
constexpr Quaternion<T> extract_rotation(const Matrix<T, 4, 4>& m) {
    const Vector<T, 3> scale_factors = extract_scale(m);
    Matrix<T, 3, 3>    rotation_matrix(m);
    for (int r = 0; r < 3; ++r) {
        rotation_matrix[r][0] /= scale_factors.x();
        rotation_matrix[r][1] /= scale_factors.y();
        rotation_matrix[r][2] /= scale_factors.z();
    }
    return Quaternion<T>::from_mat3(rotation_matrix);
}

template <typename T>
constexpr Matrix<T, 4, 4> look_at(const Point<T, 3>& eye, const Point<T, 3>& target, const Vector<T, 3>& up) {
    const Vector<T, 3> f = (target - eye).normalized();
    const Vector<T, 3> s = cross(f, up).normalized();
    const Vector<T, 3> u = cross(s, f);
    const Vector<T, 3> eye_v = eye.to_vector();

    return Matrix<T, 4, 4>(s.x(), s.y(), s.z(), -s.dot(eye_v), u.x(), u.y(), u.z(), -u.dot(eye_v), -f.x(), -f.y(),
                           -f.z(), f.dot(eye_v), T(0), T(0), T(0), T(1));
}

template <typename T>
constexpr Matrix<T, 4, 4> look_at(const Vector<T, 3>& eye, const Vector<T, 3>& target, const Vector<T, 3>& up) {
    return look_at(Point<T, 3>(eye.x(), eye.y(), eye.z()), Point<T, 3>(target.x(), target.y(), target.z()), up);
}

template <typename T>
constexpr Matrix<T, 4, 4> orthographic(T left, T right, T bottom, T top, T near, T far) {
    return Matrix<T, 4, 4>(T(2) / (right - left), T(0), T(0), -(right + left) / (right - left), T(0),
                           T(2) / (top - bottom), T(0), -(top + bottom) / (top - bottom), T(0), T(0),
                           T(1) / (far - near), -near / (far - near), T(0), T(0), T(0), T(1));
}

template <typename T>
constexpr Matrix<T, 4, 4> perspective_to_orthographic(T near, T far) {
    return Matrix<T, 4, 4>(-near, T(0), T(0), T(0), T(0), -near, T(0), T(0), T(0), T(0), -(near + far), near * far,
                           T(0), T(0), -T(1), T(0));
}

template <typename T>
constexpr Matrix<T, 4, 4> perspective(T fov_y_rad, T aspect_xy, T near, T far) {
    const Matrix<T, 4, 4> persp_to_ortho = perspective_to_orthographic(near, far);
    const T               near_distance = std::abs(near);
    const T               right = near_distance * std::tan(fov_y_rad / T(2)) * aspect_xy;
    const T               left = -right;
    const T               top = near_distance * std::tan(fov_y_rad / T(2));
    const T               bottom = -top;
    return orthographic(left, right, bottom, top, near, far) * persp_to_ortho;
}

template <typename T>
constexpr Matrix<T, 4, 4> viewport(T width, T height) {
    return Matrix<T, 4, 4>(width / T(2), T(0), T(0), width / T(2), T(0), height / T(2), T(0), height / T(2), T(0),
                           T(0), T(1), T(0), T(0), T(0), T(0), T(1));
}

}  // namespace pbpt::math
