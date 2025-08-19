#pragma once

#include <utility>
#include <concepts>


#include "math/homogeneous.hpp"
#include "math/type_alias.hpp"
#include "math/matrix.hpp"
#include "math/normal.hpp"
#include "math/point.hpp"
#include "math/vector.hpp"

#include "bounds.hpp"
#include "ray.hpp"

using namespace pbpt::math;

namespace pbpt::geometry {

template <std::floating_point T>
class Transform {
private:
    Matrix<T, 4, 4> m_mat;
    Matrix<T, 4, 4> m_inv;

public:
    constexpr Transform<T>()
        : m_mat(Matrix<T, 4, 4>::identity()), m_inv(Matrix<T, 4, 4>::identity()) {}

    constexpr Transform<T>(const Matrix<T, 4, 4>& m)
        : m_mat(m), m_inv(m.inversed()) {}

    constexpr Transform<T>(const Matrix<T, 4, 4>& m, const Matrix<T, 4, 4>& inv)
        : m_mat(m), m_inv(inv) {}

    static constexpr Transform<T> identity() {
        return Transform<T>(Matrix<T, 4, 4>::identity());
    }

    static constexpr Transform<T> translate(const Vector<T, 3>& t) {
        Matrix<T, 4, 4> m(
            T(1), T(0), T(0), t.x(),
            T(0), T(1), T(0), t.y(),
            T(0), T(0), T(1), t.z(),
            T(0), T(0), T(0), T(1)
        );
        Matrix<T, 4, 4> inv(
            T(1), T(0), T(0), -t.x(),
            T(0), T(1), T(0), -t.y(),
            T(0), T(0), T(1), -t.z(),
            T(0), T(0), T(0), T(1)
        );
        return Transform<T>(m, inv);
    }

    static constexpr Transform<T> scale(T s) {
        return scale(Vector<T, 3>{s, s, s});
    }

    static constexpr Transform<T> scale(const Vector<T, 3>& s) {
        Matrix<T, 4, 4> m(
            s.x(), T(0), T(0), T(0),
            T(0), s.y(), T(0), T(0),
            T(0), T(0), s.z(), T(0),
            T(0), T(0), T(0), T(1)
        );
        Matrix<T, 4, 4> inv(
            T(1)/s.x(), T(0), T(0), T(0),
            T(0), T(1)/s.y(), T(0), T(0),
            T(0), T(0), T(1)/s.z(), T(0),
            T(0), T(0), T(0), T(1)
        );
        return Transform<T>(m, inv);
    }

    static Transform<T> rotate_x(T angle_rad) {
        T s = std::sin(angle_rad), c = std::cos(angle_rad);
        Matrix<T, 4, 4> m(
            T(1), T(0), T(0), T(0),
            T(0), c,   -s,   T(0),
            T(0), s,    c,   T(0),
            T(0), T(0), T(0), T(1)
        );
        return Transform<T>(m, m.transposed());
    }

    static Transform<T> rotate_y(T angle_rad) {
        T s = std::sin(angle_rad), c = std::cos(angle_rad);
        Matrix<T, 4, 4> m(
             c, T(0), s, T(0),
             T(0), T(1), T(0), T(0),
            -s, T(0), c, T(0),
             T(0), T(0), T(0), T(1)
        );
        return Transform<T>(m, m.transposed());
    }

    static Transform<T> rotate_z(T angle_rad) {
        T s = std::sin(angle_rad), c = std::cos(angle_rad);
        Matrix<T, 4, 4> m(
            c, -s, T(0), T(0),
            s,  c, T(0), T(0),
            T(0), T(0), T(1), T(0),
            T(0), T(0), T(0), T(1)
        );
        return Transform<T>(m, m.transposed());
    }

    static Transform<T> rotate(T angle_rad, const Vector<T, 3>& axis) {
        Vector<T, 3> a = axis.normalized();
        T s = std::sin(angle_rad), c = std::cos(angle_rad), omc = T(1) - c;
        T ax = a.x(), ay = a.y(), az = a.z();

        Matrix<T, 4, 4> m(
            c + ax*ax*omc, ax*ay*omc - az*s, ax*az*omc + ay*s, T(0),
            ay*ax*omc + az*s, c + ay*ay*omc, ay*az*omc - ax*s, T(0),
            az*ax*omc - ay*s, az*ay*omc + ax*s, c + az*az*omc, T(0),
            T(0), T(0), T(0), T(1)
        );
        return Transform<T>(m, m.transposed());
    }

    static Transform<T> from_mat3x3(Matrix<T, 3, 3> m) {
        Matrix<T, 4, 4> mat(
            m.at(0, 0), m.at(0, 1), m.at(0, 2), T(0),
            m.at(1, 0), m.at(1, 1), m.at(1, 2), T(0),
            m.at(2, 0), m.at(2, 1), m.at(2, 2), T(0),
            T(0), T(0), T(0), T(1)
        );
        return Transform<T>(mat, mat.inversed());
    }

    static constexpr Transform<T> look_at(const Point<T, 3>& eye, const Point<T, 3>& target, const Vector<T, 3>& up) {
        Vector<T, 3> f = (target - eye).normalized();
        Vector<T, 3> s = cross(f, up).normalized();
        Vector<T, 3> u = cross(s, f);
        Matrix<T, 4, 4> m(
            s.x(), s.y(), s.z(), -s.dot(eye.to_vector()),
            u.x(), u.y(), u.z(), -u.dot(eye.to_vector()),
           -f.x(), -f.y(), -f.z(), f.dot(eye.to_vector()),
            T(0), T(0), T(0), T(1)
        );
        return Transform<T>(m, m.inversed());
    }

    static Transform<T> perspective(T fov_y_rad, T aspect, T z_near, T z_far) {
        T tan_half = std::tan(fov_y_rad / T(2));
        Matrix<T, 4, 4> m(
            T(1)/(aspect * tan_half), T(0), T(0), T(0),
            T(0), T(1)/tan_half, T(0), T(0),
            T(0), T(0), z_far / (z_far - z_near), -z_far * z_near / (z_far - z_near),
            T(0), T(0), T(1), T(0)
        );
        return Transform<T>(m);
    }

    static constexpr Transform<T> orthographic(T l, T r, T b, T t, T zn, T zf) {
        Matrix<T, 4, 4> m(
            T(2)/(r-l), T(0), T(0), -(r+l)/(r-l),
            T(0), T(2)/(t-b), T(0), -(t+b)/(t-b),
            T(0), T(0), T(1)/(zf-zn), -zn/(zf-zn),
            T(0), T(0), T(0), T(1)
        );
        return Transform<T>(m, m.inversed());
    }

    constexpr bool operator==(const Transform<T>& rhs) const {
        return m_mat == rhs.m_mat;
    }

    constexpr bool operator!=(const Transform<T>& rhs) const {
        return !(*this == rhs);
    }

    constexpr const Matrix<T, 4, 4>& matrix() const { return m_mat; }
    constexpr const Matrix<T, 4, 4>& inverse_matrix() const { return m_inv; }

    constexpr Transform<T>& set_matrix(const Matrix<T, 4, 4>& m) { 
        m_mat = m;
        m_inv = m.inversed();
        return *this;
    }

    constexpr Transform<T>& set_matrix(const Matrix<T, 4, 4>& m, const Matrix<T, 4, 4>& inv) { 
        m_mat = m;
        m_inv = inv;
        return *this;
    }

    constexpr bool is_identity() const { return m_mat.is_identity(); }

    constexpr bool has_scale() const {
        return !is_equal(((*this) * Vector<T, 3>(1,0,0)).length_squared(), T(1)) ||
               !is_equal(((*this) * Vector<T, 3>(0,1,0)).length_squared(), T(1)) ||
               !is_equal(((*this) * Vector<T, 3>(0,0,1)).length_squared(), T(1));
    }

    constexpr Transform<T>& inverse() {
        std::swap(m_mat, m_inv);
        return *this;
    }

    constexpr Transform<T> inversed() const {
        return Transform<T>(m_inv, m_mat);
    }

    constexpr Transform<T>& transpose() {
        m_mat.transpose();
        m_inv.transpose();
        return *this;
    }

    constexpr bool swaps_handedness() const {
        Matrix<T, 3, 3> upper3x3 = m_mat.template view<3, 3>(0, 0).to_matrix();
        return upper3x3.determinant() < T(0);
    }

    constexpr Transform<T> transposed() const {
        return Transform<T>(m_mat.transposed(), m_inv.transposed());
    }

    template<typename U>
    constexpr auto operator*(const Transform<U>& rhs) const {
        using R = std::common_type_t<T, U>;
        return Transform<R>(m_mat * rhs.matrix(), rhs.inverse_matrix() * m_inv);
    }

    template<typename U>
    constexpr auto operator*(const Point<U, 3>& p) const {
        using R = std::common_type_t<T, U>;
        return (m_mat * Homogeneous<R, 3>::from_point(p)).to_point();
    }

    template<typename U>
    constexpr auto operator*(const Vector<U, 3>& v) const {
        using R = std::common_type_t<T, U>;
        return (m_mat * Homogeneous<R, 3>::from_vector(v)).to_vector();
    }

    template<typename U>
    constexpr auto operator*(const Normal<U, 3>& n) const {
        using R = std::common_type_t<T, U>;
        auto r = m_inv.transposed() * Homogeneous<R, 3>::from_vector(n.to_vector());
        return Normal<R, 3>::from_vector(r.to_vector());
    }

    template<typename U>
    constexpr auto operator*(const Ray<U, 3>& ray) const {
        using R = std::common_type_t<T, U>;
        return Ray<R, 3>((*this) * ray.origin(), (*this) * ray.direction());
    }

    template<typename U>
    constexpr auto operator*(const Bounds<U, 3>& b) const {
        using R = std::common_type_t<T, U>;
        Bounds<R, 3> result;
        for (int i = 0; i < 8; i ++) {
            result.unite((*this) * b.corner(i));
        }
        return result;
    }
};

using Trans = Transform<Float>;

} // namespace pbpt::geometry

