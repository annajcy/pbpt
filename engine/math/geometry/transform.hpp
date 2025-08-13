#pragma once

#include "bounds.hpp"
#include "homogeneous.hpp"
#include "math/global/type_alias.hpp"
#include "matrix.hpp"
#include "normal.hpp"
#include "point.hpp"
#include "ray.hpp"
#include "vector.hpp"

namespace pbpt::math {

template <std::floating_point T>
class Transform {
public:

    static constexpr Transform<T> translate(const Vector<T, 3>& t) noexcept {
        return Transform(Matrix<T, 4, 4>(T(1), T(0), T(0), t.x(), T(0), T(1), T(0), t.y(), T(0), T(0), T(1), t.z(), T(0), T(0), T(0), T(1)));
    }

    static constexpr Transform<T> scale(T s) noexcept {
        return Transform(Matrix<T, 4, 4>(s, T(0), T(0), T(0), T(0), s, T(0), T(0), T(0), T(0), s, T(0), T(0), T(0), T(0), T(1)));
    }

    static constexpr Transform<T> scale(const Vector<T, 3>& s) noexcept {
        return Transform(Matrix<T, 4, 4>(s.x(), T(0), T(0), T(0), T(0), s.y(), T(0), T(0), T(0), T(0), s.z(), T(0), T(0), T(0), T(0), T(1)));
    }

    static Transform<T> rotate_x(T angle_rad) noexcept {
        const T s = std::sin(angle_rad);
        const T c = std::cos(angle_rad);
        return Transform(Matrix<T, 4, 4>(T(1), T(0), T(0), T(0), T(0), c, -s, T(0), T(0), s, c, T(0), T(0), T(0), T(0), T(1)));
    }

    static Transform<T> rotate_y(T angle_rad) noexcept {
        const T s = std::sin(angle_rad);
        const T c = std::cos(angle_rad);
        return Transform(Matrix<T, 4, 4>(c, T(0), s, T(0), T(0), T(1), T(0), T(0), -s, T(0), c, T(0), T(0), T(0), T(0), T(1)));
    }

    static Transform<T> rotate_z(T angle_rad) noexcept {
        const T s = std::sin(angle_rad);
        const T c = std::cos(angle_rad);
        return Transform(Matrix<T, 4, 4>(c, -s, T(0), T(0), s, c, T(0), T(0), T(0), T(0), T(1), T(0), T(0), T(0), T(0), T(1)));
    }

    static Transform<T> rotate(T angle_rad, const Vector<T, 3>& axis) noexcept {
        // 确保旋转轴是单位向量，这对于公式的正确性至关重要
        const Vector<T, 3> a = axis.normalized();
        const T s = std::sin(angle_rad);
        const T c = std::cos(angle_rad);
        const T omc = T(1) - c;  // one-minus-cosine

        const T ax = a.x();
        const T ay = a.y();
        const T az = a.z();

        // 罗德里格斯旋转公式的矩阵形式
        return Transform(Matrix<T, 4, 4>(c + ax * ax * omc, ax * ay * omc - az * s, ax * az * omc + ay * s, T(0),
                              ay * ax * omc + az * s, c + ay * ay * omc, ay * az * omc - ax * s, T(0),
                              az * ax * omc - ay * s, az * ay * omc + ax * s, c + az * az * omc, T(0), T(0), T(0), T(0), T(1)));
    }

    static constexpr Transform<T> look_at(const Point<T, 3>& eye, const Point<T, 3>& target, const Vector<T, 3>& up) noexcept {
        const Vector<T, 3> f = (target - eye).normalized();  // Forward vector
        const Vector<T, 3> s = cross(f, up).normalized();    // Right vector
        const Vector<T, 3> u = cross(s, f);                  // Recalculated Up vector

        return Transform(Matrix<T, 4, 4>(s.x(), s.y(), s.z(), -s.dot(eye.to_vector()), u.x(), u.y(), u.z(),
                              -u.dot(eye.to_vector()), -f.x(), -f.y(), -f.z(), f.dot(eye.to_vector()), T(0), T(0), T(0), T(1)));
    }

    static Transform<T> perspective(T fov_y_rad, T aspect_ratio, T z_near, T z_far) noexcept {
        const T tan_half_fovy = std::tan(fov_y_rad / T(2));
        return Transform(Matrix<T, 4, 4>(T(1) / (aspect_ratio * tan_half_fovy), T(0), T(0), T(0), T(0), T(1) / tan_half_fovy, T(0), T(0), T(0), T(0),
                              z_far / (z_far - z_near), -z_far * z_near / (z_far - z_near), T(0), T(0), T(1), T(0)));
    }

    static constexpr Transform<T> orthographic(T left, T right, T bottom, T top, T z_near,
                                            T z_far) noexcept {
        return Transform(Matrix<T, 4, 4>(T(2) / (right - left), T(0), T(0), -(right + left) / (right - left), T(0), T(2) / (top - bottom), T(0),
                              -(top + bottom) / (top - bottom), T(0), T(0), T(1) / (z_far - z_near),
                              -z_near / (z_far - z_near), T(0), T(0), T(0), T(1)));
    }

private:
    Matrix<T, 4, 4> m_mat{};

public:
    constexpr Transform() noexcept = default;
    constexpr Transform(const Matrix<T, 4, 4>& mat) : m_mat(mat) {}

    constexpr const Matrix<T, 4, 4>& mat() const { return m_mat; }

    template<typename U>
    constexpr Transform& operator*=(const Transform<U>& rhs) noexcept {
        m_mat = m_mat * rhs.mat();
        return *this;
    }

    template<typename U>
    constexpr auto operator*(const Transform<U>& rhs) const noexcept {
        using R = std::common_type_t<T, U>;
        return Transform<R>(m_mat * rhs.mat());
    }

    template<typename U>
    constexpr auto operator*(const Point<U, 3>& point) const noexcept {
        using R = std::common_type_t<T, U>;
        return (m_mat * Homogeneous<R, 3>::from_point(point)).to_point();
    }

    template<typename U>
    constexpr auto operator*(const Vector<U, 3>& vec) const noexcept {
        using R = std::common_type_t<T, U>;
        return (m_mat * Homogeneous<R, 3>::from_vector(vec)).to_vector();
    }

    template<typename U>
    constexpr auto operator*(const Normal<U, 3>& normal) const noexcept {
        using R = std::common_type_t<T, U>;
        auto result = (m_mat.inversed().transposed() * Homogeneous<R, 3>::from_vector(normal.to_vector())).to_vector();
        return Normal<R, 3>::from_vector(result);
    }

    template<typename U>
    constexpr auto operator*(const Ray<U, 3>& ray) const noexcept {
        using R = std::common_type_t<T, U>;
        auto origin = (*this) * ray.origin();
        auto direction = (*this) * ray.direction();
        return Ray<R, 3>(origin, direction);
    }

    template<typename U>
    constexpr auto operator*(const Bounds<U, 3>& bound) const noexcept {
        using R = std::common_type_t<T, U>;
        return Bounds<R, 3>{*this * bound.min(), *this * bound.max()};
    }
};

using Trans = Transform<Float>;

}  // namespace pbpt::math
