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

template <typename T>
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

    // view matrix
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

    // projection matrix, from camera space to clip space
    static constexpr Transform<T> orthographic(T left, T right, T bottom, T top, T near, T far) {
        Matrix<T, 4, 4> m(
            T(2) / (right - left), T(0), T(0), -(right + left) / (right - left),
            T(0), T(2) / (top - bottom), T(0), -(top + bottom) / (top - bottom),
            T(0), T(0), T(1) / (far - near), -near / (far - near),
            T(0), T(0), T(0), T(1)
        );
        return Transform<T>(m, m.inversed());
    }

    static Transform<T> pesp_to_ortho(T near, T far) {
        Matrix<T, 4, 4> m(
            near, T(0), T(0), T(0),
            T(0), near, T(0), T(0),
            T(0), T(0), near + far, -near * far,
            T(0), T(0), T(1), T(0)
        );
        return Transform<T>(m, m.inversed());
    }

    static Transform<T> perspective(T fov_y_rad, T aspect_xy, T near, T far) {
        auto persp_to_ortho = pesp_to_ortho(near, far);
        T right = near * std::tan(fov_y_rad / T(2)) * aspect_xy, left = -right;
        T top = near * std::tan(fov_y_rad / T(2)), bottom = -top;
        return orthographic(left, right, bottom, top, near, far) * persp_to_ortho;
    }

    // from clip space to viewport space, perspective division can be done before or after this transform
    static Transform<T> viewport(T width, T height) {
        Matrix<T, 4, 4> m(
            width / T(2), T(0), T(0), width / T(2),
            T(0), height / T(2), T(0), height / T(2),
            T(0), T(0), T(1), T(0),
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
    constexpr auto transform_homogeneous(const Homogeneous<U, 4>& rhs) const {
        using R = std::common_type_t<T, U>;
        return m_mat * rhs;
    }

    template<typename U>
    constexpr auto operator*(const Homogeneous<U, 4>& rhs) const {
        using R = std::common_type_t<T, U>;
        return m_mat * rhs;
    }

    template<typename U>
    constexpr auto transform_point(const Point<U, 3>& p) const {
        using R = std::common_type_t<T, U>;
        return (m_mat * Homogeneous<R, 4>::from_point(p)).to_point();
    }

    template<typename U>
    constexpr auto operator*(const Point<U, 3>& p) const {
        using R = std::common_type_t<T, U>;
        return (m_mat * Homogeneous<R, 4>::from_point(p)).to_point();
    }

    template<typename U>
    constexpr auto transform_vector(const Vector<U, 3>& v) const {
        using R = std::common_type_t<T, U>;
        return (m_mat * Homogeneous<R, 4>::from_vector(v)).to_vector();
    }

    template<typename U>
    constexpr auto operator*(const Vector<U, 3>& v) const {
        using R = std::common_type_t<T, U>;
        return (m_mat * Homogeneous<R, 4>::from_vector(v)).to_vector();
    }

    template<typename U>
    constexpr auto transform_normal(const Normal<U, 3>& n) const {
        using R = std::common_type_t<T, U>;
        auto r = m_inv.transposed() * Homogeneous<R, 4>::from_vector(n.to_vector());
        return Normal<R, 3>::from_vector(r.to_vector());
    }

    template<typename U>
    constexpr auto operator*(const Normal<U, 3>& n) const {
        using R = std::common_type_t<T, U>;
        auto r = m_inv.transposed() * Homogeneous<R, 4>::from_vector(n.to_vector());
        return Normal<R, 3>::from_vector(r.to_vector());
    }

    template<typename U>
    constexpr auto transform_ray(const Ray<U, 3>& ray) const {
        using R = std::common_type_t<T, U>;
        return Ray<R, 3>((*this) * ray.origin(), (*this) * ray.direction());
    }

    template<typename U>
    constexpr auto operator*(const Ray<U, 3>& ray) const {
        using R = std::common_type_t<T, U>;
        return Ray<R, 3>((*this) * ray.origin(), (*this) * ray.direction());
    }

    template<typename U>
    constexpr auto transform_bounds(const Bounds<U, 3>& b) const {
        using R = std::common_type_t<T, U>;
        Bounds<R, 3> result;
        for (int i = 0; i < 8; i ++) {
            result.unite((*this) * b.corner(i));
        }
        return result;
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

template<typename T>
struct TransformDecompositionResult {
    Transform<T> translation{};
    Transform<T> rotation{};
    Transform<T> scale{};
};

//decompose a transform matrix to translation, rotation and scale
// assume the matrix is composed by T * R * S

// fast decomposition without shear (column-vector convention): M = T * R * S
template<typename T>
TransformDecompositionResult<T> fast_decompose_transform(const Transform<T>& transform) {
    const auto& M4 = transform.matrix();            // 4x4
    using Mat3 = Matrix<T,3,3>;
    using Vec3 = Vector<T,3>;

    // --- 1) Translation ---
    const Vec3 t(M4.at(0,3), M4.at(1,3), M4.at(2,3));
    Transform<T> Txf = Transform<T>::translate(t);

    // --- 2) A = upper-left 3x3 ---
    MatrixView<T, 4, 4, 3, 3> Aview{ M4, 0, 0 };
    Matrix<T, 3, 3> A = Aview.to_matrix();                     // A = R * S  (no shear assumed)

    // --- 3) scales = column norms ---
    Vector<T, 3> s{};
    const T eps = T(1e-12);
    for (int c = 0; c < 3; ++c) {
        // 手动求列范数（VectorView 无 /=）
        T n2 = T(0);
        for (int r = 0; r < 3; ++r) n2 += A.at(r,c) * A.at(r,c);
        T n = std::sqrt(n2);
        s[c] = (n > eps) ? n : T(1);    // 退化保护
    }

    // --- 4) R = A * S^{-1}：逐列除尺度 ---
    Matrix<T, 3, 3> R = A;
    for (int c = 0; c < 3; ++c) {
        T invs = T(1) / s[c];
        for (int r = 0; r < 3; ++r) R.at(r,c) *= invs;
    }

    // --- 5) 可选：简单正交化（Gram–Schmidt），提升稳健性 ---
    auto normalize_col = [&](int c){
        T n2 = T(0); for (int r=0;r<3;++r) n2 += R.at(r,c)*R.at(r,c);
        T n = std::sqrt(n2); if (n > eps) for (int r=0;r<3;++r) R.at(r,c) /= n;
    };
    // Gram–Schmidt: c0, c1, c2
    // c1 := c1 - proj_{c0}(c1)
    auto dot_col = [&](int i, int j){
        T s = T(0); for (int r=0;r<3;++r) s += R.at(r,i)*R.at(r,j); return s;
    };
    auto axpy_col = [&](int dst, int src, T alpha){
        for (int r=0;r<3;++r) R.at(r,dst) -= alpha * R.at(r,src);
    };
    normalize_col(0);
    T a10 = dot_col(1,0); axpy_col(1,0,a10); normalize_col(1);
    // c2 := c2 - proj_{c0}(c2) - proj_{c1}(c2)
    T a20 = dot_col(2,0); axpy_col(2,0,a20);
    T a21 = dot_col(2,1); axpy_col(2,1,a21);
    normalize_col(2);

    // --- 6) 右手系修正（det(R) < 0） ---
    T detR = R.determinant();
    if (detR < T(0)) {
        // 翻转最大尺度的那一列，使 det(R) -> +1，同时尺度取负
        int k = 0;
        if (std::abs(s[1]) > std::abs(s[k])) k = 1;
        if (std::abs(s[2]) > std::abs(s[k])) k = 2;
        s[k] = -s[k];
        for (int r = 0; r < 3; ++r) R.at(r,k) = -R.at(r,k);
    }

    // --- 7) 输出 ---
    Transform<T> Rxf = Transform<T>::from_mat3x3(R);
    Transform<T> Sxf = Transform<T>::scale(s);
    return { Txf, Rxf, Sxf };
}

} // namespace pbpt::geometry

