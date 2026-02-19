/**
 * @file
 * @brief Rigid and projective transforms and fast decomposition utilities.
 */
#pragma once

#include <utility>

#include "pbpt/geometry/interaction.hpp"
#include "pbpt/math/homogeneous.hpp"
#include "pbpt/math/type_alias.hpp"
#include "pbpt/math/matrix.hpp"
#include "pbpt/math/normal.hpp"
#include "pbpt/math/point.hpp"
#include "pbpt/math/vector.hpp"

#include "bounds.hpp"
#include "ray.hpp"
#include "pbpt/math/format.hpp"
#include <iostream>

using namespace pbpt::math;

namespace pbpt::geometry {

/**
 * @brief 4×4 homogeneous transform with cached inverse.
 *
 * This class represents general homogeneous transforms in 3D, including
 * translation, rotation, scaling and perspective projections. It stores
 * both a forward matrix and its inverse, so that transforming geometry
 * and transforming normals can be implemented efficiently.
 *
 * @tparam T Scalar type.
 */
template <typename T>
class Transform {
private:
    Matrix<T, 4, 4> m_mat;
    Matrix<T, 4, 4> m_inv;

public:
    constexpr Transform<T>() : m_mat(Matrix<T, 4, 4>::identity()), m_inv(Matrix<T, 4, 4>::identity()) {}

    /// Construct from a 4×4 matrix; the inverse is computed automatically.
    constexpr Transform<T>(const Matrix<T, 4, 4>& m) : m_mat(m), m_inv(m.inversed()) {}

    /// Construct from a matrix and its explicit inverse.
    constexpr Transform<T>(const Matrix<T, 4, 4>& m, const Matrix<T, 4, 4>& inv) : m_mat(m), m_inv(inv) {}

    /// Identity transform.
    static constexpr Transform<T> identity() { return Transform<T>(Matrix<T, 4, 4>::identity()); }

    /**
     * @brief Translation by vector @p t.
     */
    static constexpr Transform<T> translate(const Vector<T, 3>& t) {
        Matrix<T, 4, 4> m(T(1), T(0), T(0), t.x(), T(0), T(1), T(0), t.y(), T(0), T(0), T(1), t.z(), T(0), T(0), T(0),
                          T(1));
        Matrix<T, 4, 4> inv(T(1), T(0), T(0), -t.x(), T(0), T(1), T(0), -t.y(), T(0), T(0), T(1), -t.z(), T(0), T(0),
                            T(0), T(1));
        return Transform<T>(m, inv);
    }

    /// Uniform scaling by factor @p s.
    static constexpr Transform<T> scale(T s) { return scale(Vector<T, 3>{s, s, s}); }

    /// Non-uniform scaling along x, y, z.
    static constexpr Transform<T> scale(const Vector<T, 3>& s) {
        Matrix<T, 4, 4> m(s.x(), T(0), T(0), T(0), T(0), s.y(), T(0), T(0), T(0), T(0), s.z(), T(0), T(0), T(0), T(0),
                          T(1));
        Matrix<T, 4, 4> inv(T(1) / s.x(), T(0), T(0), T(0), T(0), T(1) / s.y(), T(0), T(0), T(0), T(0), T(1) / s.z(),
                            T(0), T(0), T(0), T(0), T(1));
        return Transform<T>(m, inv);
    }

    /// Rotation around the x-axis by @p angle_rad (right-handed).
    static Transform<T> rotate_x(T angle_rad) {
        T s = std::sin(angle_rad), c = std::cos(angle_rad);
        Matrix<T, 4, 4> m(T(1), T(0), T(0), T(0), T(0), c, -s, T(0), T(0), s, c, T(0), T(0), T(0), T(0), T(1));
        return Transform<T>(m, m.transposed());
    }

    /// Rotation around the y-axis by @p angle_rad (right-handed).
    static Transform<T> rotate_y(T angle_rad) {
        T s = std::sin(angle_rad), c = std::cos(angle_rad);
        Matrix<T, 4, 4> m(c, T(0), s, T(0), T(0), T(1), T(0), T(0), -s, T(0), c, T(0), T(0), T(0), T(0), T(1));
        return Transform<T>(m, m.transposed());
    }

    /// Rotation around the z-axis by @p angle_rad (right-handed).
    static Transform<T> rotate_z(T angle_rad) {
        T s = std::sin(angle_rad), c = std::cos(angle_rad);
        Matrix<T, 4, 4> m(c, -s, T(0), T(0), s, c, T(0), T(0), T(0), T(0), T(1), T(0), T(0), T(0), T(0), T(1));
        return Transform<T>(m, m.transposed());
    }

    /**
     * @brief Rotation by @p angle_rad around an arbitrary axis.
     *
     * The axis is normalized internally before constructing the matrix.
     */
    static Transform<T> rotate(T angle_rad, const Vector<T, 3>& axis) {
        Vector<T, 3> a = axis.normalized();
        T s = std::sin(angle_rad), c = std::cos(angle_rad), omc = T(1) - c;
        T ax = a.x(), ay = a.y(), az = a.z();

        Matrix<T, 4, 4> m(c + ax * ax * omc, ax * ay * omc - az * s, ax * az * omc + ay * s, T(0),
                          ay * ax * omc + az * s, c + ay * ay * omc, ay * az * omc - ax * s, T(0),
                          az * ax * omc - ay * s, az * ay * omc + ax * s, c + az * az * omc, T(0), T(0), T(0), T(0),
                          T(1));
        return Transform<T>(m, m.transposed());
    }

    /**
     * @brief Promote a 3×3 matrix to a 4×4 homogeneous transform.
     *
     * The bottom row and rightmost column are set to represent an affine
     * transform with no translation.
     */
    static Transform<T> from_mat3x3(Matrix<T, 3, 3> m) {
        Matrix<T, 4, 4> mat(m.at(0, 0), m.at(0, 1), m.at(0, 2), T(0), m.at(1, 0), m.at(1, 1), m.at(1, 2), T(0),
                            m.at(2, 0), m.at(2, 1), m.at(2, 2), T(0), T(0), T(0), T(0), T(1));
        return Transform<T>(mat, mat.inversed());
    }

    /**
     * @brief View matrix that looks from @p eye to @p target with up-vector @p up.
     *
     * Builds a right-handed look-at matrix that transforms points from
     * world space into camera space.
     */
    static constexpr Transform<T> look_at(const Point<T, 3>& eye, const Point<T, 3>& target, const Vector<T, 3>& up) {
        Vector<T, 3> f = (target - eye).normalized();
        Vector<T, 3> s = cross(f, up).normalized();
        Vector<T, 3> u = cross(s, f);
        Matrix<T, 4, 4> m(s.x(), s.y(), s.z(), -s.dot(eye.to_vector()), u.x(), u.y(), u.z(), -u.dot(eye.to_vector()),
                          -f.x(), -f.y(), -f.z(), f.dot(eye.to_vector()), T(0), T(0), T(0), T(1));
        return Transform<T>(m, m.inversed());
    }

    /**
     * @brief Orthographic projection from camera space to clip space.
     *
     * Maps the axis-aligned frustum defined by left/right, bottom/top
     * and near/far planes into the canonical clip space cube.
     */
    static constexpr Transform<T> orthographic(T left, T right, T bottom, T top, T near, T far) {
        Matrix<T, 4, 4> m(T(2) / (right - left), T(0), T(0), -(right + left) / (right - left), T(0),
                          T(2) / (top - bottom), T(0), -(top + bottom) / (top - bottom), T(0), T(0),
                          T(1) / (far - near), -near / (far - near), T(0), T(0), T(0), T(1));
        return Transform<T>(m, m.inversed());
    }

    /**
     * @brief Perspective-to-orthographic helper used in perspective().
     *
     * This transform maps a perspective frustum into an intermediate
     * space where an orthographic transform can be applied.
     */
    static Transform<T> pesp_to_ortho(T near, T far) {
        Matrix<T, 4, 4> m(near, T(0), T(0), T(0), T(0), near, T(0), T(0), T(0), T(0), near + far, -near * far, T(0),
                          T(0), T(1), T(0));
        return Transform<T>(m, m.inversed());
    }

    /**
     * @brief Perspective projection from camera space to clip space.
     *
     * Uses a vertical field of view, aspect ratio and near/far planes.
     */
    static Transform<T> perspective(T fov_y_rad, T aspect_xy, T near, T far) {
        auto persp_to_ortho = pesp_to_ortho(near, far);
        T right = near * std::tan(fov_y_rad / T(2)) * aspect_xy, left = -right;
        T top = near * std::tan(fov_y_rad / T(2)), bottom = -top;
        return orthographic(left, right, bottom, top, near, far) * persp_to_ortho;
    }

    /**
     * @brief Transform from clip space to viewport (raster) space.
     *
     * Perspective division can be applied before or after this transform.
     */
    static Transform<T> viewport(T width, T height) {
        Matrix<T, 4, 4> m(width / T(2), T(0), T(0), width / T(2), T(0), height / T(2), T(0), height / T(2), T(0), T(0),
                          T(1), T(0), T(0), T(0), T(0), T(1));
        return Transform<T>(m, m.inversed());
    }

    /// Compare two transforms for exact matrix equality.
    constexpr bool operator==(const Transform<T>& rhs) const { return m_mat == rhs.m_mat; }

    /// Compare two transforms for inequality.
    constexpr bool operator!=(const Transform<T>& rhs) const { return !(*this == rhs); }

    /// Get the forward 4×4 matrix.
    constexpr const Matrix<T, 4, 4>& matrix() const { return m_mat; }
    /// Get the inverse 4×4 matrix.
    constexpr const Matrix<T, 4, 4>& inverse_matrix() const { return m_inv; }

    /// Set the matrix and recompute its inverse.
    constexpr Transform<T>& set_matrix(const Matrix<T, 4, 4>& m) {
        m_mat = m;
        m_inv = m.inversed();
        return *this;
    }

    /// Set both matrix and inverse explicitly.
    constexpr Transform<T>& set_matrix(const Matrix<T, 4, 4>& m, const Matrix<T, 4, 4>& inv) {
        m_mat = m;
        m_inv = inv;
        return *this;
    }

    /// Test whether this is exactly the identity transform.
    constexpr bool is_identity() const { return m_mat.is_identity(); }

    /**
     * @brief Test whether the transform contains non-uniform scaling.
     *
     * This is done by transforming the canonical basis vectors and
     * checking whether their lengths remain 1.
     */
    constexpr bool has_scale() const {
        return !is_equal(((*this).transform_vector(Vector<T, 3>(1, 0, 0))).length_squared(), T(1)) ||
               !is_equal(((*this).transform_vector(Vector<T, 3>(0, 1, 0))).length_squared(), T(1)) ||
               !is_equal(((*this).transform_vector(Vector<T, 3>(0, 0, 1))).length_squared(), T(1));
    }

    /// Invert this transform in-place by swapping matrix and inverse.
    constexpr Transform<T>& inverse() {
        std::swap(m_mat, m_inv);
        return *this;
    }

    /// Return a new transform that is the inverse of this one.
    constexpr Transform<T> inversed() const { return Transform<T>(m_inv, m_mat); }

    /// Transpose the transform matrix and its inverse in-place.
    constexpr Transform<T>& transpose() {
        m_mat.transpose();
        m_inv.transpose();
        return *this;
    }

    /**
     * @brief Check whether the transform reverses handedness.
     *
     * This is determined by the sign of the determinant of the upper
     * 3×3 linear part.
     */
    constexpr bool is_swaps_handedness() const {
        Matrix<T, 3, 3> upper3x3 = m_mat.template view<3, 3>(0, 0).to_matrix();
        return upper3x3.determinant() < T(0);
    }

    /// Return a new transform whose matrix is the transpose of this one.
    constexpr Transform<T> transposed() const { return Transform<T>(m_mat.transposed(), m_inv.transposed()); }

    /// Compose two transforms (this followed by @p rhs).
    constexpr Transform operator*(const Transform& rhs) const {
        return Transform(m_mat * rhs.matrix(), rhs.inverse_matrix() * m_inv);
    }

    /// Transform a homogeneous 4D vector.
    constexpr Homogeneous<T, 4> transform_homogeneous(const Homogeneous<T, 4>& rhs) const { return m_mat * rhs; }

    /// Transform a 3D point.
    constexpr Point<T, 3> transform_point(const Point<T, 3>& p) const {
        auto result = (m_mat * Homogeneous<T, 4>::from_point(p));
        return result.to_point();
    }

    /// Transform a 3D vector (ignores translation).
    constexpr Vector<T, 3> transform_vector(const Vector<T, 3>& v) const {
        auto result = (m_mat * Homogeneous<T, 4>::from_vector(v));
        return result.to_vector();
    }

    /**
     * @brief Transform a surface normal.
     *
     * Uses the inverse-transpose of the linear part of the transform,
     * which is required for correct transformation in the presence of
     * non-uniform scale.
     */
    constexpr Normal<T, 3> transform_normal(const Normal<T, 3>& n) const {
        auto inv_t = m_inv.transposed();
        auto linear_part = inv_t.template view<3, 3>(0, 0).to_matrix();
        auto transformed = linear_part * n.to_vector();
        return Normal<T, 3>::from_vector(transformed);
    }

    /// Transform a 3D ray (origin and direction, preserving t-range).
    constexpr Ray<T, 3> transform_ray_main(const Ray<T, 3>& ray) const {
        return Ray<T, 3>(transform_point(ray.origin()), transform_vector(ray.direction()), ray.t_max(), ray.t_min());
    }

    /// Transform a 3D ray differential (main ray and differential rays).
    constexpr RayDifferential<T, 3> transform_ray_differential(const RayDifferential<T, 3>& ray_diff) const {
        std::array<Ray<T, 3>, 2> diff_rays = {transform_ray_main(ray_diff.x()), transform_ray_main(ray_diff.y())};
        return RayDifferential<T, 3>(transform_ray_main(ray_diff.main_ray()), diff_rays);
    }

    /// Transform an axis-aligned 3D bounding box by transforming all corners.
    constexpr Bounds<T, 3> transform_bounds(const Bounds<T, 3>& b) const {
        Bounds<T, 3> result;
        for (int i = 0; i < 8; i++) {
            result.unite(transform_point(b.corner(i)));
        }
        return result;
    }

    /**
     * @brief Transform a @c SurfaceInteraction into this transform's space.
     *
     * The point bounds, direction and geometric normal are transformed
     * consistently. If the transform flips handedness, the normal is
     * also flipped.
     */
    constexpr auto transform_surface_interaction(const SurfaceInteraction<T>& si) const {
        auto n = this->transform_normal(si.n());
        auto dndu = this->transform_normal(si.dndu());
        auto dndv = this->transform_normal(si.dndv());

        if (this->is_swaps_handedness()) {
            n = -n;
            dndu = -dndu;
            dndv = -dndv;
        }

        auto result = SurfaceInteraction<T>(
            this->transform_point(si.p_lower()), this->transform_point(si.p_upper()), this->transform_vector(si.wo()),
            n, si.uv(), this->transform_vector(si.dpdu()), this->transform_vector(si.dpdv()), dndu, dndv);
        return result;
    }

    constexpr auto transform_volume(const T area) const {
        // Scale area by the determinant of the upper-left 3x3 matrix
        Matrix<T, 3, 3> linear_part = m_mat.template view<3, 3>(0, 0).to_matrix();
        T scale_factor = std::abs(linear_part.determinant());
        return area * scale_factor;
    }
};

/// Common alias for a float-based transform.
using Trans = Transform<Float>;

/**
 * @brief Result of decomposing a transform into translation, rotation and scale.
 *
 * Each component is itself a @c Transform so that they can be applied
 * independently.
 */
template <typename T>
struct TransformDecompositionResult {
    /// Pure translation component T in the decomposition M = T * R * S.
    Transform<T> translation{};
    /// Pure rotation component R in the decomposition M = T * R * S.
    Transform<T> rotation{};
    /// Pure scale component S in the decomposition M = T * R * S.
    Transform<T> scale{};
};

/**
 * @brief Fast decomposition of a transform into translation, rotation and scale.
 *
 * Assumes the matrix has no shear and can be written as
 *   M = T * R * S
 * where T is translation, R is a rotation matrix, and S is a diagonal
 * scaling matrix. The algorithm:
 * 1. Extract translation from the last column.
 * 2. Extract the upper-left 3×3 block A = R * S.
 * 3. Compute column norms of A as scale factors.
 * 4. Divide columns of A by these scales to obtain R.
 * 5. Optionally apply Gram–Schmidt to improve orthogonality.
 * 6. Fix handedness if det(R) < 0 by flipping one axis.
 *
 * @param transform Transform to decompose.
 * @return Translation, rotation and scale components as transforms.
 */
template <typename T>
TransformDecompositionResult<T> fast_decompose_transform(const Transform<T>& transform) {
    const auto& M4 = transform.matrix();  // 4x4
    using Mat3 = Matrix<T, 3, 3>;
    using Vec3 = Vector<T, 3>;

    // --- 1) Translation ---
    const Vec3 t(M4.at(0, 3), M4.at(1, 3), M4.at(2, 3));
    Transform<T> Txf = Transform<T>::translate(t);

    // --- 2) A = upper-left 3x3 ---
    MatrixView<T, 4, 4, 3, 3> Aview{M4, 0, 0};
    Matrix<T, 3, 3> A = Aview.to_matrix();  // A = R * S  (no shear assumed)

    // --- 3) scales = column norms ---
    Vector<T, 3> s{};
    const T eps = T(1e-12);
    for (int c = 0; c < 3; ++c) {
        // 手动求列范数（VectorView 无 /=）
        T n2 = T(0);
        for (int r = 0; r < 3; ++r)
            n2 += A.at(r, c) * A.at(r, c);
        T n = std::sqrt(n2);
        s[c] = (n > eps) ? n : T(1);  // 退化保护
    }

    // --- 4) R = A * S^{-1}：逐列除尺度 ---
    Matrix<T, 3, 3> R = A;
    for (int c = 0; c < 3; ++c) {
        T invs = T(1) / s[c];
        for (int r = 0; r < 3; ++r)
            R.at(r, c) *= invs;
    }

    // --- 5) 可选：简单正交化（Gram–Schmidt），提升稳健性 ---
    auto normalize_col = [&](int c) {
        T n2 = T(0);
        for (int r = 0; r < 3; ++r)
            n2 += R.at(r, c) * R.at(r, c);
        T n = std::sqrt(n2);
        if (n > eps)
            for (int r = 0; r < 3; ++r)
                R.at(r, c) /= n;
    };
    // Gram–Schmidt: c0, c1, c2
    // c1 := c1 - proj_{c0}(c1)
    auto dot_col = [&](int i, int j) {
        T s = T(0);
        for (int r = 0; r < 3; ++r)
            s += R.at(r, i) * R.at(r, j);
        return s;
    };
    auto axpy_col = [&](int dst, int src, T alpha) {
        for (int r = 0; r < 3; ++r)
            R.at(r, dst) -= alpha * R.at(r, src);
    };
    normalize_col(0);
    T a10 = dot_col(1, 0);
    axpy_col(1, 0, a10);
    normalize_col(1);
    // c2 := c2 - proj_{c0}(c2) - proj_{c1}(c2)
    T a20 = dot_col(2, 0);
    axpy_col(2, 0, a20);
    T a21 = dot_col(2, 1);
    axpy_col(2, 1, a21);
    normalize_col(2);

    // --- 6) 右手系修正（det(R) < 0） ---
    T detR = R.determinant();
    if (detR < T(0)) {
        // 翻转最大尺度的那一列，使 det(R) -> +1，同时尺度取负
        int k = 0;
        if (std::abs(s[1]) > std::abs(s[k]))
            k = 1;
        if (std::abs(s[2]) > std::abs(s[k]))
            k = 2;
        s[k] = -s[k];
        for (int r = 0; r < 3; ++r)
            R.at(r, k) = -R.at(r, k);
    }

    // --- 7) 输出 ---
    Transform<T> Rxf = Transform<T>::from_mat3x3(R);
    Transform<T> Sxf = Transform<T>::scale(s);
    return {Txf, Rxf, Sxf};
}

}  // namespace pbpt::geometry
