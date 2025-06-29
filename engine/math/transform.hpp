#pragma once

#include "homogeneous.hpp"
#include "math/bounding_box.hpp"
#include "ray.hpp"
#include "vector.hpp"
#include "matrix.hpp" // Assumes this includes vector.hpp and point.hpp
#include "point.hpp"

/**
 * @file transform.hpp
 * @brief Provides factory functions for creating common 3D transformation matrices.
 * @details This file contains functions to generate matrices for translation, rotation,
 * scaling, camera viewing (look-at), and projection (perspective/orthographic).
 * These are the fundamental building blocks of a Model-View-Projection (MVP) pipeline.
 */

namespace pbpt::math {

/**
 * @brief A namespace for 3D transformation factory functions.
 * @details This namespace contains static functions to generate common 3D transformation matrices.
 * Each function corresponds to a specific transformation (translation, rotation, scaling, etc.)
 * and returns a Mat4 (4x4 matrix) that can be used in 3D rendering or other applications.
 */

class Transform {

public:

    // --- 仿射变换 (Affine Transformations) ---

    /**
    * @brief Creates a 4x4 translation matrix.
    * @param t A Vec3 representing the translation amounts (tx, ty, tz).
    * @return A Mat4 that translates a point by the given vector.
    */
    static constexpr Transform translate(const Vec3& t) noexcept {
        return Transform(Mat4(
            1, 0, 0, t.x(),
            0, 1, 0, t.y(),
            0, 0, 1, t.z(),
            0, 0, 0, 1
        ));
    }

    /**
    * @brief Creates a 4x4 uniform scaling matrix.
    * @param s The scalar factor to scale by on all axes.
    * @return A Mat4 that scales a point or vector uniformly.
    */
    static constexpr Transform scale(Float s) noexcept {
        return Transform(Mat4(
            s, 0, 0, 0,
            0, s, 0, 0,
            0, 0, s, 0,
            0, 0, 0, 1
        ));
    }

    /**
    * @brief Creates a 4x4 non-uniform scaling matrix.
    * @param s A Vec3 containing the scaling factors for the x, y, and z axes.
    * @return A Mat4 that scales a point or vector non-uniformly.
    */
    static constexpr Transform scale(const Vec3& s) noexcept {
        return Transform(Mat4(
            s.x(), 0,     0,     0,
            0,     s.y(), 0,     0,
            0,     0,     s.z(), 0,
            0,     0,     0,     1
        ));
    }

    /**
    * @brief Creates a 4x4 rotation matrix around the X-axis.
    * @param angle_rad The rotation angle in radians.
    * @return A Mat4 that rotates points/vectors around the world's X-axis.
    */
    static constexpr Transform rotate_x(Float angle_rad) noexcept {
        const Float s = sin(angle_rad);
        const Float c = cos(angle_rad);
        return Transform(Mat4(
            1, 0, 0,  0,
            0, c, -s, 0,
            0, s, c,  0,
            0, 0, 0,  1
        ));
    }

    /**
    * @brief Creates a 4x4 rotation matrix around the Y-axis.
    * @param angle_rad The rotation angle in radians.
    * @return A Mat4 that rotates points/vectors around the world's Y-axis.
    */
    static constexpr Transform rotate_y(Float angle_rad) noexcept {
        const Float s = sin(angle_rad);
        const Float c = cos(angle_rad);
        return Transform(Mat4(
            c,  0, s, 0,
            0,  1, 0, 0,
            -s, 0, c, 0,
            0,  0, 0, 1
        ));
    }

    /**
    * @brief Creates a 4x4 rotation matrix around the Z-axis.
    * @param angle_rad The rotation angle in radians.
    * @return A Mat4 that rotates points/vectors around the world's Z-axis.
    */
    static constexpr Transform rotate_z(Float angle_rad) noexcept {
        const Float s = sin(angle_rad);
        const Float c = cos(angle_rad);
        return Transform(Mat4(
            c, -s, 0, 0,
            s, c,  0, 0,
            0, 0,  1, 0,
            0, 0,  0, 1
        ));
    }

    /**
* @brief Creates a rotation matrix from an arbitrary axis and an angle (Rodrigues' rotation formula).
* @param angle_rad The rotation angle in radians.
* @param axis The axis of rotation. Should be a unit vector for correct results.
* @return A Mat4 that performs the specified rotation.
*/
static constexpr Transform rotate(Float angle_rad, const Vec3& axis) noexcept {
    // 确保旋转轴是单位向量，这对于公式的正确性至关重要
    const Vec3 a = axis.normalized();
    const Float s = sin(angle_rad);
    const Float c = cos(angle_rad);
    const Float omc = 1.0f - c; // one-minus-cosine

    const Float ax = a.x();
    const Float ay = a.y();
    const Float az = a.z();

    // 罗德里格斯旋转公式的矩阵形式
    return Transform(Mat4(
        c + ax * ax * omc,       ax * ay * omc - az * s,  ax * az * omc + ay * s,  0,
        ay * ax * omc + az * s,  c + ay * ay * omc,       ay * az * omc - ax * s,  0,
        az * ax * omc - ay * s,  az * ay * omc + ax * s,  c + az * az * omc,       0,
        0,                       0,                       0,                       1
    ));
}

    // --- 视图/摄像机变换 (View/Camera Transformations) ---

    /**
    * @brief Creates a view matrix (a.k.a. camera matrix) using the "look-at" method.
    * @details This matrix transforms coordinates from world space to view (camera) space.
    * @param eye The position of the camera in world space.
    * @param target The point in world space that the camera is looking at.
    * @param up A vector indicating the "up" direction of the world (usually (0, 1, 0)).
    * @return A Mat4 representing the view transformation.
    */
    static constexpr Transform look_at(const Pt3& eye, const Pt3& target, const Vec3& up) noexcept {
        const Vec3 f = (target - eye).normalized(); // Forward vector
        const Vec3 s = f.cross(up).normalized();    // Right vector
        const Vec3 u = s.cross(f);                  // Recalculated Up vector

        return Transform(Mat4(
            s.x(),  s.y(),  s.z(),  -s.dot(eye.to_vector()),
            u.x(),  u.y(),  u.z(),  -u.dot(eye.to_vector()),
            -f.x(), -f.y(), -f.z(), f.dot(eye.to_vector()),
            0,      0,      0,      1
        ));
    }


    // --- 投影变换 (Projection Transformations) ---

    /**
    * @brief Creates a left-handed perspective projection matrix.
    * @details This matrix transforms coordinates from view space to clip space, creating
    * the illusion of depth.
    * @param fov_y_rad Vertical field of view, in radians.
    * @param aspect_ratio The aspect ratio of the viewport (width / height).
    * @param z_near Distance to the near clipping plane (must be positive).
    * @param z_far Distance to the far clipping plane (must be positive and > z_near).
    * @return A Mat4 representing the perspective projection.
    */
    static constexpr Transform perspective(Float fov_y_rad, Float aspect_ratio, Float z_near, Float z_far) noexcept {
        const Float tan_half_fovy = tan(fov_y_rad / 2.0);
        return Transform(Mat4(
            1.0 / (aspect_ratio * tan_half_fovy), 0,                     0,                        0,
            0,                                    1.0 / (tan_half_fovy), 0,                        0,
            0,                                    0,                     z_far / (z_far - z_near), -z_far * z_near / (z_far - z_near),
            0,                                    0,                     1.0,                      0
        ));
       
    }

    /**
    * @brief Creates a left-handed orthographic projection matrix.
    * @details This matrix transforms coordinates from view space to clip space without any
    * perspective distortion. It maps a rectangular box (view volume) to the
    * canonical view volume [-1, 1]^3.
    * @param left The x-coordinate of the left clipping plane.
    * @param right The x-coordinate of the right clipping plane.
    * @param bottom The y-coordinate of the bottom clipping plane.
    * @param top The y-coordinate of the top clipping plane.
    * @param z_near The z-coordinate of the near clipping plane.
    * @param z_far The z-coordinate of the far clipping plane.
    * @return A Mat4 representing the orthographic projection.
    */
    static constexpr Transform orthographic(Float left, Float right, Float bottom, Float top, Float z_near, Float z_far) noexcept {
  
        return Transform(Mat4(
            2.0 / (right - left), 0,                    0,                      -(right + left) / (right - left),
            0,                    2.0 / (top - bottom), 0,                      -(top + bottom) / (top - bottom),
            0,                    0,                    1.0 / (z_far - z_near), -z_near / (z_far - z_near),
            0,                    0,                    0,                      1
        ));
    }

private:
    Mat4 m_mat{};

public:
    constexpr Transform() noexcept = default;
    constexpr Transform(const Mat4& mat) : m_mat(mat) {}

    constexpr const Mat4& mat() const { return m_mat; }

    constexpr Transform& operator*=(const Transform& rhs) noexcept {
        m_mat = m_mat * rhs.m_mat;
        return *this;
    }

    constexpr Transform operator*(const Transform& rhs) const noexcept {
        return Transform(m_mat * rhs.m_mat);
    }

    /**
    * @brief Transforms a point using the transform matrix.
    * @param point The point to transform.
    * @return The transformed point.
    */
    constexpr Pt3 operator*(const Pt3& point) const noexcept {
        return (m_mat * Homo3(point)).to_point();
    }

    /**
    * @brief Transforms a vector using the transform matrix.
    * @param vec The vector to transform.
    * @return The transformed vector.
    */
    constexpr Vec3 operator*(const Vec3& vec) const noexcept {
        return (m_mat * Homo3(vec)).to_vector();
    }

    /**
    * @brief Transforms a normal using the transform matrix.
    * @param normal The normal to transform.
    * @return The transformed normal.
    */
    constexpr Normal3 operator*(const Normal3& normal) const noexcept {
        auto result = (m_mat.inversed().transposed() * Homo3(normal)).to_vector();
        return Normal3(result);
    }

    constexpr Ray3 operator*(const Ray3& ray) const noexcept {
        return Ray3(
            *this * ray.origin(), 
            *this * ray.direction()
        );
    }

    constexpr Bound3 operator*(const Bound3& bound) const noexcept {
        return Bound3 {*this * bound.min(), *this * bound.max()};;
    }
    
};


} // namespace pbpt