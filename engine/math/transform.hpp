#pragma once

#include <cmath> // For sin, cos, tan
#include "matrix.hpp" // Assumes this includes vector.hpp and point.hpp

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
    static constexpr Mat4 translate(const Vec3& t) noexcept {
        return Mat4(
            1, 0, 0, t.x(),
            0, 1, 0, t.y(),
            0, 0, 1, t.z(),
            0, 0, 0, 1
        );
    }

    /**
    * @brief Creates a 4x4 uniform scaling matrix.
    * @param s The scalar factor to scale by on all axes.
    * @return A Mat4 that scales a point or vector uniformly.
    */
    static constexpr Mat4 scale(Float s) noexcept {
        return Mat4(
            s, 0, 0, 0,
            0, s, 0, 0,
            0, 0, s, 0,
            0, 0, 0, 1
        );
    }

    /**
    * @brief Creates a 4x4 non-uniform scaling matrix.
    * @param s A Vec3 containing the scaling factors for the x, y, and z axes.
    * @return A Mat4 that scales a point or vector non-uniformly.
    */
    static constexpr Mat4 scale(const Vec3& s) noexcept {
        return Mat4(
            s.x(), 0, 0, 0,
            0, s.y(), 0, 0,
            0, 0, s.z(), 0,
            0, 0, 0, 1
        );
    }

    /**
    * @brief Creates a 4x4 rotation matrix around the X-axis.
    * @param angle_rad The rotation angle in radians.
    * @return A Mat4 that rotates points/vectors around the world's X-axis.
    */
    static constexpr Mat4 rotate_x(Float angle_rad) noexcept {
        const Float s = std::sin(angle_rad);
        const Float c = std::cos(angle_rad);
        return Mat4(
            1, 0, 0, 0,
            0, c, -s, 0,
            0, s, c, 0,
            0, 0, 0, 1
        );
    }

    /**
    * @brief Creates a 4x4 rotation matrix around the Y-axis.
    * @param angle_rad The rotation angle in radians.
    * @return A Mat4 that rotates points/vectors around the world's Y-axis.
    */
    static constexpr Mat4 rotate_y(Float angle_rad) noexcept {
        const Float s = std::sin(angle_rad);
        const Float c = std::cos(angle_rad);
        return Mat4(
            c, 0, s, 0,
            0, 1, 0, 0,
            -s, 0, c, 0,
            0, 0, 0, 1
        );
    }

    /**
    * @brief Creates a 4x4 rotation matrix around the Z-axis.
    * @param angle_rad The rotation angle in radians.
    * @return A Mat4 that rotates points/vectors around the world's Z-axis.
    */
    static constexpr Mat4 rotate_z(Float angle_rad) noexcept {
        const Float s = std::sin(angle_rad);
        const Float c = std::cos(angle_rad);
        return Mat4(
            c, -s, 0, 0,
            s, c, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1
        );
    }

    /**
* @brief Creates a rotation matrix from an arbitrary axis and an angle (Rodrigues' rotation formula).
* @param angle_rad The rotation angle in radians.
* @param axis The axis of rotation. Should be a unit vector for correct results.
* @return A Mat4 that performs the specified rotation.
*/
static constexpr Mat4 rotate(Float angle_rad, const Vec3& axis) noexcept {
    // 确保旋转轴是单位向量，这对于公式的正确性至关重要
    const Vec3 a = axis.normalized();
    const Float s = std::sin(angle_rad);
    const Float c = std::cos(angle_rad);
    const Float omc = 1.0f - c; // one-minus-cosine

    const Float ax = a.x();
    const Float ay = a.y();
    const Float az = a.z();

    // 罗德里格斯旋转公式的矩阵形式
    return Mat4(
        c + ax * ax * omc,       ax * ay * omc - az * s,  ax * az * omc + ay * s,  0,
        ay * ax * omc + az * s,  c + ay * ay * omc,       ay * az * omc - ax * s,  0,
        az * ax * omc - ay * s,  az * ay * omc + ax * s,  c + az * az * omc,       0,
        0,                       0,                       0,                       1
    );
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
    static constexpr Mat4 look_at(const Pt3& eye, const Pt3& target, const Vec3& up) noexcept {
        const Vec3 f = (target - eye).normalized(); // Forward vector
        const Vec3 s = f.cross(up).normalized();    // Right vector
        const Vec3 u = s.cross(f);                  // Recalculated Up vector

        Mat4 result = Mat4::identity();
        result(0, 0) = s.x(); result(0, 1) = s.y(); result(0, 2) = s.z();
        result(1, 0) = u.x(); result(1, 1) = u.y(); result(1, 2) = u.z();
        result(2, 0) = -f.x(); result(2, 1) = -f.y(); result(2, 2) = -f.z();

        // The translation part moves the world origin to the camera's position.
        result(0, 3) = -s.dot(static_cast<Vec3>(eye));
        result(1, 3) = -u.dot(static_cast<Vec3>(eye));
        result(2, 3) = f.dot(static_cast<Vec3>(eye));

        return result;
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
    static constexpr Mat4 perspective(Float fov_y_rad, Float aspect_ratio, Float z_near, Float z_far) noexcept {
        const Float tan_half_fovy = std::tan(fov_y_rad / 2.0);
        Mat4 result = Mat4::zeros();

        result(0, 0) = 1.0 / (aspect_ratio * tan_half_fovy);
        result(1, 1) = 1.0 / (tan_half_fovy);
        result(2, 2) = z_far / (z_far - z_near);
        result(2, 3) = -(z_far * z_near) / (z_far - z_near);
        result(3, 2) = 1.0;

        return result;
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
    static constexpr Mat4 orthographic(Float left, Float right, Float bottom, Float top, Float z_near, Float z_far) noexcept {
        Mat4 result = Mat4::identity();

        result(0, 0) = 2.0 / (right - left);
        result(1, 1) = 2.0 / (top - bottom);
        result(2, 2) = 1.0 / (z_far - z_near);
        result(0, 3) = -(right + left) / (right - left);
        result(1, 3) = -(top + bottom) / (top - bottom);
        result(2, 3) = -z_near / (z_far - z_near);

        return result;
    }

};


} // namespace pbpt