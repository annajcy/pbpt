#include <gtest/gtest.h>

#include "math/geometry/homogeneous.hpp"
#include "math/geometry/transform.hpp"
#include "math/global/function.hpp"

namespace pbpt::math::testing {

constexpr Float kEpsilon = 1e-6;

// A test fixture to hold common test data.
class TransformTest : public ::testing::Test {
protected:
    // A point on the X-axis
    const Pt3 p_x{10.0, 0.0, 0.0};
    // A point on the Y-axis
    const Pt3 p_y{0.0, 10.0, 0.0};
    // A vector pointing up
    const Vec3 v_up{0.0, 1.0, 0.0};
    // A point at the origin
    const Pt3 origin{0.0, 0.0, 0.0};
};

TEST_F(TransformTest, Translate) {
    const Vec3 translation_vec(5, -2, 100);
    const Mat4 t = Transform::translate(translation_vec).mat();

    const Pt3 p_transformed = (t * Homo3(origin)).to_point();
    EXPECT_DOUBLE_EQ(p_transformed.x(), 5.0);
    EXPECT_DOUBLE_EQ(p_transformed.y(), -2.0);
    EXPECT_DOUBLE_EQ(p_transformed.z(), 100.0);

    const Vec3 v_transformed = (t * Homo3(v_up)).to_vector();
    EXPECT_DOUBLE_EQ(v_transformed.x(), v_up.x());
    EXPECT_DOUBLE_EQ(v_transformed.y(), v_up.y());
    EXPECT_DOUBLE_EQ(v_transformed.z(), v_up.z());
}

TEST_F(TransformTest, Scale) {
    const Vec3 scale_vec(2, 3, 0.5);
    const Mat4 s = Transform::scale(scale_vec).mat();

    const Pt3 p_start(1, 1, 10);
    const Pt3 p_transformed = (s * Homo3(p_start)).to_point();
    EXPECT_DOUBLE_EQ(p_transformed.x(), 1.0 * 2.0);
    EXPECT_DOUBLE_EQ(p_transformed.y(), 1.0 * 3.0);
    EXPECT_DOUBLE_EQ(p_transformed.z(), 10.0 * 0.5);

    const Vec3 v_start(1, 1, 10);
    const Vec3 v_transformed = (s * Homo3(v_start)).to_vector();
    EXPECT_DOUBLE_EQ(v_transformed.x(), 1.0 * 2.0);
    EXPECT_DOUBLE_EQ(v_transformed.y(), 1.0 * 3.0);
    EXPECT_DOUBLE_EQ(v_transformed.z(), 10.0 * 0.5);
}

TEST_F(TransformTest, RotateZ90Degrees) {
    const Mat4 rz            = Transform::rotate_z(deg2rad(90.0)).mat();
    const Pt3  p_transformed = (rz * Homo3(p_x)).to_point();

    // It should end up on the Y-axis
    EXPECT_NEAR(p_transformed.x(), 0.0, kEpsilon);
    EXPECT_NEAR(p_transformed.y(), 10.0, kEpsilon);
    EXPECT_NEAR(p_transformed.z(), 0.0, kEpsilon);
}

TEST_F(TransformTest, RotateY90Degrees) {
    const Mat4 ry = Transform::rotate_y(deg2rad(90.0)).mat();

    // Rotate a point on the X-axis around Y
    const Pt3 p_transformed = (ry * Homo3(p_x)).to_point();

    // It should end up on the negative Z-axis
    EXPECT_NEAR(p_transformed.x(), 0.0, kEpsilon);
    EXPECT_NEAR(p_transformed.y(), 0.0, kEpsilon);
    EXPECT_NEAR(p_transformed.z(), -10.0, kEpsilon);
}

TEST_F(TransformTest, RotateX90Degrees) {
    const Mat4 rx = Transform::rotate_x(deg2rad(90.0)).mat();

    // Rotate a point on the Y-axis around X
    const Pt3 p_transformed = (rx * Homo3(p_y)).to_point();

    // It should end up on the Z-axis
    EXPECT_NEAR(p_transformed.x(), 0.0, kEpsilon);
    EXPECT_NEAR(p_transformed.y(), 0.0, kEpsilon);
    EXPECT_NEAR(p_transformed.z(), 10.0, kEpsilon);
}

TEST_F(TransformTest, RotateAxisAngle) {
    const auto angle = static_cast<Float>(M_PI / 2.0);  // 90 degrees
    const Vec3 z_axis(0, 0, 1);

    // An arbitrary axis-angle rotation should be identical to the specific-axis
    // version
    const Mat4 r_axis_angle = Transform::rotate(angle, z_axis).mat();
    const Mat4 r_z          = Transform::rotate_z(angle).mat();

    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            EXPECT_NEAR(r_axis_angle.at(r, c), r_z.at(r, c), kEpsilon);
        }
    }
}

TEST_F(TransformTest, LookAt) {
    const Pt3  eye(0, 0, 10);
    const Pt3  target(0, 0, 0);
    const Vec3 up(0, 1, 0);
    const Mat4 view_matrix = Transform::look_at(eye, target, up).mat();

    // A point in the world at (5, 0, 0)
    const Pt3 world_point(5, 0, 0);
    const Pt3 view_point = (view_matrix * Homo3(world_point)).to_point();

    // In view space, X should be 5, Y should be 0, Z should be -10 (10 units
    // away from camera looking down -Z)
    EXPECT_NEAR(view_point.x(), 5.0, kEpsilon);
    EXPECT_NEAR(view_point.y(), 0.0, kEpsilon);
    EXPECT_NEAR(view_point.z(), -10.0, kEpsilon);

    // The world origin should be at (0,0,-10) in view space
    const Pt3 origin_in_view = (view_matrix * Homo3(origin)).to_point();
    EXPECT_NEAR(origin_in_view.z(), -10.0, kEpsilon);
}

TEST_F(TransformTest, Orthographic) {
    const Mat4 ortho_matrix = Transform::orthographic(-10.0, 10.0, -5.0, 5.0, 1.0, 101.0).mat();

    // A point at the top-right-far corner of the view volume
    const Pt3 top_right_far(10, 5, 101);
    const Pt3 p1_ndc = (ortho_matrix * Homo3(top_right_far)).to_point();
    // Should map to (1, 1, 1) in Normalized Device Coordinates (NDC)
    EXPECT_NEAR(p1_ndc.x(), 1.0, kEpsilon);
    EXPECT_NEAR(p1_ndc.y(), 1.0, kEpsilon);
    EXPECT_NEAR(p1_ndc.z(), 1.0,
                kEpsilon);  // Far plane maps to 1 with this formula

    // A point at the bottom-left-near corner
    const Pt3 bottom_left_near(-10, -5, 1);
    const Pt3 p2_ndc = (ortho_matrix * Homo3(bottom_left_near)).to_point();
    // Should map to (-1, -1, 0) in NDC
    EXPECT_NEAR(p2_ndc.x(), -1.0, kEpsilon);
    EXPECT_NEAR(p2_ndc.y(), -1.0, kEpsilon);
    EXPECT_NEAR(p2_ndc.z(), 0.0,
                kEpsilon);  // Near plane maps to 0 with this formula
}

TEST_F(TransformTest, Perspective) {
    const Float fov_rad      = static_cast<Float>(M_PI / 2.0);  // 90 degrees
    const Float aspect       = 1.0;
    const Float z_near       = 1.0;
    const Float z_far        = 100.0;
    const Mat4  persp_matrix = Transform::perspective(fov_rad, aspect, z_near, z_far).mat();

    // Test a point on the near plane. Its Z should map to 0 in NDC.
    const Pt3   near_plane_point(0, 0, z_near);
    const Homo3 near_clip = persp_matrix * Homo3(near_plane_point);
    const Pt3   near_ndc  = near_clip.to_point();  // Perform perspective divide
    EXPECT_NEAR(near_ndc.z(), 0.0, kEpsilon);

    // Test a point on the far plane. Its Z should map to 1 in NDC.
    const Pt3   far_plane_point(50, 50, z_far);  // A point within the frustum on the far plane
    const Homo3 far_clip = persp_matrix * Homo3(far_plane_point);
    const Pt3   far_ndc  = far_clip.to_point();
    EXPECT_NEAR(far_ndc.z(), 1.0, kEpsilon);

    // Also check the w component before division for the far point
    EXPECT_NEAR(far_clip.raw().w(), z_far, kEpsilon);
}

}  // namespace pbpt::math::testing