#include <gtest/gtest.h>

#include "math/homogeneous.hpp"
#include "geometry/transform.hpp"
#include "math/normal.hpp"
#include "geometry/ray.hpp"
#include "geometry/bounds.hpp"
#include "math/vector.hpp"
#include "math/function.hpp"
#include "math/type_alias.hpp"

using namespace pbpt::geometry;

namespace pbpt::geometry::testing {

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
    const Mat4 t = Transform<double>::translate(translation_vec).matrix();

    const Pt3 p_transformed = (t * Homo3::from_point(origin)).to_point();
    EXPECT_DOUBLE_EQ(p_transformed.x(), 5.0);
    EXPECT_DOUBLE_EQ(p_transformed.y(), -2.0);
    EXPECT_DOUBLE_EQ(p_transformed.z(), 100.0);

    const Vec3 v_transformed = (t * Homo3::from_vector(v_up)).to_vector();
    EXPECT_DOUBLE_EQ(v_transformed.x(), v_up.x());
    EXPECT_DOUBLE_EQ(v_transformed.y(), v_up.y());
    EXPECT_DOUBLE_EQ(v_transformed.z(), v_up.z());
}

TEST_F(TransformTest, Scale) {
    const Vec3 scale_vec(2, 3, 0.5);
    const Mat4 s = Transform<double>::scale(scale_vec).matrix();

    const Pt3 p_start(1, 1, 10);
    const Pt3 p_transformed = (s * Homo3::from_point(p_start)).to_point();
    EXPECT_DOUBLE_EQ(p_transformed.x(), 1.0 * 2.0);
    EXPECT_DOUBLE_EQ(p_transformed.y(), 1.0 * 3.0);
    EXPECT_DOUBLE_EQ(p_transformed.z(), 10.0 * 0.5);

    const Vec3 v_start(1, 1, 10);
    const Vec3 v_transformed = (s * Homo3::from_vector(v_start)).to_vector();
    EXPECT_DOUBLE_EQ(v_transformed.x(), 1.0 * 2.0);
    EXPECT_DOUBLE_EQ(v_transformed.y(), 1.0 * 3.0);
    EXPECT_DOUBLE_EQ(v_transformed.z(), 10.0 * 0.5);
}

TEST_F(TransformTest, RotateZ90Degrees) {
    const Mat4 rz            = Transform<double>::rotate_z(deg2rad(90.0)).matrix();
    const Pt3  p_transformed = (rz * Homo3::from_point(p_x)).to_point();

    // It should end up on the Y-axis
    EXPECT_NEAR(p_transformed.x(), 0.0, kEpsilon);
    EXPECT_NEAR(p_transformed.y(), 10.0, kEpsilon);
    EXPECT_NEAR(p_transformed.z(), 0.0, kEpsilon);
}

TEST_F(TransformTest, RotateY90Degrees) {
    const Mat4 ry = Transform<double>::rotate_y(deg2rad(90.0)).matrix();

    // Rotate a point on the X-axis around Y
    const Pt3 p_transformed = (ry * Homo3::from_point(p_x)).to_point();

    // It should end up on the negative Z-axis
    EXPECT_NEAR(p_transformed.x(), 0.0, kEpsilon);
    EXPECT_NEAR(p_transformed.y(), 0.0, kEpsilon);
    EXPECT_NEAR(p_transformed.z(), -10.0, kEpsilon);
}

TEST_F(TransformTest, RotateX90Degrees) {
    const Mat4 rx = Transform<double>::rotate_x(deg2rad(90.0)).matrix();

    // Rotate a point on the Y-axis around X
    const Pt3 p_transformed = (rx * Homo3::from_point(p_y)).to_point();

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
    const Mat4 r_axis_angle = Transform<double>::rotate(angle, z_axis).matrix();
    const Mat4 r_z          = Transform<double>::rotate_z(angle).matrix();

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
    const Mat4 view_matrix = Transform<double>::look_at(eye, target, up).matrix();

    // A point in the world at (5, 0, 0)
    const Pt3 world_point(5, 0, 0);
    const Pt3 view_point = (view_matrix * Homo3::from_point(world_point)).to_point();

    // In view space, X should be 5, Y should be 0, Z should be -10 (10 units
    // away from camera looking down -Z)
    EXPECT_NEAR(view_point.x(), 5.0, kEpsilon);
    EXPECT_NEAR(view_point.y(), 0.0, kEpsilon);
    EXPECT_NEAR(view_point.z(), -10.0, kEpsilon);

    // The world origin should be at (0,0,-10) in view space
    const Pt3 origin_in_view = (view_matrix * Homo3::from_point(origin)).to_point();
    EXPECT_NEAR(origin_in_view.z(), -10.0, kEpsilon);
}

TEST_F(TransformTest, Orthographic) {
    const Mat4 ortho_matrix = Transform<double>::orthographic(-10.0, 10.0, -5.0, 5.0, 1.0, 101.0).matrix();

    // A point at the top-right-far corner of the view volume
    const Pt3 top_right_far(10, 5, 101);
    const Pt3 p1_ndc = (ortho_matrix * Homo3::from_point(top_right_far)).to_point();
    // Should map to (1, 1, 1) in Normalized Device Coordinates (NDC)
    EXPECT_NEAR(p1_ndc.x(), 1.0, kEpsilon);
    EXPECT_NEAR(p1_ndc.y(), 1.0, kEpsilon);
    EXPECT_NEAR(p1_ndc.z(), 1.0,
                kEpsilon);  // Far plane maps to 1 with this formula

    // A point at the bottom-left-near corner
    const Pt3 bottom_left_near(-10, -5, 1);
    const Pt3 p2_ndc = (ortho_matrix * Homo3::from_point(bottom_left_near)).to_point();
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
    const Mat4  persp_matrix = Transform<double>::perspective(fov_rad, aspect, z_near, z_far).matrix();

    // Test a point on the near plane. Its Z should map to 0 in NDC.
    const Pt3   near_plane_point(0, 0, z_near);
    const Homo3 near_clip = persp_matrix * Homo3::from_point(near_plane_point);
    const Pt3   near_ndc  = near_clip.to_point();  // Perform perspective divide
    EXPECT_NEAR(near_ndc.z(), 0.0, kEpsilon);

    // Test a point on the far plane. Its Z should map to 1 in NDC.
    const Pt3   far_plane_point(50, 50, z_far);  // A point within the frustum on the far plane
    const Homo3 far_clip = persp_matrix * Homo3::from_point(far_plane_point);
    const Pt3   far_ndc  = far_clip.to_point();
    EXPECT_NEAR(far_ndc.z(), 1.0, kEpsilon);

    // Also check the w component before division for the far point
    EXPECT_NEAR(far_clip.to_vector_raw().w(), z_far, kEpsilon);
}

// --- Identity Transform Tests ---
TEST_F(TransformTest, Identity) {
    Transform<double> identity_transform;
    EXPECT_TRUE(identity_transform.is_identity());

    // Identity transform should not change points
    const Pt3 test_point(3.5, -2.1, 7.8);
    const Pt3 transformed = identity_transform * test_point;
    EXPECT_DOUBLE_EQ(transformed.x(), test_point.x());
    EXPECT_DOUBLE_EQ(transformed.y(), test_point.y());
    EXPECT_DOUBLE_EQ(transformed.z(), test_point.z());

    // Identity transform should not change vectors
    const Vec3 test_vector(1.2, -4.5, 0.3);
    const Vec3 transformed_vec = identity_transform * test_vector;
    EXPECT_DOUBLE_EQ(transformed_vec.x(), test_vector.x());
    EXPECT_DOUBLE_EQ(transformed_vec.y(), test_vector.y());
    EXPECT_DOUBLE_EQ(transformed_vec.z(), test_vector.z());
}

// --- Transform Composition Tests ---
TEST_F(TransformTest, Composition) {
    // Test T * S * R (translate after scale after rotate)
    const Vec3 translation(5, 3, -2);
    const Vec3 scale_factors(2, 0.5, 3);
    const Float rotation_angle = deg2rad(45.0);
    
    Transform<double> T = Transform<double>::translate(translation);
    Transform<double> S = Transform<double>::scale(scale_factors);
    Transform<double> R = Transform<double>::rotate_z(rotation_angle);
    
    Transform<double> combined = T * S * R;
    
    // Test with a known point
    const Pt3 original(1, 0, 0);
    
    // Apply transformations step by step
    Pt3 after_rotate = R * original;
    Pt3 after_scale = S * after_rotate;
    Pt3 after_translate = T * after_scale;
    
    // Should be same as combined transform
    Pt3 combined_result = combined * original;
    
    EXPECT_NEAR(combined_result.x(), after_translate.x(), kEpsilon);
    EXPECT_NEAR(combined_result.y(), after_translate.y(), kEpsilon);
    EXPECT_NEAR(combined_result.z(), after_translate.z(), kEpsilon);
}

// --- Inverse Transform Tests ---
TEST_F(TransformTest, InverseTransform) {
    const Vec3 translation(3, -1, 5);
    Transform<double> T = Transform<double>::translate(translation);
    Transform<double> T_inv = T.inversed();
    
    // T * T_inv should be identity
    Transform<double> should_be_identity = T * T_inv;
    EXPECT_TRUE(should_be_identity.is_identity());
    
    // Test that inverse actually inverts the transformation
    const Pt3 test_point(2, 4, -1);
    Pt3 transformed = T * test_point;
    Pt3 back_to_original = T_inv * transformed;
    
    EXPECT_NEAR(back_to_original.x(), test_point.x(), kEpsilon);
    EXPECT_NEAR(back_to_original.y(), test_point.y(), kEpsilon);
    EXPECT_NEAR(back_to_original.z(), test_point.z(), kEpsilon);
}

// --- Scale Detection Tests ---
TEST_F(TransformTest, HasScale) {
    // Uniform scale should be detected
    Transform<double> uniform_scale = Transform<double>::scale(2.0);
    EXPECT_TRUE(uniform_scale.has_scale());
    
    // Non-uniform scale should be detected
    Transform<double> non_uniform_scale = Transform<double>::scale(Vec3(1.5, 2.0, 0.8));
    EXPECT_TRUE(non_uniform_scale.has_scale());
    
    // Identity should have no scale
    Transform<double> identity;
    EXPECT_FALSE(identity.has_scale());
    
    // Translation should have no scale
    Transform<double> translation = Transform<double>::translate(Vec3(1, 2, 3));
    EXPECT_FALSE(translation.has_scale());
    
    // Rotation should have no scale
    Transform<double> rotation = Transform<double>::rotate_x(deg2rad(30.0));
    EXPECT_FALSE(rotation.has_scale());
}

// --- Normal Transformation Tests ---
TEST_F(TransformTest, NormalTransform) {
    // Create a non-uniform scale transform
    const Vec3 scale_factors(2.0, 0.5, 1.0);
    Transform<double> S = Transform<double>::scale(scale_factors);
    
    // Create a normal vector pointing up
    Normal3 normal = Normal3::from_vector(Vec3(0, 1, 0));
    
    // Transform the normal
    Normal3 transformed_normal = S * normal;
    
    // Normal transformation uses the inverse transpose, so we expect:
    // For scaling (sx, sy, sz), the normal transform matrix should be (1/sx, 1/sy, 1/sz)
    // So (0, 1, 0) should become (0, 1/0.5, 0) = (0, 2, 0)
    EXPECT_NEAR(transformed_normal.to_vector().x(), 0.0, kEpsilon);
    EXPECT_NEAR(transformed_normal.to_vector().y(), 2.0, kEpsilon);
    EXPECT_NEAR(transformed_normal.to_vector().z(), 0.0, kEpsilon);
}

// --- Ray Transformation Tests ---
TEST_F(TransformTest, RayTransform) {
    const Vec3 translation(1, 2, 3);
    Transform<double> T = Transform<double>::translate(translation);
    
    // Create a ray from origin pointing in +X direction
    Ray3 ray(Pt3(0, 0, 0), Vec3(1, 0, 0));
    
    // Transform the ray
    Ray3 transformed_ray = T * ray;
    
    // Origin should be translated
    EXPECT_NEAR(transformed_ray.origin().x(), 1.0, kEpsilon);
    EXPECT_NEAR(transformed_ray.origin().y(), 2.0, kEpsilon);
    EXPECT_NEAR(transformed_ray.origin().z(), 3.0, kEpsilon);
    
    // Direction should remain the same (translation doesn't affect direction)
    EXPECT_NEAR(transformed_ray.direction().x(), 1.0, kEpsilon);
    EXPECT_NEAR(transformed_ray.direction().y(), 0.0, kEpsilon);
    EXPECT_NEAR(transformed_ray.direction().z(), 0.0, kEpsilon);
    
    // Test with rotation
    Transform<double> R = Transform<double>::rotate_z(deg2rad(90.0));
    Ray3 rotated_ray = R * ray;
    
    // Direction should be rotated from (1,0,0) to (0,1,0)
    EXPECT_NEAR(rotated_ray.direction().x(), 0.0, kEpsilon);
    EXPECT_NEAR(rotated_ray.direction().y(), 1.0, kEpsilon);
    EXPECT_NEAR(rotated_ray.direction().z(), 0.0, kEpsilon);
}

// --- Bounds Transformation Tests ---
TEST_F(TransformTest, BoundsTransform) {
    // Create a unit cube bounds
    Point<double, 3> min_pt(0, 0, 0);
    Point<double, 3> max_pt(1, 1, 1);
    Bounds<double, 3> unit_cube(min_pt, max_pt);
    
    // Scale it by 2
    Transform<double> S = Transform<double>::scale(2.0);
    auto scaled_bounds = S * unit_cube;
    
    // The bounds should now be from (0,0,0) to (2,2,2)
    EXPECT_NEAR(scaled_bounds.min().x(), 0.0, kEpsilon);
    EXPECT_NEAR(scaled_bounds.min().y(), 0.0, kEpsilon);
    EXPECT_NEAR(scaled_bounds.min().z(), 0.0, kEpsilon);
    EXPECT_NEAR(scaled_bounds.max().x(), 2.0, kEpsilon);
    EXPECT_NEAR(scaled_bounds.max().y(), 2.0, kEpsilon);
    EXPECT_NEAR(scaled_bounds.max().z(), 2.0, kEpsilon);
    
    // Translate the bounds
    const Vec3 translation(1, 1, 1);
    Transform<double> T = Transform<double>::translate(translation);
    auto translated_bounds = T * unit_cube;
    
    // The bounds should now be from (1,1,1) to (2,2,2)
    EXPECT_NEAR(translated_bounds.min().x(), 1.0, kEpsilon);
    EXPECT_NEAR(translated_bounds.min().y(), 1.0, kEpsilon);
    EXPECT_NEAR(translated_bounds.min().z(), 1.0, kEpsilon);
    EXPECT_NEAR(translated_bounds.max().x(), 2.0, kEpsilon);
    EXPECT_NEAR(translated_bounds.max().y(), 2.0, kEpsilon);
    EXPECT_NEAR(translated_bounds.max().z(), 2.0, kEpsilon);
}

// --- Matrix Access Tests ---
TEST_F(TransformTest, MatrixAccess) {
    const Vec3 translation(1, 2, 3);
    Transform<double> T = Transform<double>::translate(translation);
    
    // Check that we can access the matrix
    const Mat4& matrix = T.matrix();
    EXPECT_DOUBLE_EQ(matrix.at(0, 3), 1.0);  // Translation X
    EXPECT_DOUBLE_EQ(matrix.at(1, 3), 2.0);  // Translation Y
    EXPECT_DOUBLE_EQ(matrix.at(2, 3), 3.0);  // Translation Z
    
    // Check that we can access the inverse matrix
    const Mat4& inv_matrix = T.inverse_matrix();
    EXPECT_DOUBLE_EQ(inv_matrix.at(0, 3), -1.0);  // Inverse translation X
    EXPECT_DOUBLE_EQ(inv_matrix.at(1, 3), -2.0);  // Inverse translation Y
    EXPECT_DOUBLE_EQ(inv_matrix.at(2, 3), -3.0);  // Inverse translation Z
}

// --- Setting Matrix Tests ---
TEST_F(TransformTest, SetMatrix) {
    Transform<double> T;
    
    // Create a custom matrix (translation by (5, 6, 7))
    Mat4 custom_matrix(
        1.0, 0.0, 0.0, 5.0,
        0.0, 1.0, 0.0, 6.0,
        0.0, 0.0, 1.0, 7.0,
        0.0, 0.0, 0.0, 1.0
    );
    
    T.set_matrix(custom_matrix);
    
    // Test that the transform works correctly
    const Pt3 test_point(1, 1, 1);
    const Pt3 transformed = T * test_point;
    
    EXPECT_NEAR(transformed.x(), 6.0, kEpsilon);
    EXPECT_NEAR(transformed.y(), 7.0, kEpsilon);
    EXPECT_NEAR(transformed.z(), 8.0, kEpsilon);
}

// --- Uniform Scale Tests ---
TEST_F(TransformTest, UniformScale) {
    const Float scale_factor = 3.0;
    Transform<double> S = Transform<double>::scale(scale_factor);
    
    const Pt3 test_point(1, 2, 3);
    const Pt3 transformed = S * test_point;
    
    EXPECT_NEAR(transformed.x(), 3.0, kEpsilon);
    EXPECT_NEAR(transformed.y(), 6.0, kEpsilon);
    EXPECT_NEAR(transformed.z(), 9.0, kEpsilon);
}

// --- Axis-Angle Rotation with Different Axes ---
TEST_F(TransformTest, AxisAngleRotationDifferentAxes) {
    const Float angle = static_cast<Float>(M_PI / 4.0);  // 45 degrees
    
    // Test rotation around X axis
    const Vec3 x_axis(1, 0, 0);
    Transform<double> Rx_axis = Transform<double>::rotate(angle, x_axis);
    Transform<double> Rx_func = Transform<double>::rotate_x(angle);
    
    // Should be equivalent to rotate_x
    const Pt3 test_point(0, 1, 0);
    Pt3 result_axis = Rx_axis * test_point;
    Pt3 result_func = Rx_func * test_point;
    
    EXPECT_NEAR(result_axis.x(), result_func.x(), kEpsilon);
    EXPECT_NEAR(result_axis.y(), result_func.y(), kEpsilon);
    EXPECT_NEAR(result_axis.z(), result_func.z(), kEpsilon);
    
    // Test rotation around Y axis
    const Vec3 y_axis(0, 1, 0);
    Transform<double> Ry_axis = Transform<double>::rotate(angle, y_axis);
    Transform<double> Ry_func = Transform<double>::rotate_y(angle);
    
    // Should be equivalent to rotate_y
    const Pt3 test_point2(1, 0, 0);
    Pt3 result_axis2 = Ry_axis * test_point2;
    Pt3 result_func2 = Ry_func * test_point2;
    
    EXPECT_NEAR(result_axis2.x(), result_func2.x(), kEpsilon);
    EXPECT_NEAR(result_axis2.y(), result_func2.y(), kEpsilon);
    EXPECT_NEAR(result_axis2.z(), result_func2.z(), kEpsilon);
}

// --- Look At Advanced Tests ---
TEST_F(TransformTest, LookAtAdvanced) {
    // Test 1: Standard camera setup - eye at (5,5,5) looking at origin
    const Pt3 eye1(5, 5, 5);
    const Pt3 target1(0, 0, 0);
    const Vec3 up1(0, 1, 0);
    
    Transform<double> view1 = Transform<double>::look_at(eye1, target1, up1);
    
    // The target should be in front of the camera (negative Z in view space)
    const Pt3 target_in_view1 = view1 * target1;
    EXPECT_LT(target_in_view1.z(), 0.0);
    
    // The eye position should be at origin in view space
    const Pt3 eye_in_view1 = view1 * eye1;
    EXPECT_NEAR(eye_in_view1.x(), 0.0, kEpsilon);
    EXPECT_NEAR(eye_in_view1.y(), 0.0, kEpsilon);
    EXPECT_NEAR(eye_in_view1.z(), 0.0, kEpsilon);
    
    // Test 2: Right-handed coordinate system verification
    // A point to the right of the camera should have positive X in view space
    const Pt3 eye2(0, 0, 5);
    const Pt3 target2(0, 0, 0);
    const Vec3 up2(0, 1, 0);
    
    Transform<double> view2 = Transform<double>::look_at(eye2, target2, up2);
    
    // Point to the right of camera (positive X in world)
    const Pt3 right_point(1, 0, 0);
    const Pt3 right_in_view = view2 * right_point;
    EXPECT_GT(right_in_view.x(), 0.0);  // Should be positive X in view
    EXPECT_NEAR(right_in_view.y(), 0.0, kEpsilon);  // Same height as camera
    
    // Point above camera (positive Y in world)  
    const Pt3 up_point(0, 1, 0);
    const Pt3 up_in_view = view2 * up_point;
    EXPECT_NEAR(up_in_view.x(), 0.0, kEpsilon);  // Centered horizontally
    EXPECT_GT(up_in_view.y(), 0.0);  // Should be positive Y in view
    
    // Test 3: Direction vectors should transform correctly
    // Forward direction (-Z in world for this setup) should become -Z in view
    const Vec3 forward_world(0, 0, -1);
    const Vec3 forward_view = view2 * forward_world;
    EXPECT_NEAR(forward_view.x(), 0.0, kEpsilon);
    EXPECT_NEAR(forward_view.y(), 0.0, kEpsilon);
    EXPECT_LT(forward_view.z(), 0.0);  // Forward should be negative Z
    
    // Test 4: Orthogonality check - view matrix should preserve orthogonality
    const Vec3 right_dir(1, 0, 0);
    const Vec3 up_dir(0, 1, 0);
    const Vec3 forward_dir(0, 0, -1);
    
    const Vec3 right_transformed = view2 * right_dir;
    const Vec3 up_transformed = view2 * up_dir;
    const Vec3 forward_transformed = view2 * forward_dir;
    
    // Check that transformed vectors are still orthogonal
    EXPECT_NEAR(right_transformed.dot(up_transformed), 0.0, kEpsilon);
    EXPECT_NEAR(right_transformed.dot(forward_transformed), 0.0, kEpsilon);
    EXPECT_NEAR(up_transformed.dot(forward_transformed), 0.0, kEpsilon);
    
    // And they should be unit vectors (preserving length)
    EXPECT_NEAR(right_transformed.length(), 1.0, kEpsilon);
    EXPECT_NEAR(up_transformed.length(), 1.0, kEpsilon);
    EXPECT_NEAR(forward_transformed.length(), 1.0, kEpsilon);
}

// --- Transpose Tests ---
TEST_F(TransformTest, Transpose) {
    const Vec3 translation(1, 2, 3);
    Transform<double> T = Transform<double>::translate(translation);
    Transform<double> T_transposed = T.transposed();
    
    // For a translation matrix, transpose should change the structure
    // Original: translation in last column
    // Transposed: translation in last row
    const Mat4& orig_matrix = T.matrix();
    const Mat4& trans_matrix = T_transposed.matrix();
    
    EXPECT_DOUBLE_EQ(orig_matrix.at(0, 3), trans_matrix.at(3, 0));
    EXPECT_DOUBLE_EQ(orig_matrix.at(1, 3), trans_matrix.at(3, 1));
    EXPECT_DOUBLE_EQ(orig_matrix.at(2, 3), trans_matrix.at(3, 2));
}

// --- Multiple Type Support Tests ---
TEST_F(TransformTest, MixedTypes) {
    // Test float and double compatibility with simple transformations
    Transform<double> T = Transform<double>::translate(Vec3{1.0, 2.0, 3.0});
    
    // Test point transformation with mixed types
    Point<float, 3> pf{1.0f, 1.0f, 1.0f};
    auto result = T * pf;  // Should create Point<double, 3>
    
    EXPECT_NEAR(result.x(), 2.0, kEpsilon);  // 1+1 = 2
    EXPECT_NEAR(result.y(), 3.0, kEpsilon);  // 1+2 = 3  
    EXPECT_NEAR(result.z(), 4.0, kEpsilon);  // 1+3 = 4
    
    // Test vector transformation
    Vector<float, 3> vf{1.0f, 0.0f, 0.0f};
    auto vec_result = T * vf;  // Should create Vector<double, 3>
    
    // Translation shouldn't affect vectors
    EXPECT_NEAR(vec_result.x(), 1.0, kEpsilon);
    EXPECT_NEAR(vec_result.y(), 0.0, kEpsilon);
    EXPECT_NEAR(vec_result.z(), 0.0, kEpsilon);
}

}  // namespace pbpt::math::testing