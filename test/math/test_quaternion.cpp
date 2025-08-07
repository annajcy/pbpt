#include <gtest/gtest.h>
#include <cmath>
#include <sstream>

#include "math/geometry/quaternion.hpp"
#include "math/geometry/vector.hpp"
#include "math/global/function.hpp"

namespace pbpt::math::testing {

using namespace pbpt::math;

bool quat_almost_equal(const Quat& a, const Quat& b, Float epsilon = epsilon_v<Float>) {
    return abs(a.w() - b.w()) < epsilon &&
           abs(a.x() - b.x()) < epsilon &&
           abs(a.y() - b.y()) < epsilon &&
           abs(a.z() - b.z()) < epsilon;
}

bool vec_almost_equal(const Vec3& a, const Vec3& b, Float epsilon = epsilon_v<Float>) {
    return abs(a.x() - b.x()) < epsilon &&
           abs(a.y() - b.y()) < epsilon &&
           abs(a.z() - b.z()) < epsilon;
}

// ========== Construction Tests ==========

TEST(QuaternionTest, DefaultConstruction) {
    Quat q;
    EXPECT_FLOAT_EQ(q.w(), 1.0f);
    EXPECT_FLOAT_EQ(q.x(), 0.0f);
    EXPECT_FLOAT_EQ(q.y(), 0.0f);
    EXPECT_FLOAT_EQ(q.z(), 0.0f);
    EXPECT_TRUE(q.is_identity());
}

TEST(QuaternionTest, ComponentConstruction) {
    Quat q(0.5f, 0.5f, 0.5f, 0.5f);
    EXPECT_FLOAT_EQ(q.w(), 0.5f);
    EXPECT_FLOAT_EQ(q.x(), 0.5f);
    EXPECT_FLOAT_EQ(q.y(), 0.5f);
    EXPECT_FLOAT_EQ(q.z(), 0.5f);
}

TEST(QuaternionTest, ScalarVectorConstruction) {
    Vec3 v(1.0f, 2.0f, 3.0f);
    Quat q(0.5f, v);
    EXPECT_FLOAT_EQ(q.w(), 0.5f);
    EXPECT_FLOAT_EQ(q.x(), 1.0f);
    EXPECT_FLOAT_EQ(q.y(), 2.0f);
    EXPECT_FLOAT_EQ(q.z(), 3.0f);
}

TEST(QuaternionTest, AxisAngleConstruction) {
    Vec3 axis(0.0f, 0.0f, 1.0f);  // Z-axis
    Float angle = M_PI / 2;       // 90 degrees
    Quat q(axis, angle);
    
    // Should be approximately (cos(45°), 0, 0, sin(45°))
    Float expected_w = std::cos(angle / 2);
    Float expected_z = std::sin(angle / 2);
    
    EXPECT_TRUE(abs(q.w() - expected_w) < epsilon_v<Float>);
    EXPECT_TRUE(abs(q.x()) < epsilon_v<Float>);
    EXPECT_TRUE(abs(q.y()) < epsilon_v<Float>);
    EXPECT_TRUE(abs(q.z() - expected_z) < epsilon_v<Float>);
}

TEST(QuaternionTest, VectorToVectorConstruction) {
    Vec3 from(1.0f, 0.0f, 0.0f);  // X-axis
    Vec3 to(0.0f, 1.0f, 0.0f);    // Y-axis
    Quat q(from, to);
    
    // Rotate from vector by quaternion should give to vector
    Vec3 rotated = q.rotate(from);
    EXPECT_TRUE(vec_almost_equal(rotated, to, 1e-5f));
}

TEST(QuaternionTest, IdentityFactory) {
    Quat q = Quat::identity();
    EXPECT_TRUE(q.is_identity());
    EXPECT_FLOAT_EQ(q.w(), 1.0f);
    EXPECT_FLOAT_EQ(q.x(), 0.0f);
    EXPECT_FLOAT_EQ(q.y(), 0.0f);
    EXPECT_FLOAT_EQ(q.z(), 0.0f);
}

TEST(QuaternionTest, EulerConstruction) {
    Float roll = 0.1f, pitch = 0.2f, yaw = 0.3f;
    Quat q = Quat::from_euler(roll, pitch, yaw);
    
    // Convert back to Euler and check
    Vec3 euler = q.to_euler();
    EXPECT_TRUE(abs(euler.x() - roll) < 1e-5f);
    EXPECT_TRUE(abs(euler.y() - pitch) < 1e-5f);
    EXPECT_TRUE(abs(euler.z() - yaw) < 1e-5f);
}

// ========== Accessor Tests ==========

TEST(QuaternionTest, Accessors) {
    Quat q(1.0f, 2.0f, 3.0f, 4.0f);
    
    EXPECT_FLOAT_EQ(q.w(), 1.0f);
    EXPECT_FLOAT_EQ(q.x(), 2.0f);
    EXPECT_FLOAT_EQ(q.y(), 3.0f);
    EXPECT_FLOAT_EQ(q.z(), 4.0f);
    
    EXPECT_FLOAT_EQ(q.scalar(), 1.0f);
    Vec3 v = q.vector();
    EXPECT_FLOAT_EQ(v.x(), 2.0f);
    EXPECT_FLOAT_EQ(v.y(), 3.0f);
    EXPECT_FLOAT_EQ(v.z(), 4.0f);
}

TEST(QuaternionTest, ArrayAccess) {
    Quat q(1.0f, 2.0f, 3.0f, 4.0f);
    
    EXPECT_FLOAT_EQ(q[0], 1.0f);  // w
    EXPECT_FLOAT_EQ(q[1], 2.0f);  // x
    EXPECT_FLOAT_EQ(q[2], 3.0f);  // y
    EXPECT_FLOAT_EQ(q[3], 4.0f);  // z
    
    q[0] = 5.0f;
    EXPECT_FLOAT_EQ(q.w(), 5.0f);
}

// ========== Property Tests ==========

TEST(QuaternionTest, Length) {
    Quat q(1.0f, 2.0f, 3.0f, 4.0f);
    Float expected_length = std::sqrt(1 + 4 + 9 + 16);
    EXPECT_TRUE(abs(q.length() - expected_length) < epsilon_v<Float>);
    EXPECT_TRUE(abs(q.length_squared() - 30.0f) < epsilon_v<Float>);
}

TEST(QuaternionTest, Normalization) {
    Quat q(1.0f, 2.0f, 3.0f, 4.0f);
    Quat normalized = q.normalized();
    
    EXPECT_TRUE(normalized.is_normalized());
    EXPECT_TRUE(abs(normalized.length() - 1.0f) < epsilon_v<Float>);
    
    // Test in-place normalization
    q.normalize();
    EXPECT_TRUE(q.is_normalized());
}

TEST(QuaternionTest, IsIdentity) {
    EXPECT_TRUE(Quat::identity().is_identity());
    EXPECT_FALSE(Quat(0.5f, 0.5f, 0.5f, 0.5f).is_identity());
}

// ========== Operation Tests ==========

TEST(QuaternionTest, Conjugate) {
    Quat q(1.0f, 2.0f, 3.0f, 4.0f);
    Quat conj = q.conjugate();
    
    EXPECT_FLOAT_EQ(conj.w(), 1.0f);
    EXPECT_FLOAT_EQ(conj.x(), -2.0f);
    EXPECT_FLOAT_EQ(conj.y(), -3.0f);
    EXPECT_FLOAT_EQ(conj.z(), -4.0f);
}

TEST(QuaternionTest, Inverse) {
    Quat q(1.0f, 0.0f, 0.0f, 0.0f);  // Identity
    Quat inv = q.inverse();
    EXPECT_TRUE(quat_almost_equal(inv, q));
    
    // Test q * q^-1 = identity
    Quat q2(0.5f, 0.5f, 0.5f, 0.5f);
    Quat inv2 = q2.inverse();
    Quat product = q2 * inv2;
    EXPECT_TRUE(product.is_identity(1e-5f));
}

TEST(QuaternionTest, DotProduct) {
    Quat q1(1.0f, 2.0f, 3.0f, 4.0f);
    Quat q2(5.0f, 6.0f, 7.0f, 8.0f);
    Float dot = q1.dot(q2);
    Float expected = 1*5 + 2*6 + 3*7 + 4*8;  // 70
    EXPECT_FLOAT_EQ(dot, expected);
}

// ========== Rotation Tests ==========

TEST(QuaternionTest, RotateVector) {
    // 90-degree rotation around Z-axis
    Vec3 axis(0.0f, 0.0f, 1.0f);
    Float angle = M_PI / 2;
    Quat q(axis, angle);
    
    Vec3 v(1.0f, 0.0f, 0.0f);  // X-axis
    Vec3 rotated = q.rotate(v);
    Vec3 expected(0.0f, 1.0f, 0.0f);  // Should become Y-axis
    
    EXPECT_TRUE(vec_almost_equal(rotated, expected, 1e-5f));
}

TEST(QuaternionTest, AxisAngleConversion) {
    Vec3 original_axis(1.0f, 2.0f, 3.0f);
    original_axis = original_axis.normalized();
    Float original_angle = 1.5f;
    
    Quat q(original_axis, original_angle);
    auto [axis, angle] = q.to_axis_angle();
    
    EXPECT_TRUE(vec_almost_equal(axis, original_axis, 1e-5f));
    EXPECT_TRUE(abs(angle - original_angle) < 1e-5f);
}

// ========== Arithmetic Tests ==========

TEST(QuaternionTest, Addition) {
    Quat q1(1.0f, 2.0f, 3.0f, 4.0f);
    Quat q2(5.0f, 6.0f, 7.0f, 8.0f);
    Quat result = q1 + q2;
    
    EXPECT_FLOAT_EQ(result.w(), 6.0f);
    EXPECT_FLOAT_EQ(result.x(), 8.0f);
    EXPECT_FLOAT_EQ(result.y(), 10.0f);
    EXPECT_FLOAT_EQ(result.z(), 12.0f);
}

TEST(QuaternionTest, Subtraction) {
    Quat q1(5.0f, 6.0f, 7.0f, 8.0f);
    Quat q2(1.0f, 2.0f, 3.0f, 4.0f);
    Quat result = q1 - q2;
    
    EXPECT_FLOAT_EQ(result.w(), 4.0f);
    EXPECT_FLOAT_EQ(result.x(), 4.0f);
    EXPECT_FLOAT_EQ(result.y(), 4.0f);
    EXPECT_FLOAT_EQ(result.z(), 4.0f);
}

TEST(QuaternionTest, Negation) {
    Quat q(1.0f, 2.0f, 3.0f, 4.0f);
    Quat neg = -q;
    
    EXPECT_FLOAT_EQ(neg.w(), -1.0f);
    EXPECT_FLOAT_EQ(neg.x(), -2.0f);
    EXPECT_FLOAT_EQ(neg.y(), -3.0f);
    EXPECT_FLOAT_EQ(neg.z(), -4.0f);
}

TEST(QuaternionTest, QuaternionMultiplication) {
    // Test quaternion multiplication (Hamilton product)
    Quat q1(1.0f, 0.0f, 0.0f, 0.0f);  // Identity
    Quat q2(0.0f, 1.0f, 0.0f, 0.0f);  // i
    Quat result = q1 * q2;
    
    EXPECT_TRUE(quat_almost_equal(result, q2));
    
    // Test i * j = k
    Quat i(0.0f, 1.0f, 0.0f, 0.0f);
    Quat j(0.0f, 0.0f, 1.0f, 0.0f);
    Quat k(0.0f, 0.0f, 0.0f, 1.0f);
    Quat ij = i * j;
    
    EXPECT_TRUE(quat_almost_equal(ij, k));
}

TEST(QuaternionTest, ScalarMultiplication) {
    Quat q(1.0f, 2.0f, 3.0f, 4.0f);
    Quat result = q * 2.0f;
    
    EXPECT_FLOAT_EQ(result.w(), 2.0f);
    EXPECT_FLOAT_EQ(result.x(), 4.0f);
    EXPECT_FLOAT_EQ(result.y(), 6.0f);
    EXPECT_FLOAT_EQ(result.z(), 8.0f);
    
    // Test commutative property
    Quat result2 = 2.0f * q;
    EXPECT_TRUE(quat_almost_equal(result, result2));
}

TEST(QuaternionTest, ScalarDivision) {
    Quat q(2.0f, 4.0f, 6.0f, 8.0f);
    Quat result = q / 2.0f;
    
    EXPECT_FLOAT_EQ(result.w(), 1.0f);
    EXPECT_FLOAT_EQ(result.x(), 2.0f);
    EXPECT_FLOAT_EQ(result.y(), 3.0f);
    EXPECT_FLOAT_EQ(result.z(), 4.0f);
}

// ========== Assignment Tests ==========

TEST(QuaternionTest, CompoundAssignment) {
    Quat q1(1.0f, 2.0f, 3.0f, 4.0f);
    Quat q2(1.0f, 1.0f, 1.0f, 1.0f);
    
    q1 += q2;
    EXPECT_FLOAT_EQ(q1.w(), 2.0f);
    EXPECT_FLOAT_EQ(q1.x(), 3.0f);
    EXPECT_FLOAT_EQ(q1.y(), 4.0f);
    EXPECT_FLOAT_EQ(q1.z(), 5.0f);
    
    q1 -= q2;
    EXPECT_FLOAT_EQ(q1.w(), 1.0f);
    EXPECT_FLOAT_EQ(q1.x(), 2.0f);
    EXPECT_FLOAT_EQ(q1.y(), 3.0f);
    EXPECT_FLOAT_EQ(q1.z(), 4.0f);
    
    q1 *= 2.0f;
    EXPECT_FLOAT_EQ(q1.w(), 2.0f);
    EXPECT_FLOAT_EQ(q1.x(), 4.0f);
    EXPECT_FLOAT_EQ(q1.y(), 6.0f);
    EXPECT_FLOAT_EQ(q1.z(), 8.0f);
    
    q1 /= 2.0f;
    EXPECT_FLOAT_EQ(q1.w(), 1.0f);
    EXPECT_FLOAT_EQ(q1.x(), 2.0f);
    EXPECT_FLOAT_EQ(q1.y(), 3.0f);
    EXPECT_FLOAT_EQ(q1.z(), 4.0f);
}

// ========== Comparison Tests ==========

TEST(QuaternionTest, Equality) {
    Quat q1(1.0f, 2.0f, 3.0f, 4.0f);
    Quat q2(1.0f, 2.0f, 3.0f, 4.0f);
    Quat q3(1.0f, 2.0f, 3.0f, 5.0f);
    
    EXPECT_TRUE(q1 == q2);
    EXPECT_FALSE(q1 == q3);
    EXPECT_FALSE(q1 != q2);
    EXPECT_TRUE(q1 != q3);
}

// ========== Interpolation Tests ==========

TEST(QuaternionTest, Lerp) {
    Quat q1 = Quat::identity();
    Quat q2(0.0f, 1.0f, 0.0f, 0.0f);
    
    Quat mid = Quat::lerp(q1, q2, 0.5f);
    EXPECT_TRUE(mid.is_normalized());
    
    Quat start = Quat::lerp(q1, q2, 0.0f);
    Quat end = Quat::lerp(q1, q2, 1.0f);
    
    EXPECT_TRUE(quat_almost_equal(start, q1, 1e-5f));
    EXPECT_TRUE(quat_almost_equal(end, q2.normalized(), 1e-5f));
}

TEST(QuaternionTest, Slerp) {
    // Test SLERP between identity and 90-degree rotation
    Quat q1 = Quat::identity();
    Vec3 axis(0.0f, 0.0f, 1.0f);
    Quat q2(axis, M_PI / 2);
    
    Quat mid = Quat::slerp(q1, q2, 0.5f);
    EXPECT_TRUE(mid.is_normalized());
    
    // At t=0.5, should be 45-degree rotation
    auto [result_axis, result_angle] = mid.to_axis_angle();
    EXPECT_TRUE(vec_almost_equal(result_axis, axis, 1e-5f));
    EXPECT_TRUE(abs(result_angle - M_PI / 4) < 1e-5f);
    
    Quat start = Quat::slerp(q1, q2, 0.0f);
    Quat end = Quat::slerp(q1, q2, 1.0f);
    
    EXPECT_TRUE(quat_almost_equal(start, q1, 1e-5f));
    EXPECT_TRUE(quat_almost_equal(end, q2, 1e-5f));
}

// ========== Edge Case Tests ==========

TEST(QuaternionTest, ZeroQuaternion) {
    Quat zero(0.0f, 0.0f, 0.0f, 0.0f);
    Quat normalized = zero.normalized();
    EXPECT_TRUE(normalized.is_identity());
}

TEST(QuaternionTest, OppositeVectors) {
    Vec3 v1(1.0f, 0.0f, 0.0f);
    Vec3 v2(-1.0f, 0.0f, 0.0f);
    Quat q(v1, v2);
    
    Vec3 rotated = q.rotate(v1);
    EXPECT_TRUE(vec_almost_equal(rotated, v2, 1e-5f));
}

TEST(QuaternionTest, SameVectors) {
    Vec3 v(1.0f, 2.0f, 3.0f);
    Quat q(v, v);
    
    EXPECT_TRUE(q.is_identity(1e-5f));
}

// ========== Type Alias Tests ==========

TEST(QuaternionTest, TypeAliases) {
    // Test that type aliases work correctly
    static_assert(std::is_same_v<Quat, Quaternion<Float>>);
    static_assert(std::is_same_v<Quatf, Quaternion<float>>);
    static_assert(std::is_same_v<Quatd, Quaternion<double>>);
    
    Quat q1;
    Quatf q2;
    Quatd q3;
    
    EXPECT_TRUE(q1.is_identity());
    EXPECT_TRUE(q2.is_identity());
    EXPECT_TRUE(q3.is_identity());
}

// ========== Stream Output Test ==========

TEST(QuaternionTest, StreamOutput) {
    Quat q(1.0f, 2.0f, 3.0f, 4.0f);
    std::stringstream ss;
    ss << q;
    EXPECT_EQ(ss.str(), "Quat(1, 2, 3, 4)");
}

// ========== Constexpr Tests ==========

TEST(QuaternionTest, ConstexprSupport) {
    // Test compile-time construction and operations
    constexpr Quat q1;
    constexpr Quat q2(1.0f, 0.0f, 0.0f, 0.0f);
    constexpr Quat q3 = Quat::identity();
    
    static_assert(q1.w() == 1.0f);
    static_assert(q2.w() == 1.0f);
    static_assert(q3.w() == 1.0f);
    
    constexpr Quat sum = q1 + q2;
    static_assert(sum.w() == 2.0f);
    
    SUCCEED() << "Constexpr operations work correctly.";
}

// ========== Performance Test ==========

TEST(QuaternionTest, RotationComposition) {
    // Test that multiple rotations compose correctly
    Vec3 axis1(1.0f, 0.0f, 0.0f);
    Vec3 axis2(0.0f, 1.0f, 0.0f);
    Vec3 axis3(0.0f, 0.0f, 1.0f);
    
    Quat q1(axis1, M_PI / 4);
    Quat q2(axis2, M_PI / 4);
    Quat q3(axis3, M_PI / 4);
    
    Quat composed = q3 * q2 * q1;
    
    Vec3 v(1.0f, 0.0f, 0.0f);
    Vec3 result1 = composed.rotate(v);
    Vec3 result2 = q3.rotate(q2.rotate(q1.rotate(v)));
    
    EXPECT_TRUE(vec_almost_equal(result1, result2, 1e-5f));
}

} // namespace pbpt::math::testing