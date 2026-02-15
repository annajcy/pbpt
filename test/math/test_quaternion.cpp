#include <gtest/gtest.h>

#include "pbpt/pbpt.h"

namespace pbpt::math::testing {

TEST(QuaternionTest, IdentityRotateVector) {
    const Quat q = Quat::identity();
    const Vec3 v(1.0f, 2.0f, 3.0f);
    EXPECT_EQ(q * v, v);
}

TEST(QuaternionTest, AxisAngleRotate90DegY) {
    const Quat q = angleAxis(radians(90.0f), Vec3(0.0f, 1.0f, 0.0f));
    const Vec3 z(0.0f, 0.0f, 1.0f);
    const Vec3 x(1.0f, 0.0f, 0.0f);
    const Vec3 rz = normalize(q * z);
    EXPECT_NEAR(rz.x(), x.x(), 1e-4f);
    EXPECT_NEAR(rz.y(), x.y(), 1e-4f);
    EXPECT_NEAR(rz.z(), x.z(), 1e-4f);
}

TEST(QuaternionTest, Mat3RoundTrip) {
    const Quat q0 = normalize(angleAxis(radians(33.0f), Vec3(1.0f, 2.0f, 3.0f)));
    const Mat3 m = mat3_cast(q0);
    const Quat q1 = normalize(quat_cast(m));
    EXPECT_NEAR(q0.w(), q1.w(), 1e-4f);
    EXPECT_NEAR(q0.x(), q1.x(), 1e-4f);
    EXPECT_NEAR(q0.y(), q1.y(), 1e-4f);
    EXPECT_NEAR(q0.z(), q1.z(), 1e-4f);
}

TEST(QuaternionTest, RotationFromToDegenerate) {
    const Vec3 v(1.0f, 0.0f, 0.0f);
    const Quat q = rotation(v, v);
    const Vec3 rv = q * v;
    EXPECT_NEAR(rv.x(), v.x(), 1e-4f);
    EXPECT_NEAR(rv.y(), v.y(), 1e-4f);
    EXPECT_NEAR(rv.z(), v.z(), 1e-4f);
}

}  // namespace pbpt::math::testing
