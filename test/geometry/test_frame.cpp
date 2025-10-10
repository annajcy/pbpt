#include <gtest/gtest.h>

#include "pbpt.h"

using namespace pbpt::geometry;

namespace pbpt::geometry::testing {

using Vec3d = Vector<double, 3>;

constexpr double kEps = 1e-6;

TEST(FrameTest, ConstructFromNormalRightHanded) {
    Vec3d normal(0.0, 0.0, 1.0);
    Frame<double> f(normal);
    // Ensure basis vectors are orthonormal
    EXPECT_NEAR(f.n().dot(normal.normalized()), 1.0, kEps);
    EXPECT_NEAR(f.t().dot(f.n()), 0.0, kEps);
    EXPECT_NEAR(f.b().dot(f.n()), 0.0, kEps);
    EXPECT_NEAR(f.t().dot(f.b()), 0.0, kEps);
    // Right-handed: cross(t, b) == n
    Vec3d cross_tb = cross(f.t(), f.b());
    EXPECT_NEAR(cross_tb.x(), f.n().x(), kEps);
    EXPECT_NEAR(cross_tb.y(), f.n().y(), kEps);
    EXPECT_NEAR(cross_tb.z(), f.n().z(), kEps);
}

TEST(FrameTest, ConstructFromNormalLeftHanded) {
    Vec3d normal(0.0, 1.0, 0.0);
    Frame<double> f(normal, true);
    // Left-handed: cross(t, b) == -n
    Vec3d cross_tb = cross(f.t(), f.b());
    EXPECT_NEAR(cross_tb.x(), -f.n().x(), kEps);
    EXPECT_NEAR(cross_tb.y(), -f.n().y(), kEps);
    EXPECT_NEAR(cross_tb.z(), -f.n().z(), kEps);
}

TEST(FrameTest, ConstructFromTangentAndNormal) {
    Vec3d tangent(1.0, 0.0, 0.0);
    Vec3d normal(0.0, 1.0, 0.0);
    Frame<double> f(tangent, normal);
    // t == normalized tangent
    EXPECT_NEAR(f.t().x(), 1.0, kEps);
    EXPECT_NEAR(f.t().y(), 0.0, kEps);
    EXPECT_NEAR(f.t().z(), 0.0, kEps);
    // n == normalized normal
    EXPECT_NEAR(f.n().x(), 0.0, kEps);
    EXPECT_NEAR(f.n().y(), 1.0, kEps);
    EXPECT_NEAR(f.n().z(), 0.0, kEps);
    // Orthogonality
    EXPECT_NEAR(f.t().dot(f.n()), 0.0, kEps);
    EXPECT_NEAR(f.b().dot(f.n()), 0.0, kEps);
    EXPECT_NEAR(f.t().dot(f.b()), 0.0, kEps);
    // Right-handed: cross(t, b) == n
    Vec3d cross_tb = cross(f.t(), f.b());
    EXPECT_NEAR(cross_tb.x(), f.n().x(), kEps);
    EXPECT_NEAR(cross_tb.y(), f.n().y(), kEps);
    EXPECT_NEAR(cross_tb.z(), f.n().z(), kEps);
}

TEST(FrameTest, LocalToWorldAndWorldToLocal) {
    Vec3d normal(1.0, 1.0, 1.0);
    Frame<double> f(normal);
    Vec3d localVec(2.0, 3.0, 4.0);
    // Convert local to world
    Vec3d world = f.local_to_world() * localVec;
    // Manual computation: world = t*2 + b*3 + n*4
    Vec3d manual = f.t() * 2.0 + f.b() * 3.0 + f.n() * 4.0;
    EXPECT_NEAR(world.x(), manual.x(), kEps);
    EXPECT_NEAR(world.y(), manual.y(), kEps);
    EXPECT_NEAR(world.z(), manual.z(), kEps);
    // Convert back to local
    Vec3d back = f.world_to_local() * world;
    EXPECT_NEAR(back.x(), localVec.x(), kEps);
    EXPECT_NEAR(back.y(), localVec.y(), kEps);
    EXPECT_NEAR(back.z(), localVec.z(), kEps);
}

TEST(FrameTest, BasisVectorsNormalized) {
    // Test with various normal vectors
    std::vector<Vec3d> normals = {
        Vec3d(0.0, 0.0, 1.0),
        Vec3d(1.0, 0.0, 0.0), 
        Vec3d(0.0, 1.0, 0.0),
        Vec3d(1.0, 1.0, 1.0),
        Vec3d(-0.5, 0.8, 0.3),
        Vec3d(0.267261, 0.534522, 0.801784)  // Random normalized vector
    };
    
    for (const auto& normal : normals) {
        Frame<double> f(normal);
        // All basis vectors should be unit length
        EXPECT_NEAR(f.t().length(), 1.0, kEps);
        EXPECT_NEAR(f.b().length(), 1.0, kEps);
        EXPECT_NEAR(f.n().length(), 1.0, kEps);
    }
}

TEST(FrameTest, HandednessConsistency) {
    Vec3d normal(0.0, 0.0, 1.0);
    
    // Right-handed frame
    Frame<double> f_right(normal, false);
    Vec3d cross_right = cross(f_right.t(), f_right.b());
    EXPECT_NEAR(cross_right.dot(f_right.n()), 1.0, kEps);
    
    // Left-handed frame
    Frame<double> f_left(normal, true);
    Vec3d cross_left = cross(f_left.t(), f_left.b());
    EXPECT_NEAR(cross_left.dot(f_left.n()), -1.0, kEps);
}

TEST(FrameTest, TransformInverseProperty) {
    Vec3d normal(0.2, 0.6, 0.8);
    Frame<double> f(normal);
    
    // Test that local_to_world and world_to_local are inverses
    auto comp1 = f.local_to_world() * f.world_to_local();
    auto comp2 = f.world_to_local() * f.local_to_world();
    
    EXPECT_TRUE(comp1.is_identity());
    EXPECT_TRUE(comp2.is_identity());
}

TEST(FrameTest, EdgeCaseNormalAlmostX) {
    // Test when normal is very close to (1,0,0) - should use (0,1,0) as up
    Vec3d normal(0.999, 0.0, 0.0);
    Frame<double> f(normal);
    
    // Should still be orthonormal
    EXPECT_NEAR(f.t().dot(f.n()), 0.0, kEps);
    EXPECT_NEAR(f.b().dot(f.n()), 0.0, kEps);
    EXPECT_NEAR(f.t().dot(f.b()), 0.0, kEps);
    EXPECT_NEAR(f.t().length(), 1.0, kEps);
    EXPECT_NEAR(f.b().length(), 1.0, kEps);
    EXPECT_NEAR(f.n().length(), 1.0, kEps);
}

TEST(FrameTest, TangentNormalConstructor) {
    Vec3d tangent(1.0, 0.0, 0.0);
    Vec3d normal(0.0, 1.0, 0.0);
    Frame<double> f(tangent, normal);
    
    // Expected bitangent should be cross(normal, tangent)
    Vec3d expected_b = cross(normal, tangent);
    EXPECT_NEAR(f.b().x(), expected_b.x(), kEps);
    EXPECT_NEAR(f.b().y(), expected_b.y(), kEps);
    EXPECT_NEAR(f.b().z(), expected_b.z(), kEps);
}

TEST(FrameTest, TangentNormalLeftHanded) {
    Vec3d tangent(1.0, 0.0, 0.0);
    Vec3d normal(0.0, 1.0, 0.0);
    Frame<double> f(tangent, normal, true);
    
    // For left-handed, bitangent should be -cross(normal, tangent)
    Vec3d expected_b = -cross(normal, tangent);
    EXPECT_NEAR(f.b().x(), expected_b.x(), kEps);
    EXPECT_NEAR(f.b().y(), expected_b.y(), kEps);
    EXPECT_NEAR(f.b().z(), expected_b.z(), kEps);
}

TEST(FrameTest, MultipleVectorTransformation) {
    Vec3d normal(0.0, 0.0, 1.0);
    Frame<double> f(normal);
    
    std::vector<Vec3d> local_vectors = {
        Vec3d(1.0, 0.0, 0.0),  // Pure tangent
        Vec3d(0.0, 1.0, 0.0),  // Pure bitangent
        Vec3d(0.0, 0.0, 1.0),  // Pure normal
        Vec3d(1.0, 1.0, 1.0),  // Mixed
        Vec3d(-0.5, 2.0, -1.5) // Arbitrary
    };
    
    for (const auto& local_vec : local_vectors) {
        Vec3d world_vec = f.local_to_world() * local_vec;
        Vec3d back_to_local = f.world_to_local() * world_vec;
        
        EXPECT_NEAR(back_to_local.x(), local_vec.x(), kEps);
        EXPECT_NEAR(back_to_local.y(), local_vec.y(), kEps);
        EXPECT_NEAR(back_to_local.z(), local_vec.z(), kEps);
    }
}

TEST(FrameTest, FloatTypeCompatibility) {
    // Test with float type instead of double
    using Vec3f = Vector<float, 3>;
    Vec3f normal_f(0.0f, 0.0f, 1.0f);
    Frame<float> f_float(normal_f);
    
    // Should still maintain orthonormality
    const float kEpsf = 1e-5f;
    EXPECT_NEAR(f_float.t().dot(f_float.n()), 0.0f, kEpsf);
    EXPECT_NEAR(f_float.b().dot(f_float.n()), 0.0f, kEpsf);
    EXPECT_NEAR(f_float.t().dot(f_float.b()), 0.0f, kEpsf);
    EXPECT_NEAR(f_float.t().length(), 1.0f, kEpsf);
    EXPECT_NEAR(f_float.b().length(), 1.0f, kEpsf);
    EXPECT_NEAR(f_float.n().length(), 1.0f, kEpsf);
}

} // namespace pbpt::math::testing
