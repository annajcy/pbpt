#include <gtest/gtest.h>

#include "pbpt/pbpt.h"

namespace pbpt::sampler::testing {

using pbpt::math::Float;
using pbpt::math::pi_v;
using pbpt::math::Point;

TEST(SamplingTest, UniformHemisphere) {
    Float radius = 1.0f;
    auto equator = sample_uniform_hemisphere(Point<Float, 2>{0.0f, 0.0f}, radius);
    auto pole = sample_uniform_hemisphere(Point<Float, 2>{1.0f, 0.25f}, radius);

    EXPECT_GE(equator.z(), 0.0f);
    EXPECT_NEAR(equator.to_vector().length(), radius, 1e-5f);
    EXPECT_NEAR(pole.x(), 0.0f, 1e-5f);
    EXPECT_NEAR(pole.y(), 0.0f, 1e-5f);
    EXPECT_NEAR(pole.z(), radius, 1e-5f);
    EXPECT_NEAR(sample_uniform_hemisphere_pdf(radius), 1.0f / (2.0f * pi_v<Float> * radius * radius), 1e-7f);
}

TEST(SamplingTest, CosineWeightedHemisphere) {
    auto top = sample_cosine_weighted_hemisphere(Point<Float, 2>{0.5f, 0.5f});
    EXPECT_NEAR(top.x(), 0.0f, 1e-5f);
    EXPECT_NEAR(top.y(), 0.0f, 1e-5f);
    EXPECT_NEAR(top.z(), 1.0f, 1e-5f);
    EXPECT_NEAR(top.to_vector().length(), 1.0f, 1e-5f);
    EXPECT_NEAR(sample_cosine_weighted_hemisphere_pdf(top), 1.0f / pi_v<Float>, 1e-7f);
    EXPECT_FLOAT_EQ(sample_cosine_weighted_hemisphere_pdf(Point<Float, 3>{0.0f, 0.0f, -1.0f}), 0.0f);
}

TEST(SamplingTest, UniformSphere) {
    auto north = sample_uniform_sphere(Point<Float, 2>{0.0f, 0.1f});
    auto equator = sample_uniform_sphere(Point<Float, 2>{0.5f, 0.5f});

    EXPECT_NEAR(north.z(), 1.0f, 1e-5f);
    EXPECT_NEAR(north.to_vector().length(), 1.0f, 1e-5f);
    EXPECT_NEAR(equator.z(), 0.0f, 1e-5f);
    EXPECT_NEAR(equator.to_vector().length(), 1.0f, 1e-5f);
    EXPECT_NEAR(sample_uniform_sphere_pdf<Float>(2.0f), 1.0f / (4.0f * pi_v<Float> * 4.0f), 1e-7f);
}

TEST(SamplingTest, UniformTriangle) {
    Point<Float, 3> v0{0.0f, 0.0f, 0.0f};
    Point<Float, 3> v1{1.0f, 0.0f, 0.0f};
    Point<Float, 3> v2{0.0f, 1.0f, 0.0f};

    auto p0 = sample_uniform_triangle(Point<Float, 2>{1.0f, 0.0f}, v0, v1, v2);
    auto p1 = sample_uniform_triangle(Point<Float, 2>{0.0f, 0.0f}, v0, v1, v2);
    auto p2 = sample_uniform_triangle(Point<Float, 2>{1.0f, 1.0f}, v0, v1, v2);
    auto interior = sample_uniform_triangle(Point<Float, 2>{0.25f, 0.5f}, v0, v1, v2);

    EXPECT_FLOAT_EQ(p0.x(), v0.x());
    EXPECT_FLOAT_EQ(p0.y(), v0.y());
    EXPECT_FLOAT_EQ(p1.x(), v1.x());
    EXPECT_FLOAT_EQ(p1.y(), v1.y());
    EXPECT_FLOAT_EQ(p2.x(), v2.x());
    EXPECT_FLOAT_EQ(p2.y(), v2.y());
    EXPECT_NEAR(interior.x(), 0.5f, 1e-6f);
    EXPECT_NEAR(interior.y(), 0.25f, 1e-6f);
    EXPECT_FLOAT_EQ(interior.z(), 0.0f);
    EXPECT_NEAR(sample_uniform_triangle_pdf(v0, v1, v2), 2.0f, 1e-7f);
    EXPECT_FLOAT_EQ(sample_uniform_triangle_pdf(v0, v0, v0), 0.0f);
}

}  // namespace pbpt::sampler::testing
