#include <gtest/gtest.h>

#include <cmath>

#include "pbpt.h"

namespace pbpt::math::testing {

TEST(SamplingTest, UniformInterval) {
    EXPECT_FLOAT_EQ(sample_uniform(0.0f, -1.0f, 3.0f), -1.0f);
    EXPECT_FLOAT_EQ(sample_uniform(1.0f, -1.0f, 3.0f), 3.0f);
    EXPECT_FLOAT_EQ(sample_uniform(0.5f, 2.0f, 4.0f), 3.0f);

    EXPECT_FLOAT_EQ(sample_uniform_pdf(0.3f), 1.0f);
    EXPECT_FLOAT_EQ(sample_uniform_pdf(0.3f, -1.0f, 3.0f), 0.25f);
    EXPECT_FLOAT_EQ(sample_uniform_pdf(-0.1f), 0.0f);
    EXPECT_FLOAT_EQ(sample_uniform_pdf(1.1f), 0.0f);
}

TEST(SamplingTest, Uniform2D) {
    Point<Float, 2>   uv{0.25f, 0.75f};
    Vector<Float, 2>  x_range{2.0f, 4.0f};
    Vector<Float, 2>  y_range{-2.0f, 2.0f};
    auto              p = sample_uniform_2d(uv, x_range, y_range);

    std::cout << "Sampled Point: " << p << std::endl;

    EXPECT_FLOAT_EQ(p.x(), 2.5f);
    EXPECT_FLOAT_EQ(p.y(), 1.0f);
    EXPECT_FLOAT_EQ(sample_uniform_2d_pdf(p, x_range, y_range), 0.125f);
    EXPECT_FLOAT_EQ(sample_uniform_2d_pdf(Point<Float, 2>{5.0f, 0.0f}, x_range, y_range), 0.0f);
}

TEST(SamplingTest, TentDistribution) {
    Float r = 2.0f;
    Float left_expected = -r + r * std::sqrt(2.0f * 0.25f);
    Float right_expected = r - r * std::sqrt(2.0f * (1.0f - 0.75f));

    EXPECT_NEAR(sample_tent(0.0f, r), -r, 1e-6f);
    EXPECT_NEAR(sample_tent(0.25f, r), left_expected, 1e-6f);
    EXPECT_NEAR(sample_tent(0.75f, r), right_expected, 1e-6f);
    EXPECT_NEAR(sample_tent_pdf(0.0f, r), 0.5f, 1e-6f);
    EXPECT_FLOAT_EQ(sample_tent_pdf(3.0f, r), 0.0f);
}

TEST(SamplingTest, Tent2D) {
    Point<Float, 2> uv{0.25f, 0.75f};
    Float           rx = 2.0f;
    Float           ry = 1.0f;
    auto            p = sample_tent_2d(uv, rx, ry);

    Float expected_x = sample_tent(uv.x(), rx);
    Float expected_y = sample_tent(uv.y(), ry);
    EXPECT_NEAR(p.x(), expected_x, 1e-6f);
    EXPECT_NEAR(p.y(), expected_y, 1e-6f);
    EXPECT_NEAR(sample_tent_2d_pdf(p, rx, ry),
                sample_tent_pdf(expected_x, rx) * sample_tent_pdf(expected_y, ry),
                1e-6f);
}

TEST(SamplingTest, UniformDisk) {
    Float radius = 2.0f;
    auto  rim = sample_uniform_disk(Point<Float, 2>{1.0f, 0.0f}, radius);
    auto  center = sample_uniform_disk(Point<Float, 2>{0.0f, 0.25f}, radius);

    EXPECT_NEAR(rim.x(), radius, 1e-5f);
    EXPECT_NEAR(rim.y(), 0.0f, 1e-5f);
    EXPECT_NEAR(center.x(), 0.0f, 1e-5f);
    EXPECT_NEAR(center.y(), 0.0f, 1e-5f);
    EXPECT_NEAR(sample_uniform_disk_pdf(radius), 1.0f / (pi_v<Float> * radius * radius), 1e-7f);
}

TEST(SamplingTest, UniformDiskConcentric) {
    Float radius = 1.5f;
    auto  on_axis = sample_uniform_disk_concentric(Point<Float, 2>{1.0f, 0.5f}, radius);
    auto  diagonal = sample_uniform_disk_concentric(Point<Float, 2>{0.0f, 1.0f}, radius);
    auto  origin = sample_uniform_disk_concentric(Point<Float, 2>{0.5f, 0.5f}, radius);

    EXPECT_NEAR(on_axis.x(), radius, 1e-5f);
    EXPECT_NEAR(on_axis.y(), 0.0f, 1e-5f);
    EXPECT_LE(diagonal.to_vector().length(), radius + 1e-5f);
    EXPECT_FLOAT_EQ(origin.x(), 0.0f);
    EXPECT_FLOAT_EQ(origin.y(), 0.0f);
}

TEST(SamplingTest, UniformHemisphere) {
    Float radius = 1.0f;
    auto  equator = sample_uniform_hemisphere(Point<Float, 2>{0.0f, 0.0f}, radius);
    auto  pole = sample_uniform_hemisphere(Point<Float, 2>{1.0f, 0.25f}, radius);

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

}  // namespace pbpt::math::testing
