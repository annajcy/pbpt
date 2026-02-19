#include <gtest/gtest.h>

#include <iostream>

#include "pbpt/pbpt.h"

namespace pbpt::sampler::testing {

using pbpt::math::Float;
using pbpt::math::pi_v;
using pbpt::math::Point;
using pbpt::math::Vector;

TEST(SamplingTest, Uniform2D) {
    Point<Float, 2> uv{0.25f, 0.75f};
    Vector<Float, 2> x_range{2.0f, 4.0f};
    Vector<Float, 2> y_range{-2.0f, 2.0f};
    auto p = sample_uniform_2d(uv, x_range, y_range);

    std::cout << "Sampled Point: " << p << std::endl;

    EXPECT_FLOAT_EQ(p.x(), 2.5f);
    EXPECT_FLOAT_EQ(p.y(), 1.0f);
    EXPECT_FLOAT_EQ(sample_uniform_2d_pdf(p, x_range, y_range), 0.125f);
    EXPECT_FLOAT_EQ(sample_uniform_2d_pdf(Point<Float, 2>{5.0f, 0.0f}, x_range, y_range), 0.0f);
}

TEST(SamplingTest, Tent2D) {
    Point<Float, 2> uv{0.25f, 0.75f};
    Float rx = 2.0f;
    Float ry = 1.0f;
    auto p = sample_tent_2d(uv, rx, ry);

    Float expected_x = sample_tent(uv.x(), rx);
    Float expected_y = sample_tent(uv.y(), ry);
    EXPECT_NEAR(p.x(), expected_x, 1e-6f);
    EXPECT_NEAR(p.y(), expected_y, 1e-6f);
    EXPECT_NEAR(sample_tent_2d_pdf(p, rx, ry), sample_tent_pdf(expected_x, rx) * sample_tent_pdf(expected_y, ry),
                1e-6f);
}

TEST(SamplingTest, UniformDisk) {
    Float radius = 2.0f;
    auto rim = sample_uniform_disk(Point<Float, 2>{1.0f, 0.0f}, radius);
    auto center = sample_uniform_disk(Point<Float, 2>{0.0f, 0.25f}, radius);

    EXPECT_NEAR(rim.x(), radius, 1e-5f);
    EXPECT_NEAR(rim.y(), 0.0f, 1e-5f);
    EXPECT_NEAR(center.x(), 0.0f, 1e-5f);
    EXPECT_NEAR(center.y(), 0.0f, 1e-5f);
    EXPECT_NEAR(sample_uniform_disk_pdf(radius), 1.0f / (pi_v<Float> * radius * radius), 1e-7f);
}

TEST(SamplingTest, UniformDiskConcentric) {
    Float radius = 1.5f;
    auto on_axis = sample_uniform_disk_concentric(Point<Float, 2>{1.0f, 0.5f}, radius);
    auto diagonal = sample_uniform_disk_concentric(Point<Float, 2>{0.0f, 1.0f}, radius);
    auto origin = sample_uniform_disk_concentric(Point<Float, 2>{0.5f, 0.5f}, radius);

    EXPECT_NEAR(on_axis.x(), radius, 1e-5f);
    EXPECT_NEAR(on_axis.y(), 0.0f, 1e-5f);
    EXPECT_LE(diagonal.to_vector().length(), radius + 1e-5f);
    EXPECT_FLOAT_EQ(origin.x(), 0.0f);
    EXPECT_FLOAT_EQ(origin.y(), 0.0f);
}

}  // namespace pbpt::sampler::testing
