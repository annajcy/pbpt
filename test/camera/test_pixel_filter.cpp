#include <cmath>
#include <gtest/gtest.h>

#include "pbpt/camera/plugin/pixel_filter/box_filter.hpp"
#include "pbpt/camera/plugin/pixel_filter/tent_filter.hpp"
#include "pbpt/math/point.hpp"

namespace pbpt::camera::testing {

namespace {

template <typename T>
math::Point<T, 2> pixel_center(const math::Point<int, 2>& pixel) {
    return math::Point<T, 2>(
        static_cast<T>(pixel.x()) + T(0.5),
        static_cast<T>(pixel.y()) + T(0.5)
    );
}

}  // namespace

TEST(PixelFilterTest, StratifiedUvSamplesAreCellCentered) {
    BoxFilter<float> filter(1.0f);

    auto samples = filter.get_uv_samples(2, 2);

    ASSERT_EQ(samples.size(), 4u);
    EXPECT_FLOAT_EQ(samples[0].x(), 0.25f);
    EXPECT_FLOAT_EQ(samples[0].y(), 0.25f);
    EXPECT_FLOAT_EQ(samples[1].x(), 0.75f);
    EXPECT_FLOAT_EQ(samples[1].y(), 0.25f);
    EXPECT_FLOAT_EQ(samples[2].x(), 0.25f);
    EXPECT_FLOAT_EQ(samples[2].y(), 0.75f);
    EXPECT_FLOAT_EQ(samples[3].x(), 0.75f);
    EXPECT_FLOAT_EQ(samples[3].y(), 0.75f);
}

TEST(BoxFilterTest, SampleFilmPositionMatchesUniformOffset) {
    using T = float;
    const T radius = 1.0f;
    BoxFilter<T> filter(radius);
    math::Point<int, 2> pixel(10, 5);
    math::Point<T, 2> uv(0.25f, 0.75f);

    auto filtered = filter.sample_film_position(pixel, uv);

    auto expected_offset = sampler::sample_uniform_2d(
        uv,
        math::Vector<T, 2>(-radius, radius),
        math::Vector<T, 2>(-radius, radius)
    );
    auto expected_position = pixel_center<T>(pixel) + expected_offset.to_vector();
    auto expected_pdf = sampler::sample_uniform_2d_pdf(
        expected_offset,
        math::Vector<T, 2>(-radius, radius),
        math::Vector<T, 2>(-radius, radius)
    );
    auto expected_weight = (expected_pdf > T(0)) ? (T(1) / expected_pdf) : T(0);

    EXPECT_FLOAT_EQ(filtered.film_position.x(), expected_position.x());
    EXPECT_FLOAT_EQ(filtered.film_position.y(), expected_position.y());
    EXPECT_FLOAT_EQ(filtered.offset.x(), expected_offset.x());
    EXPECT_FLOAT_EQ(filtered.offset.y(), expected_offset.y());
    EXPECT_FLOAT_EQ(filtered.weight, expected_weight);
}

TEST(TentFilterTest, TentPdfMatchesSampledOffset) {
    using T = float;
    const T radius = 1.0f;
    TentFilter<T> filter(radius);
    math::Point<int, 2> pixel(3, 7);
    math::Point<T, 2> uv(0.25f, 0.75f);

    auto filtered = filter.sample_film_position(pixel, uv);
    const auto& offset = filtered.offset;
    math::Point<T, 2> offset_point = math::Point<T, 2>::from_vector(offset);

    auto expected_pdf = sampler::sample_tent_2d_pdf(offset_point, radius, radius);
    T ax = std::abs(offset.x()) / radius;
    T ay = std::abs(offset.y()) / radius;
    T expected_kernel = (ax > T(1) || ay > T(1)) ? T(0) : (T(1) - ax) * (T(1) - ay);
    auto expected_weight = (expected_pdf > T(0)) ? (expected_kernel / expected_pdf) : T(0);

    EXPECT_NEAR(filtered.weight, expected_weight, 1e-5f);
    EXPECT_GT(filtered.weight, T(0));
}

TEST(BoxFilterTest, CameraSamplesUseAllUvStrata) {
    using T = float;
    const T radius = 0.5f;
    BoxFilter<T> filter(radius);
    math::Point<int, 2> pixel(2, 4);
    auto samples = filter.get_camera_samples(pixel, 2, 2);

    ASSERT_EQ(samples.size(), 4u);

    auto uvs = filter.get_uv_samples(2, 2);
    auto pixel_ctr = pixel_center<T>(pixel);
    auto expected_pdf = sampler::sample_uniform_2d_pdf(
        math::Point<T, 2>(0, 0),  // position does not affect uniform pdf when inside range
        math::Vector<T, 2>(-radius, radius),
        math::Vector<T, 2>(-radius, radius)
    );
    auto expected_weight = (expected_pdf > T(0)) ? (T(1) / expected_pdf) : T(0);

    for (std::size_t i = 0; i < samples.size(); ++i) {
        auto expected_offset = sampler::sample_uniform_2d(
            uvs[i],
            math::Vector<T, 2>(-radius, radius),
            math::Vector<T, 2>(-radius, radius)
        );
        auto expected_position = pixel_ctr + expected_offset.to_vector();

        EXPECT_NEAR(samples[i].film_position.x(), expected_position.x(), 1e-6f);
        EXPECT_NEAR(samples[i].film_position.y(), expected_position.y(), 1e-6f);
        EXPECT_NEAR(samples[i].offset.x(), expected_offset.x(), 1e-6f);
        EXPECT_NEAR(samples[i].offset.y(), expected_offset.y(), 1e-6f);
        EXPECT_NEAR(samples[i].weight, expected_weight, 1e-6f);
    }
}

TEST(BoxFilterTest, WeightIsConstantAcrossSamples) {
    using T = float;
    const T radius = 0.75f;
    BoxFilter<T> filter(radius);
    auto uvs = filter.get_uv_samples(3, 2);

    T expected_weight = (T(2) * radius) * (T(2) * radius);  // area of the box
    for (const auto& uv : uvs) {
        auto sample = filter.sample_film_position(math::Point<T, 2>{0, 0}, uv);
        EXPECT_NEAR(sample.weight, expected_weight, 1e-6f);
    }
}

TEST(TentFilterTest, WeightIsConstantAcrossSamples) {
    using T = float;
    const T radius = 1.2f;
    TentFilter<T> filter(radius);
    auto uvs = filter.get_uv_samples(2, 3);

    T expected_weight = radius * radius;  // tent kernel / tent pdf yields r^2
    for (const auto& uv : uvs) {
        auto sample = filter.sample_film_position(math::Point<T, 2>{0, 0}, uv);
        EXPECT_NEAR(sample.weight, expected_weight, 1e-6f);
    }
}

}  // namespace pbpt::camera::testing
