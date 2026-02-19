#include <gtest/gtest.h>

#include "pbpt/geometry/interaction.hpp"
#include "pbpt/math/point.hpp"
#include "pbpt/texture/image.hpp"
#include "pbpt/texture/mipmap.hpp"

TEST(MipMapTest, BuildsExpectedPyramidDimensions) {
    pbpt::texture::Image<float> image(4, 2);
    pbpt::texture::MipMap<float, float> mip(std::move(image));

    ASSERT_EQ(mip.level_count(), 3u);
    EXPECT_EQ(mip.level(0).width(), 4);
    EXPECT_EQ(mip.level(0).height(), 2);
    EXPECT_EQ(mip.level(1).width(), 2);
    EXPECT_EQ(mip.level(1).height(), 1);
    EXPECT_EQ(mip.level(2).width(), 1);
    EXPECT_EQ(mip.level(2).height(), 1);
}

TEST(MipMapTest, RepeatWrapMatchesWrappedCoordinate) {
    pbpt::texture::Image<float> image(2, 2);
    image.get_pixel(0, 0) = 0.0f;
    image.get_pixel(1, 0) = 1.0f;
    image.get_pixel(0, 1) = 2.0f;
    image.get_pixel(1, 1) = 3.0f;

    pbpt::texture::MipMap<float, float> mip(std::move(image), pbpt::texture::WrapMode::Repeat,
                                            pbpt::texture::WrapMode::Repeat);

    pbpt::texture::TextureEvalContext<float> c0{};
    c0.uv = pbpt::math::Point<float, 2>(0.25f, 0.25f);
    pbpt::texture::TextureEvalContext<float> c1{};
    c1.uv = pbpt::math::Point<float, 2>(1.25f, 0.25f);

    const float v0 = mip.sample(c0);
    const float v1 = mip.sample(c1);
    EXPECT_NEAR(v0, v1, 1e-6f);
}

TEST(MipMapTest, LargeDifferentialsUseCoarserLevels) {
    pbpt::texture::Image<float> image(4, 4);
    for (int y = 0; y < 4; ++y) {
        for (int x = 0; x < 4; ++x) {
            image.get_pixel(x, y) = (x == 0 && y == 0) ? 0.0f : 1.0f;
        }
    }
    pbpt::texture::MipMap<float, float> mip(std::move(image));

    pbpt::texture::TextureEvalContext<float> fine{};
    fine.uv = pbpt::math::Point<float, 2>(0.125f, 0.125f);
    const float fine_value = mip.sample(fine);

    pbpt::texture::TextureEvalContext<float> coarse{};
    coarse.uv = pbpt::math::Point<float, 2>(0.125f, 0.125f);
    pbpt::geometry::SurfaceDifferentials<float> diffs{};
    diffs.dudx = 1.0f;
    diffs.dvdy = 1.0f;
    coarse.differentials = diffs;
    const float coarse_value = mip.sample(coarse);

    EXPECT_LT(fine_value, coarse_value);
    EXPECT_GT(coarse_value, 0.5f);
}
