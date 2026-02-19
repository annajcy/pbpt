#include <gtest/gtest.h>

#include "pbpt/math/point.hpp"
#include "pbpt/math/vector.hpp"
#include "pbpt/texture/plugin/texture/bitmap_texture.hpp"
#include "pbpt/texture/plugin/texture/checkerboard_texture.hpp"
#include "pbpt/texture/image.hpp"
#include "pbpt/texture/texture.hpp"

TEST(TextureSamplingTest, CheckerboardAlternatesCells) {
    using RGB = pbpt::radiometry::RGB<float>;
    pbpt::texture::CheckerboardTexture<float> tex(RGB(0.1f, 0.2f, 0.3f), RGB(0.9f, 0.8f, 0.7f), 2.0f, 2.0f);

    pbpt::texture::TextureEvalContext<float> c0{};
    c0.uv = pbpt::math::Point<float, 2>(0.1f, 0.1f);
    pbpt::texture::TextureEvalContext<float> c1{};
    c1.uv = pbpt::math::Point<float, 2>(0.6f, 0.1f);

    const auto a = tex.eval(c0);
    const auto b = tex.eval(c1);

    EXPECT_NEAR(a.r(), 0.1f, 1e-6f);
    EXPECT_NEAR(a.g(), 0.2f, 1e-6f);
    EXPECT_NEAR(a.b(), 0.3f, 1e-6f);
    EXPECT_NEAR(b.r(), 0.9f, 1e-6f);
    EXPECT_NEAR(b.g(), 0.8f, 1e-6f);
    EXPECT_NEAR(b.b(), 0.7f, 1e-6f);
}

TEST(TextureSamplingTest, BitmapTextureSamplesImageData) {
    pbpt::texture::Image<pbpt::math::Vector<float, 3>> image(2, 2);
    image.get_pixel(0, 0) = pbpt::math::Vector<float, 3>(1.0f, 0.0f, 0.0f);
    image.get_pixel(1, 0) = pbpt::math::Vector<float, 3>(0.0f, 1.0f, 0.0f);
    image.get_pixel(0, 1) = pbpt::math::Vector<float, 3>(0.0f, 0.0f, 1.0f);
    image.get_pixel(1, 1) = pbpt::math::Vector<float, 3>(1.0f, 1.0f, 1.0f);

    pbpt::texture::BitmapTexture<float> tex(std::move(image), pbpt::texture::WrapMode::Clamp,
                                            pbpt::texture::WrapMode::Clamp);

    pbpt::texture::TextureEvalContext<float> ctx{};
    ctx.uv = pbpt::math::Point<float, 2>(0.5f, 0.5f);

    const auto rgb = tex.eval(ctx);
    EXPECT_NEAR(rgb.r(), 0.5f, 1e-5f);
    EXPECT_NEAR(rgb.g(), 0.5f, 1e-5f);
    EXPECT_NEAR(rgb.b(), 0.5f, 1e-5f);
}
