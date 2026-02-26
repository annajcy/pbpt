#include <gtest/gtest.h>
#include "pbpt/light_sampler/plugin/light_sampler/uniform_light_sampler.hpp"
#include "pbpt/light_sampler/plugin/light_sampler/light_sampler_type.hpp"
#include "pbpt/camera/render_transform.hpp"
#include "pbpt/shape/plugin/shape/triangle.hpp"
#include "pbpt/radiometry/plugin/spectrum_distribution/piecewise_linear.hpp"
#include "pbpt/radiometry/constant/illuminant_spectrum.hpp"
#include <memory>
#include <vector>
#include <span>

namespace pbpt::light_sampler::testing {

TEST(UniformLightSamplerTest, EmptyLibrary) {
    using T = float;
    std::vector<const light::AnyLight<T>*> empty_lights;
    UniformLightSampler<T> sampler(empty_lights);

    EXPECT_EQ(sampler.light_count(), 0);
    EXPECT_EQ(sampler.pdf(0), T(0));
    EXPECT_EQ(sampler.pdf(10), T(0));

    auto result = sampler.sample(T(0.5));
    EXPECT_EQ(result.light, nullptr);
    EXPECT_EQ(result.selection_pdf, T(0));
}

TEST(UniformLightSamplerTest, MultipleLights) {
    using T = float;

    camera::RenderTransform<T> render_transform;
    std::vector<int> indices = {0, 1, 2};
    std::vector<math::Point<T, 3>> positions = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
    auto mesh = std::make_shared<shape::TriangleMesh<T>>(render_transform, indices, positions);
    shape::Triangle<T> tri(*mesh, 0);

    std::vector<std::pair<T, T>> points = {{400, 1}, {700, 1}};
    radiometry::PiecewiseLinearSpectrumDistribution<T> pwl(points);
    auto dist = pwl * radiometry::constant::CIE_D65_ilum<T>;

    light::AnyLight<T> area_light1 = light::StandardAreaLightType<T>(tri, dist);
    light::AnyLight<T> area_light2 = light::StandardAreaLightType<T>(tri, dist);
    light::AnyLight<T> area_light3 = light::StandardAreaLightType<T>(tri, dist);

    std::vector<const light::AnyLight<T>*> lights = {&area_light1, &area_light2, &area_light3};

    UniformLightSampler<T> sampler(lights);

    EXPECT_EQ(sampler.light_count(), 3);
    EXPECT_FLOAT_EQ(sampler.pdf(0), T(1) / T(3));
    EXPECT_FLOAT_EQ(sampler.pdf(1), T(1) / T(3));
    EXPECT_FLOAT_EQ(sampler.pdf(2), T(1) / T(3));

    auto res1 = sampler.sample(T(0.1));
    EXPECT_FLOAT_EQ(res1.selection_pdf, T(1) / T(3));
    EXPECT_NE(res1.light, nullptr);

    auto res2 = sampler.sample(T(0.5));
    EXPECT_NE(res2.light, nullptr);

    auto res3 = sampler.sample(T(0.9));
    EXPECT_NE(res3.light, nullptr);

    EXPECT_NE(res1.light, res2.light);
    EXPECT_NE(res2.light, res3.light);
    EXPECT_NE(res1.light, res3.light);
}

}  // namespace pbpt::light_sampler::testing
