#include <gtest/gtest.h>

#include "pbpt.h"

namespace pbpt::radiometry::testing {

TEST(RadiometricIntegrals, UniformHemisphereIntegral_Cosine)
{
    math::RandomGenerator<double, 2> rng2d(123);
    UniformHemisphereDomain<double> hemisphere;
    math::Normal3 n(0.0, 0.0, 1.0);
    int sample_count = 1000000;
    auto res = integrate<double>(hemisphere, [&n](const math::Vector<double, 3>& wi) {
        return n.to_vector().dot(wi);
    }, sample_count, rng2d);
    EXPECT_NEAR(res, math::pi_v<double>, 0.01);
}

TEST(RadiometricIntegrals, UniformDiskIntegral_Constant)
{
    math::RandomGenerator<double, 2> rng2d(456);
    UniformDiskDomain<double> disk;
    int sample_count = 100000;
    auto res = integrate<double>(disk, [](const math::Point<double, 2>& p) {
        return 1.0;
    }, sample_count, rng2d);
    EXPECT_NEAR(res, math::pi_v<double>, 0.01);
}

TEST(RadiometricIntegrals, ProjectedHemisphereIntegral_Constant)
{
    math::RandomGenerator<double, 2> rng2d(789);
    ProjectedHemisphereDomain<double> proj_hemi;
    int sample_count = 100000;
    auto res = integrate<double>(proj_hemi, [](const math::Vector<double, 3>& wi) {
        return 1.0;
    }, sample_count, rng2d);
    EXPECT_NEAR(res, math::pi_v<double>, 0.01);
}

TEST(RadiometricIntegrals, ParallelogramAreaIntegral)
{
    math::RandomGenerator<double, 2> rng2d(321);
    ParallelogramAreaDomain<double> para{
        math::Point<double, 3>(-1.0, 4.0, -1.0),
        math::Vector<double, 3>(2.0, 0.0, 0.0),
        math::Vector<double, 3>(0.0, 0.0, 2.0)
    };
    auto shading_p = math::Point<double, 3>(0.0, 0.0, 0.0);
    auto shading_p_normal = math::Normal3(0.0, 1.0, 0.0);
    int sample_count = 100000;
    auto res = integrate<double>(para, [&shading_p, &shading_p_normal](const SurfaceInfo<double>& surface) {
        auto [p, normal] = surface;
        auto n = normal.to_vector();
        auto pn = shading_p_normal.to_vector();
        auto wi = (p - shading_p).normalized();
        auto cos_p = pn.dot(wi);
        auto cos_x = n.dot(-wi);
        auto r2 = (p - shading_p).length_squared();
        auto L = 1.0;
        return L * cos_p * cos_x / r2;
    }, sample_count, rng2d);
    EXPECT_NEAR(res, 0.2308367977, 0.01);
}


}

