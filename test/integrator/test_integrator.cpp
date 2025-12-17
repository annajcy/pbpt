#include <gtest/gtest.h>
#include <numeric>
#include <vector>

#include "pbpt.h"
#include "integrator/domain.hpp"

namespace pbpt::integrator::testing {

TEST(Radiometric, UniformHemisphereIntegral_Cosine)
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

TEST(Radiometric, CosineWeightedHemisphereIntegral_Cosine)
{
    math::RandomGenerator<double, 2> rng2d(789);
    CosineWeightedHemisphereDomain<double> proj_hemi;
    math::Normal3 n(0.0, 0.0, 1.0);
    int sample_count = 100000;
    auto res = integrate<double>(proj_hemi, [&n](const math::Vector<double, 3>& wi) {
        return n.to_vector().dot(wi);
    }, sample_count, rng2d);
    EXPECT_NEAR(res, math::pi_v<double>, 0.01);
}

TEST(Radiometric, HemisphereCosineVarianceComparison)
{
    math::Normal3 n(0.0, 0.0, 1.0);
    const auto n_vec = n.to_vector();
    constexpr int trials = 64;
    constexpr int sample_count = 4096;

    auto variance_of_estimator = [&](auto&& domain_factory) {
        std::vector<double> estimates;
        estimates.reserve(trials);
        for (int i = 0; i < trials; ++i) {
            math::RandomGenerator<double, 2> rng2d(1000 + i);
            auto domain = domain_factory();
            double estimate = integrate<double>(domain, [&n_vec](const math::Vector<double, 3>& wi) {
                return n_vec.dot(wi);
            }, sample_count, rng2d);
            estimates.push_back(estimate);
        }

        double mean = std::accumulate(estimates.begin(), estimates.end(), 0.0) / static_cast<double>(estimates.size());
        double var_sum = std::accumulate(estimates.begin(), estimates.end(), 0.0, [mean](double acc, double v) {
            double d = v - mean;
            return acc + d * d;
        });
        return var_sum / static_cast<double>(estimates.size() - 1);
    };

    auto var_uniform = variance_of_estimator([]() { return UniformHemisphereDomain<double>{}; });
    auto var_cosine  = variance_of_estimator([]() { return CosineWeightedHemisphereDomain<double>{}; });

    std::cout << "Variance Uniform Hemisphere: " << var_uniform << std::endl;
    std::cout << "Variance Cosine-Weighted Hemisphere: " << var_cosine << std::endl;

    EXPECT_LT(var_cosine, var_uniform);
    EXPECT_LT(var_cosine, var_uniform * 0.7); // cosine-weighted should reduce variance noticeably
}

TEST(Radiometric, UniformDiskIntegral_Constant)
{
    math::RandomGenerator<double, 2> rng2d(456);
    UniformDiskDomain<double> disk;
    int sample_count = 100000;
    auto res = integrate<double>(disk, [](const math::Point<double, 2>& p) {
        return 1.0;
    }, sample_count, rng2d);
    EXPECT_NEAR(res, math::pi_v<double>, 0.01);
}

TEST(Radiometric, ParallelogramAreaIntegral)
{
    math::RandomGenerator<double, 2> rng2d(321);
    UniformParallelogramAreaDomain<double> para{
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
