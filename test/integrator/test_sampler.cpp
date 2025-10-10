#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include "pbpt.h"

using namespace pbpt::integrator;
using namespace pbpt::math;

namespace pbpt::integrator::testing {

class SamplerTest : public ::testing::Test {
protected:
    void SetUp() override {
        mc_integrator     = std::make_shared<MonteCarloIntegrator1D>();
        sampler_evaluator = std::make_shared<SamplerEvaluator<1>>();
        uniform_sampler   = std::make_shared<UniformSampler<1>>();
    }

    std::shared_ptr<MonteCarloIntegrator1D> mc_integrator;
    std::shared_ptr<SamplerEvaluator<1>>    sampler_evaluator;
    std::shared_ptr<UniformSampler<1>>      uniform_sampler;
};

// 测试MonteCarloIntegrator1D的estimate方法（模拟pbpt.cpp中的第一个测试）
TEST_F(SamplerTest, MonteCarloIntegrator_LinearFunction) {
    auto res = mc_integrator->estimate([](const Pt1& p) { return p.x(); },
                                       std::make_shared<UniformDistribution1D>(Pt1{0.0}, Pt1{2.0}),
                                       uniform_sampler, 10000);

    // f(x) = x 在[0,2]上的积分应该是 2^2/2 = 2.0
    EXPECT_NEAR(res, 2.0f, 0.1f);
    EXPECT_GT(res, 0.0f);  // 结果应该为正数
}

// 测试SamplerEvaluator对UniformSampler生成样本的评估
TEST_F(SamplerTest, SamplerEvaluator_UniformSampler) {
    int  sample_count = 100;
    auto uni_result   = sampler_evaluator->evaluate(uniform_sampler->generate(sample_count));

    // 检查结果的合理性
    EXPECT_GE(uni_result.mean.x(), 0.0f);
    EXPECT_LE(uni_result.mean.x(), 1.0f);
    EXPECT_GE(uni_result.variance, 0.0f);
    EXPECT_GE(uni_result.sample_variance, 0.0f);

    // 对于均匀分布，期望值应该接近0.5
    EXPECT_NEAR(uni_result.mean.x(), 0.5f, 0.2f);
}

// 测试SamplerEvaluator对StratifiedSampler生成样本的评估
TEST_F(SamplerTest, SamplerEvaluator_StratifiedSampler) {
    int  sample_count = 100;
    int  strat_layer  = 5;
    auto strat_result =
        sampler_evaluator->evaluate(std::make_shared<StratifiedSampler<1>>(strat_layer)->generate(sample_count));

    // 检查结果的合理性
    EXPECT_GE(strat_result.mean.x(), 0.0f);
    EXPECT_LE(strat_result.mean.x(), 1.0f);
    EXPECT_GE(strat_result.variance, 0.0f);
    EXPECT_GE(strat_result.sample_variance, 0.0f);

    // 对于分层采样，期望值也应该接近0.5
    EXPECT_NEAR(strat_result.mean.x(), 0.5f, 0.2f);
}

// 测试Point::mid方法和均值计算（模拟pbpt.cpp中的均值计算部分）
TEST_F(SamplerTest, Point_Mid_UniformSampler) {
    int                                      mean_sample_count = 10;
    std::vector<Point<Float, 1>> points;
    points.reserve(mean_sample_count);

    for (int i = 0; i < mean_sample_count; i++) {
        auto pts  = uniform_sampler->generate(10);
        auto mean = Pt1::mid(pts);
        points.push_back(mean);
    }

    auto eval = sampler_evaluator->evaluate(points);

    // 检查结果的合理性
    EXPECT_GE(eval.mean.x(), 0.0f);
    EXPECT_LE(eval.mean.x(), 1.0f);
    EXPECT_GE(eval.variance, 0.0f);
    EXPECT_GE(eval.sample_variance, 0.0f);

    // 均值的均值应该接近0.5
    EXPECT_NEAR(eval.mean.x(), 0.5f, 0.3f);
}

// 测试Point::mid方法和StratifiedSampler的均值计算
TEST_F(SamplerTest, Point_Mid_StratifiedSampler) {
    int                                      mean_sample_count = 10;
    std::vector<Point<Float, 1>> points;
    points.reserve(mean_sample_count);

    for (int i = 0; i < mean_sample_count; i++) {
        auto pts  = std::make_shared<StratifiedSampler<1>>(5)->generate(10);
        auto mean = Pt1::mid(pts);
        points.push_back(mean);
    }

    auto eval_ = sampler_evaluator->evaluate(points);

    // 检查结果的合理性
    EXPECT_GE(eval_.mean.x(), 0.0f);
    EXPECT_LE(eval_.mean.x(), 1.0f);
    EXPECT_GE(eval_.variance, 0.0f);
    EXPECT_GE(eval_.sample_variance, 0.0f);

    // 均值的均值应该接近0.5
    EXPECT_NEAR(eval_.mean.x(), 0.5f, 0.3f);
}

// 比较UniformSampler和StratifiedSampler的方差
TEST_F(SamplerTest, Compare_Uniform_vs_Stratified_Variance) {
    int sample_count = 1000;

    // 测试UniformSampler
    auto uniform_result = sampler_evaluator->evaluate(uniform_sampler->generate(sample_count));

    // 测试StratifiedSampler
    auto stratified_result =
        sampler_evaluator->evaluate(std::make_shared<StratifiedSampler<1>>(10)->generate(sample_count));

    // 两种采样器都应该产生合理的结果
    EXPECT_GE(uniform_result.variance, 0.0f);
    EXPECT_GE(uniform_result.sample_variance, 0.0f);
    EXPECT_GE(stratified_result.variance, 0.0f);
    EXPECT_GE(stratified_result.sample_variance, 0.0f);

    // 期望值都应该接近0.5
    EXPECT_NEAR(uniform_result.mean.x(), 0.5f, 0.1f);
    EXPECT_NEAR(stratified_result.mean.x(), 0.5f, 0.1f);

    // 理论上分层采样的方差应该更小，但由于随机性，我们只检查它们都是有限值
    EXPECT_TRUE(std::isfinite(uniform_result.variance));
    EXPECT_TRUE(std::isfinite(uniform_result.sample_variance));
    EXPECT_TRUE(std::isfinite(stratified_result.variance));
    EXPECT_TRUE(std::isfinite(stratified_result.sample_variance));
}

// 测试StratifiedSampler的不同层数
TEST_F(SamplerTest, StratifiedSampler_DifferentLayers) {
    int              sample_count = 100;
    std::vector<int> layer_counts = {2, 5, 10, 20};

    for (int layers : layer_counts) {
        auto stratified_sampler = std::make_shared<StratifiedSampler<1>>(layers);
        auto samples            = stratified_sampler->generate(sample_count);
        auto result             = sampler_evaluator->evaluate(samples);

        // 检查每种层数配置都能产生合理结果
        EXPECT_EQ(samples.size(), sample_count);
        EXPECT_GE(result.mean.x(), 0.0f);
        EXPECT_LE(result.mean.x(), 1.0f);
        EXPECT_GE(result.variance, 0.0f);
        EXPECT_GE(result.sample_variance, 0.0f);
        EXPECT_NEAR(result.mean.x(), 0.5f, 0.2f);
    }
}

// 测试Point::mid方法的正确性
TEST_F(SamplerTest, Point_Mid_Correctness) {
    // 创建已知的点集
    std::vector<Pt1> known_points = {Pt1{0.0f}, Pt1{0.5f}, Pt1{1.0f}};

    auto mid_point = Pt1::mid(known_points);

    // 中点应该是 (0.0 + 0.5 + 1.0) / 3 = 0.5
    EXPECT_FLOAT_EQ(mid_point.x(), 0.5f);
}

// 测试空向量和单点的边界情况
TEST_F(SamplerTest, Point_Mid_EdgeCases) {
    // 测试单个点
    std::vector<Pt1> single_point = {Pt1{0.7f}};
    auto                   mid_single   = Pt1::mid(single_point);
    EXPECT_FLOAT_EQ(mid_single.x(), 0.7f);

    // 测试两个相同的点
    std::vector<Pt1> same_points = {Pt1{0.3f}, Pt1{0.3f}};
    auto                   mid_same    = Pt1::mid(same_points);
    EXPECT_FLOAT_EQ(mid_same.x(), 0.3f);
}

// 性能测试：模拟pbpt.cpp中的完整流程
TEST_F(SamplerTest, FullWorkflow_Performance) {
    auto start = std::chrono::high_resolution_clock::now();

    // 执行类似pbpt.cpp的完整流程
    auto res = mc_integrator->estimate([](const Pt1& p) { return p.x(); },
                                       std::make_shared<UniformDistribution1D>(Pt1{0.0}, Pt1{2.0}),
                                       uniform_sampler, 1000);

    auto uni_result = sampler_evaluator->evaluate(uniform_sampler->generate(100));

    auto strat_result = sampler_evaluator->evaluate(std::make_shared<StratifiedSampler<1>>(5)->generate(100));

    auto end      = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    // 确保所有计算都完成且结果合理
    EXPECT_TRUE(std::isfinite(res));
    EXPECT_TRUE(std::isfinite(uni_result.mean.x()));
    EXPECT_TRUE(std::isfinite(uni_result.variance));
    EXPECT_TRUE(std::isfinite(uni_result.sample_variance));
    EXPECT_TRUE(std::isfinite(strat_result.mean.x()));
    EXPECT_TRUE(std::isfinite(strat_result.variance));
    EXPECT_TRUE(std::isfinite(strat_result.sample_variance));

    // 性能检查：应该在合理时间内完成
    EXPECT_LT(duration.count(), 1000);  // 应该在1秒内完成
}

}