#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>

#include "math/point.hpp"
#include "math/type_alias.hpp"
#include "integrator/distribution.hpp"
#include "integrator/integrator.hpp"
#include "integrator/sampler.hpp"

using namespace pbpt::math;
using namespace pbpt::integrator;

namespace pbpt::integrator::testing {


class IntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 设置测试环境
        uniform_dist    = std::make_shared<UniformDistribution1D>(Pt1{0.0}, Pt1{1.0});
        uniform_sampler = std::make_shared<UniformSampler<1>>();
        integrator      = std::make_unique<MonteCarloIntegrator1D>();
    }

    std::shared_ptr<UniformDistribution1D>  uniform_dist;
    std::shared_ptr<UniformSampler<1>>      uniform_sampler;
    std::unique_ptr<MonteCarloIntegrator1D> integrator;
};

// 测试UniformDistribution1D类
TEST_F(IntegrationTest, UniformDistribution1D_Sample) {
    // 测试采样功能
    Pt1 u{0.5};
    Pt1 sample = uniform_dist->sample(u);

    // 对于[0,1]区间的均匀分布，u=0.5应该返回0.5
    EXPECT_FLOAT_EQ(sample.x(), 0.5f);

    // 测试边界情况
    Pt1 u_min{0.0};
    Pt1 sample_min = uniform_dist->sample(u_min);
    EXPECT_FLOAT_EQ(sample_min.x(), 0.0f);

    Pt1 u_max{1.0};
    Pt1 sample_max = uniform_dist->sample(u_max);
    EXPECT_FLOAT_EQ(sample_max.x(), 1.0f);
}

TEST_F(IntegrationTest, UniformDistribution1D_PDF) {
    // 测试概率密度函数
    Pt1   p{0.5};
    Float pdf_value = uniform_dist->pdf(p);

    // 对于[0,1]区间的均匀分布，PDF应该是1.0
    EXPECT_FLOAT_EQ(pdf_value, 1.0f);

    // 测试不同点的PDF值（应该都相同）
    Pt1 p1{0.1};
    Pt1 p2{0.9};
    EXPECT_FLOAT_EQ(uniform_dist->pdf(p1), uniform_dist->pdf(p2));
}

TEST_F(IntegrationTest, UniformDistribution1D_CustomRange) {
    // 测试自定义范围的均匀分布
    auto custom_dist = std::make_shared<UniformDistribution1D>(Pt1{2.0}, Pt1{5.0});

    Pt1 u{0.5};
    Pt1 sample = custom_dist->sample(u);
    EXPECT_FLOAT_EQ(sample.x(), 3.5f);  // 2 + 0.5 * (5-2) = 3.5

    Float pdf_value = custom_dist->pdf(Pt1{3.0});
    EXPECT_FLOAT_EQ(pdf_value, 1.0f / 3.0f);  // 1/(5-2) = 1/3
}

// 测试UniformSampler类
TEST_F(IntegrationTest, UniformSampler_Generate) {
    // 测试采样器生成的随机数范围
    const int num_samples   = 1000;
    int       valid_samples = 0;
    auto      samples       = uniform_sampler->generate(num_samples);

    for (int i = 0; i < num_samples; ++i) {
        Pt1 sample = samples[i];
        if (sample.x() >= 0.0f && sample.x() <= 1.0f) {
            valid_samples++;
        }
    }

    // 所有样本都应该在[0,1]范围内
    EXPECT_EQ(valid_samples, num_samples);
}

TEST_F(IntegrationTest, UniformSampler_Randomness) {
    // 测试采样器的随机性（简单统计测试）
    const int num_samples = 10000;
    Float     sum         = 0.0f;

    auto samples = uniform_sampler->generate(num_samples);

    for (int i = 0; i < num_samples; ++i) {
        auto u = samples[i];
        sum += u.x();
    }

    Float mean = sum / num_samples;
    // 均匀分布[0,1]的期望值应该接近0.5
    EXPECT_NEAR(mean, 0.5f, 0.05f);  // 允许5%的误差
}

// 测试MonteCarloIntegrator类
TEST_F(IntegrationTest, MonteCarloIntegrator_ConstantFunction) {
    // 测试常数函数的积分
    auto constant_func = [](const Pt1& p) -> Float {
        return 2.0f;  // 常数函数 f(x) = 2
    };

    Float result = integrator->estimate(constant_func, uniform_dist, uniform_sampler, 1000);

    // 常数函数在[0,1]上的积分应该等于常数值
    EXPECT_NEAR(result, 2.0f, 0.1f);
}

TEST_F(IntegrationTest, MonteCarloIntegrator_LinearFunction) {
    // 测试线性函数的积分
    auto linear_func = [](const Pt1& p) -> Float {
        return p.x();  // 线性函数 f(x) = x
    };

    Float result = integrator->estimate(linear_func, uniform_dist, uniform_sampler, 5000);

    // f(x) = x 在[0,1]上的积分应该是 0.5
    EXPECT_NEAR(result, 0.5f, 0.05f);
}

TEST_F(IntegrationTest, MonteCarloIntegrator_QuadraticFunction) {
    // 测试二次函数的积分
    auto quadratic_func = [](const Pt1& p) -> Float {
        return p.x() * p.x();  // 二次函数 f(x) = x^2
    };

    Float result = integrator->estimate(quadratic_func, uniform_dist, uniform_sampler, 10000);

    // f(x) = x^2 在[0,1]上的积分应该是 1/3
    EXPECT_NEAR(result, 1.0f / 3.0f, 0.02f);
}

TEST_F(IntegrationTest, MonteCarloIntegrator_SinFunction) {
    // 测试三角函数的积分
    auto sin_func = [](const Pt1& p) -> Float {
        return std::sin(p.x() * M_PI);  // f(x) = sin(πx)
    };

    Float result = integrator->estimate(sin_func, uniform_dist, uniform_sampler, 10000);

    // sin(πx) 在[0,1]上的积分应该是 2/π
    EXPECT_NEAR(result, 2.0f / M_PI, 0.05f);
}

// 测试积分器的收敛性
TEST_F(IntegrationTest, MonteCarloIntegrator_Convergence) {
    auto simple_func = [](const Pt1& p) -> Float {
        return 1.0f;  // f(x) = 1
    };

    // 测试不同样本数量的收敛性
    std::vector<int>   sample_counts = {100, 1000, 10000};
    std::vector<Float> results;

    for (int count : sample_counts) {
        Float result = integrator->estimate(simple_func, uniform_dist, uniform_sampler, count);
        results.push_back(result);
    }

    // 随着样本数增加，结果应该更接近真实值1.0
    for (Float result : results) {
        EXPECT_NEAR(result, 1.0f, 0.2f);
    }
}

// 测试多维采样器
TEST(IntegrationMultiDimTest, UniformSampler2D) {
    UniformSampler<2> sampler_2d;

    const int num_samples   = 1000;
    int       valid_samples = 0;

    auto samples = sampler_2d.generate(num_samples);

    for (int i = 0; i < num_samples; ++i) {
        Point<Float, 2> sample = samples[i];
        if (sample[0] >= 0.0f && sample[0] <= 1.0f && sample[1] >= 0.0f && sample[1] <= 1.0f) {
            valid_samples++;
        }
    }

    EXPECT_EQ(valid_samples, num_samples);
}

TEST(IntegrationMultiDimTest, UniformSampler3D) {
    UniformSampler<3> sampler_3d;

    const int num_samples   = 1000;
    int       valid_samples = 0;

    auto samples = sampler_3d.generate(num_samples);

    for (int i = 0; i < num_samples; ++i) {
        Point<Float, 3> sample = samples[i];
        if (sample[0] >= 0.0f && sample[0] <= 1.0f && sample[1] >= 0.0f && sample[1] <= 1.0f && sample[2] >= 0.0f &&
            sample[2] <= 1.0f) {
            valid_samples++;
        }
    }

    EXPECT_EQ(valid_samples, num_samples);
}

// 性能测试
TEST_F(IntegrationTest, Performance_LargeScale) {
    auto complex_func = [](const Pt1& p) -> Float { return std::exp(-p.x()) * std::sin(10 * p.x()); };

    auto  start  = std::chrono::high_resolution_clock::now();
    Float result = integrator->estimate(complex_func, uniform_dist, uniform_sampler, 100000);
    auto  end    = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    // 确保计算完成且在合理时间内
    EXPECT_TRUE(std::isfinite(result));
    EXPECT_LT(duration.count(), 5000);  // 应该在5秒内完成
}

}