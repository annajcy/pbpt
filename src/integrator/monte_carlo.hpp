#pragma once

#include <vector>
#include <random>
#include <span>
#include <concepts>
#include <utility>
#include <numeric>
#include <cmath>
#include <cassert>

namespace pbpt::integrator {

class UniformSampler {
public:
    UniformSampler(int dim, std::pair<double, double> range = {0.0, 1.0})
        : m_dim(dim), m_range(range), m_dist(range.first, range.second) {}

    std::vector<double> operator()() {
        std::vector<double> sample(m_dim);
        for (int i = 0; i < m_dim; ++i)
            sample[i] = m_dist(m_rng);
        return sample;
    }

    double pdf() const {
        double len = m_range.second - m_range.first;
        return std::pow(len, -m_dim); // 多维均匀PDF = 1 / 区域体积
    }

private:
    int m_dim;
    std::pair<double, double> m_range;
    std::uniform_real_distribution<double> m_dist;
    std::mt19937 m_rng{std::random_device{}()};
};

// ========== Monte Carlo 积分核心模块 ==========
template <typename Func, typename Sampler>
requires std::invocable<Func, std::span<const double>>
auto monte_carlo_integrate(Func f, Sampler& sampler, int N)
{
    using Real = decltype(f(std::span<const double>{}));

    std::vector<Real> values;
    values.reserve(N);
    double pdf = sampler.pdf();

    for (int i = 0; i < N; ++i) {
        auto x = sampler();           // std::vector<double>
        Real val = f(x) / pdf;        // Monte Carlo Estimator
        values.push_back(val);
    }

    // 计算平均值
    Real mean = std::accumulate(values.begin(), values.end(), Real{0}) / N;

    // 样本方差（Bessel 校正）
    Real variance = 0;
    for (const auto& v : values)
        variance += (v - mean) * (v - mean);
    variance /= (N - 1);

    return std::pair{mean, variance};
}

} // namespace pbpt::integrator
