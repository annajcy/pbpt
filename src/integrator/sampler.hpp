#pragma once

#include <random>
#include <stdexcept>
#include <utility>
#include <vector>

#include "math/point.hpp"
#include "math/type_alias.hpp"

namespace pbpt::integrator {

using namespace pbpt::math;

/**
 * @brief Base class for samplers that generate random points
 * @tparam N Dimension of the sample space
 */
template <int N>
class Sampler {
public:
    virtual ~Sampler()                                   = default;
    virtual std::vector<Point<Float, N>> generate(int n) = 0;
    virtual void                         generate_into(std::vector<Point<Float, N>>& samples, int n) {
        if (n <= 0) {
            throw std::invalid_argument("Sample count must be positive");
        }
        samples.clear();
        samples.reserve(n);
        auto generated = generate(n);
        samples        = std::move(generated);
    }
};

template <int N>
class SamplerEvaluator {
public:
    struct EvaluationResult {
        Point<Float, N> mean;
        Float           variance;
        Float           sample_variance;
    };

    static EvaluationResult evaluate(const std::vector<Point<Float, N>>& points) {
        if (points.empty()) {
            throw std::invalid_argument("Cannot evaluate empty point set");
        }

        auto  mean     = Point<Float, N>::mid(points);
        Float variance = 0.0;

        for (int i = 0; i < points.size(); i++) {
            variance += (points[i] - mean).dot(points[i] - mean);
        }

        Float population_variance = variance / points.size();
        Float sample_variance     = points.size() > 1 ? variance / (points.size() - 1) : 0.0;

        return EvaluationResult{mean, population_variance, sample_variance};
    }
};

template <int N>
class UniformSampler : public Sampler<N> {
private:
    mutable std::mt19937                          m_generator;
    mutable std::uniform_real_distribution<Float> m_distribution{0.0, 1.0};

public:
    explicit UniformSampler(unsigned int seed = std::random_device{}()) : m_generator(seed) {}
    ~UniformSampler() = default;

    Point<Float, N> generate_point() const {
        Point<Float, N> p;
        for (int j = 0; j < N; ++j) {
            p[j] = m_distribution(m_generator);
        }
        return p;
    }

    std::vector<Point<Float, N>> generate(int n) override {
        if (n <= 0) {
            throw std::invalid_argument("Sample count must be positive");
        }
        std::vector<Point<Float, N>> points;
        points.reserve(n);
        for (int i = 0; i < n; ++i) {
            points.push_back(generate_point());
        }
        return points;
    }

    void generate_into(std::vector<Point<Float, N>>& samples, int n) override {
        if (n <= 0) {
            throw std::invalid_argument("Sample count must be positive");
        }

        samples.clear();
        samples.reserve(n);
        for (int i = 0; i < n; ++i) {
            samples.push_back(generate_point());
        }
    }
};

template <int N>
class StratifiedSampler : public Sampler<N> {
private:
    int               m_layer_count;
    UniformSampler<N> m_uniform_sampler;

public:
    explicit StratifiedSampler(int layer_count, unsigned int seed = std::random_device{}())
        : m_layer_count(layer_count), m_uniform_sampler(seed) {
        if (layer_count <= 0) {
            throw std::invalid_argument("Layer count must be positive");
        }
    }

    ~StratifiedSampler() = default;

    Float layer_width() const { return 1.0 / m_layer_count; }

    std::pair<Float, Float> layer_interval(int layer_num) const {
        if (layer_num < 0 || layer_num >= m_layer_count) {
            throw std::invalid_argument("Layer number out of range");
        }
        return std::make_pair(layer_num * layer_width(), (layer_num + 1) * layer_width());
    }

    std::vector<Point<Float, N>> generate(int n) override {
        if (n <= 0) {
            throw std::invalid_argument("Sample count must be positive");
        }

        std::vector<Point<Float, N>> points;
        points.reserve(n);

        for (int i = 0; i < n; i++) {
            int layer_id = i % m_layer_count;
            auto [a, b]  = layer_interval(layer_id);
            auto u       = m_uniform_sampler.generate_point();

            Point<Float, N> p;
            for (int j = 0; j < N; ++j) {
                p[j] = a + (b - a) * u[j];
            }
            points.push_back(p);
        }
        return points;
    }

    void generate_into(std::vector<Point<Float, N>>& samples, int n) override {
        if (n <= 0) {
            throw std::invalid_argument("Sample count must be positive");
        }

        samples.clear();
        samples.reserve(n);

        for (int i = 0; i < n; i++) {
            int layer_id = i % m_layer_count;
            auto [a, b]  = layer_interval(layer_id);
            auto u       = m_uniform_sampler.generate_point();

            Point<Float, N> p;
            for (int j = 0; j < N; ++j) {
                p[j] = a + (b - a) * u[j];
            }
            samples.push_back(p);
        }
    }

    int layer_count() const { return m_layer_count; }
};

};  // namespace pbpt::integrator