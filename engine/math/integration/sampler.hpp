#pragma once

#include "math/global/type_alias.hpp"
#include "math/geometry/point.hpp"
#include <memory>
#include <random>
#include <utility>
#include <vector>

namespace pbpt::math {

template<int N>
class Sampler {
public:
    virtual ~Sampler() = default;
    virtual std::vector<Point<Float, N>> generate(int n) = 0;
};

template<int N>
class SamplerEvaluator {
public:

    struct EvaluationResult {
        Point<Float, N> mean;
        Float variance;
    };

    static EvaluationResult evaluate(const std::vector<Point<Float, N>>& points) {
        auto mean = Point<Float, N>::mid(points);
        Float variance = 0.0;
        for (int i = 0; i < points.size(); i ++) {
            variance += (points[i] - mean).dot(points[i] - mean);
        }
        variance /= points.size();
        return EvaluationResult{mean, variance};
    }

};

template<int N>
class UniformSampler : public Sampler<N> {
private:
    std::random_device m_rd{};
    std::mt19937 m_generator{m_rd()};
    std::uniform_real_distribution<Float> m_distribution{0.0, 1.0};

public:
    UniformSampler() = default;
    ~UniformSampler() = default;

    Point<Float, N> generate_p() {
        Point<Float, N> p;
        for (int j = 0; j < N; ++j) {
            p[j] = m_distribution(m_generator);
        }
        return p;
    }

public:

    std::vector<Point<Float, N>> generate(int n) override {
        std::vector<Point<Float, N>> points;
        points.reserve(n);
        for (int i = 0; i < n; ++i) {
            points.push_back(generate_p());
        }
        return points;
    }
};

template<int N>
class StratifiedSampler : public Sampler<N> {
private:
    int m_layer_count;
    UniformSampler<N> uniform_sampler;

public:
    StratifiedSampler(int layer_count) : m_layer_count(layer_count) {}
    ~StratifiedSampler() = default;

    Float layer_width() const {
        return 1.0 / m_layer_count;
    }

    std::pair<Float, Float> layer_interval(int layer_num) const {
        return std::make_pair(
            layer_num * layer_width(),
            (layer_num + 1) * layer_width()
        );
    }

    std::vector<Point<Float, N>> generate(int n) override {
        std::vector<Point<Float, N>> points;
        points.reserve(n);
        for (int i = 0; i < n; i ++) {
            int layer_id = i % m_layer_count;
            auto [a, b] = layer_interval(layer_id);
            auto u = uniform_sampler.generate_p();
            Point<Float, N> p;
            for (int j = 0; j < N; ++j) {
                p[j] = a + (b - a) * u[j];
            }
            points.push_back(p);
        }
        return points;
    }

};

};