#pragma once

#include "math/global/type_alias.hpp"
#include "math/geometry/point.hpp"
#include <random>

namespace pbpt::math {

template<int N>
class Sampler {
public:
    virtual ~Sampler() = default;
    virtual Point<Float, N> generate() = 0;
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

    Point<Float, N> generate() override {
        Point<Float, N> p;
        for (int i = 0; i < N; ++i) {
            p[i] = m_distribution(m_generator);
        }
        return p;
    }
};

};