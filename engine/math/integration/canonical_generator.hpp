#pragma once

#include "math/global/type_alias.hpp"
#include "math/geometry/point.hpp"
#include <array>
#include <memory>
#include <random>
#include <vector>
#include <stdexcept>

namespace pbpt::math {

/**
 * @brief 规范样本生成器接口
 * 负责在[0,1]^N规范空间中生成高质量的均匀分布样本
 * 这是采样流程的第一层：生成器层
 * @tparam N 样本空间的维度
 */
template<typename T, int N>
requires std::floating_point<T> && (N > 0)
class CanonicalGenerator {
protected:
    mutable std::mt19937 m_generator;
    mutable std::uniform_real_distribution<T> m_distribution{0.0, 1.0};
    
public:
    explicit CanonicalGenerator(unsigned int seed = std::random_device{}()) : m_generator(seed) {}
    virtual ~CanonicalGenerator() = default;
    virtual std::vector<Point<T, N>> generate(int count) = 0;

protected:
    Point<T, N> generate_once() {
        Point<T, N> p;
        for (int i = 0; i < N; ++i) {
            p[i] = m_distribution(m_generator);
        }
        return p;
    }
};

template<typename T, int N>
requires std::floating_point<T> && (N > 0)
class RandomGenerator : public CanonicalGenerator<T, N> {
public:
    explicit RandomGenerator(unsigned int seed = std::random_device{}()) : CanonicalGenerator<T, N>(seed) {}
    virtual ~RandomGenerator() = default;
    std::vector<Point<T, N>> generate(int count) override {
        std::vector<Point<T, N>> samples;
        samples.reserve(count);
        for (int i = 0; i < count; ++i) {
            samples.push_back(this->generate_once());
        }
        return samples;
    }
};


using RandomGenerator1D = RandomGenerator<Float, 1>;
using RandomGenerator2D = RandomGenerator<Float, 2>;
using RandomGenerator3D = RandomGenerator<Float, 3>;

} // namespace pbpt::math