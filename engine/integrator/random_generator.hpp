#pragma once

#include <concepts>
#include <random>

namespace pbpt::math {

template<std::floating_point T, int N>
class RandomGenerator { 
private:
    std::mt19937 engine{};

public:
    static constexpr int dimensions = N;
    RandomGenerator() : engine(std::random_device{}()) {}
    RandomGenerator(unsigned int seed) : engine(seed) {}

    std::array<T, N> generate_uniform(T min, T max) {
        std::uniform_real_distribution<T> dist(min, max);
        std::array<T, N> result;
        for (auto& val : result) {
            val = dist(engine);
        }
        return result;
    }

    std::array<T, N> generate_normal(T mean, T stddev) {
        std::normal_distribution<T> dist(mean, stddev);
        std::array<T, N> result;
        for (auto& val : result) {
            val = dist(engine);
        }
        return result;
    }
};

}; 