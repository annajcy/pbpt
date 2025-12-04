/**
 * @file
 * @brief Wrapper over std::mt19937 for generating N-dimensional uniform and normal samples.
 */
#pragma once

#include <concepts>
#include <random>

namespace pbpt::math {

/**
 * @brief Simple wrapper around `std::mt19937` for N-dimensional sampling.
 *
 * Generates arrays of uniformly or normally distributed random numbers
 * of fixed compile-time dimension.
 *
 * @tparam T Scalar type.
 * @tparam N Number of random components per sample.
 */
template<typename T, int N>
class RandomGenerator { 
private:
    /// Underlying Mersenne Twister engine.
    std::mt19937 engine{};

public:
    /// Number of components generated per call.
    static constexpr int dimensions = N;

    /// Constructs a generator seeded from `std::random_device`.
    RandomGenerator() : engine(std::random_device{}()) {}

    /// Constructs a generator with an explicit seed.
    RandomGenerator(unsigned int seed) : engine(seed) {}

    /**
     * @brief Generates a vector of N uniform samples in [min, max].
     */
    std::array<T, N> generate_uniform(T min, T max) {
        std::uniform_real_distribution<T> dist(min, max);
        std::array<T, N> result;
        for (auto& val : result) {
            val = dist(engine);
        }
        return result;
    }

    /**
     * @brief Generates a vector of N normal samples with given mean and stddev.
     */
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
