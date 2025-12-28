/**
 * @file
 * @brief Sampling routines for 1D/2D intervals, disks, spheres and hemispheres.
 */
#pragma once

#include <array>
#include "math/function.hpp"
#include "math/point.hpp"
#include "math/vector.hpp"

namespace pbpt::sampler {

/**
 * @brief Generates a stratifed array of N samples in [0, 1).
 *
 * This function divides the interval [0, 1) into N equal strata and places
 * one sample in each stratum. The input u is used to offset the samples
 * within their respective strata to avoid clustering at the edges.
 *
 * @tparam T Numeric type.
 * @tparam N Number of strata/samples.
 * @param u A uniform random variable in [0, 1) used for offsetting.
 * @return std::array<T, N> An array of N stratified samples in [0, 1).
 */
template<typename T, int N>
inline std::array<T, N> generate_strified_array(T u) {
    std::array<T, N> us;
    for (int i = 0; i < N; ++i) {
        T up = u + T(i) / T(N);
        if (up >= T(1)) {
            up -= T(1);
        }
        us[i] = up;
    }
    return us;
}

}
