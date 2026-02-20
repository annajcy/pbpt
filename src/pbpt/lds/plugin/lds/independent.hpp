#pragma once

#include <cstdint>

#include "pbpt/lds/lds_sampler.hpp"
#include "pbpt/math/random_generator.hpp"
#include "pbpt/math/point.hpp"

namespace pbpt::lds {

template <typename T>
class IndependentSampler : public LowDescrepencySequenceSampler<IndependentSampler<T>, T> {
    friend class LowDescrepencySequenceSampler<IndependentSampler<T>, T>;

private:
    math::RandomGenerator<T, 1> m_rng1d;
    math::RandomGenerator<T, 2> m_rng2d;
    std::uint64_t m_base_seed{0};
public:
    IndependentSampler() = default;

    explicit IndependentSampler(std::uint64_t base_seed)
        : m_rng1d(static_cast<unsigned int>(base_seed)),
          m_rng2d(static_cast<unsigned int>(base_seed + 1)),
          m_base_seed(base_seed) {}

    std::uint64_t base_seed() const { return m_base_seed; }

    /// Clone with a deterministic seed derived from base_seed and stream_id.
    /// Same (base_seed, stream_id) pair produces the same sequence → reproducible.
    /// Different stream_ids produce different sequences → independent per pixel.
    IndependentSampler clone(std::uint64_t stream_id) const {
        // Mix base_seed and stream_id to produce a deterministic per-stream seed.
        const std::uint64_t derived_seed = m_base_seed ^ (stream_id * 0x9E3779B97F4A7C15ULL + 0x6C62272E07BB0142ULL);
        return IndependentSampler(derived_seed);
    }

private:
    T next_1d_impl() { return m_rng1d.generate_uniform(); }
    math::Point<T, 2> next_2d_impl() { return math::Point<T, 2>::from_array(m_rng2d.generate_uniform()); }
};

}  // namespace pbpt::lds
