#pragma once

#include "lds_sampler.hpp"
#include "math/random_generator.hpp"
#include "math/point.hpp"


namespace pbpt::lds {

template<typename T>
class IndependentSampler : public LowDescrepencySequenceSampler<IndependentSampler<T>, T> {
    friend class LowDescrepencySequenceSampler<IndependentSampler<T>, T>;
private:
    math::RandomGenerator<T, 1> m_rng1d;
    math::RandomGenerator<T, 2> m_rng2d;

public:
    IndependentSampler() = default;

private:    
    T next_1d_impl() {
        return m_rng1d.generate_uniform();
    }

    math::Point<T, 2> next_2d_impl() {
        return math::Point<T, 2>::from_array(m_rng2d.generate_uniform());
    }

};

}
