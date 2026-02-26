#pragma once

#include <concepts>
#include "pbpt/light/plugin/light/light_type.hpp"
#include "pbpt/light_sampler/light_sampler.hpp"

namespace pbpt::light_sampler {

template <typename SamplerT, typename T>
concept LightSamplerConcept = requires(const SamplerT const_sampler, T u, int index) {
    { const_sampler.sample(u) } -> std::same_as<LightSamplerSampleResult<T>>;
    { const_sampler.pdf(index) } -> std::same_as<T>;
    { const_sampler.light_count() } -> std::same_as<int>;
};

}  // namespace pbpt::light_sampler
