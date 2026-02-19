#pragma once

#include <concepts>

namespace pbpt::lds {

template <typename SamplerT, typename T>
concept Sampler1D2DConcept = std::default_initializable<SamplerT> && requires(SamplerT sampler) {
    { sampler.next_1d() } -> std::convertible_to<T>;
    sampler.next_2d();
};

}  // namespace pbpt::lds
