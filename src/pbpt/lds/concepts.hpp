#pragma once

#include <concepts>
#include <cstdint>

namespace pbpt::lds {

template <typename SamplerT, typename T>
concept Sampler1D2DConcept = std::default_initializable<SamplerT> && requires(SamplerT sampler) {
    { sampler.next_1d() } -> std::convertible_to<T>;
    sampler.next_2d();
};

template <typename SamplerT, typename T>
concept CloneableSamplerConcept = Sampler1D2DConcept<SamplerT, T> && requires(const SamplerT sampler) {
    { sampler.clone(std::uint64_t{0}) } -> std::same_as<SamplerT>;
};

}  // namespace pbpt::lds
