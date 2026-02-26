#pragma once
#include "pbpt/light_sampler/plugin/light_sampler/uniform_light_sampler.hpp"
#include <variant>

namespace pbpt::light_sampler {

template <typename T>
using AnyLightSampler = std::variant<UniformLightSampler<T>>;

}  // namespace pbpt::light_sampler
