#pragma once

#include <variant>

#include "pbpt/lds/plugin/lds/independent.hpp"

namespace pbpt::lds {

template <typename T>
using AnySampler = std::variant<IndependentSampler<T>>;

}  // namespace pbpt::lds
