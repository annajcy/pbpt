#pragma once

#include <variant>
#include "embree_aggregate.hpp"
#include "linear_aggregate.hpp"

namespace pbpt::aggregate {

// Aggregate Variant (如果需要切换加速结构)
template<typename T>
using AnyAggregate = std::variant<
    aggregate::LinearAggregate<T>,
    aggregate::EmbreeAggregate<T>
>;

};
