#pragma once

#include <variant>
#include "aggregate/aggregate.hpp"
#include "aggregate/embree_aggregate.hpp"

namespace pbpt::aggregate {

// Aggregate Variant (如果需要切换加速结构)
template<typename T>
using AnyAggregate = std::variant<
    aggregate::LinearAggregate<T>,
    aggregate::EmbreeAggregate<T>
>;
};