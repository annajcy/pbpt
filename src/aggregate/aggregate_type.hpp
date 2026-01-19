#pragma once

#include <variant>
#include "aggregate/aggregate.hpp"

namespace pbpt::aggregate {

// Aggregate Variant (如果需要切换加速结构)
template<typename T>
using AnyAggregate = std::variant<
    aggregate::LinearAggregate<T>
    // 将来可以添加: aggregate::BVH<T> 等
>;
};