#pragma once

#include "math/global/type_alias.hpp"
#include "math/geometry/point.hpp"
#include <cmath>
#include <stdexcept>

namespace pbpt::math {

template<typename T, int N>
requires std::floating_point<T> && (N > 0)
class DistributionMapper {
public:
    virtual ~DistributionMapper() = default;
    virtual std::vector<Point<T, N>> map(const std::vector<Point<T, N>>& canonical_samples) = 0;
};


} // namespace pbpt::math