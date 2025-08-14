#pragma once

#include <concepts>
#include <algorithm>
#include <limits>
#include <optional>
#include "math/global/operator.hpp"

namespace pbpt::math {

template <std::floating_point T>
class Interval {
public:
    T low, high;

    // 默认构造：0 区间
    constexpr Interval<T>() noexcept : low(0), high(0) {}

    // 单值构造
    constexpr Interval<T>(T v) noexcept : low(v), high(v) {}

    // 显式区间构造
    constexpr Interval<T>(T l, T h) noexcept
        : low(std::min(l, h)), high(std::max(l, h)) {}

    // 工厂函数：从值 ± ε 创建
    static constexpr Interval<T> from_value_with_error(T v, T eps) noexcept {
        return Interval(v - eps, v + eps);
    }

    constexpr T midpoint() const noexcept {
        return (low + high) * T(0.5);
    }

    constexpr T width() const noexcept {
        return high - low;
    }

    constexpr bool contains(T v) const noexcept {
        return is_greater_equal(v, low) && is_less_equal(v, high);
    }

    constexpr bool is_empty() const noexcept {
        return is_greater_equal(low, high);
    }

    constexpr bool is_intersecting(const Interval& other) const noexcept {
        return is_greater_equal(high, other.low) && is_less_equal(low, other.high);
    }


    template<typename U>
    constexpr Interval<U> united(const Interval<U>& other) const noexcept {
        using R = std::common_type_t<T, U>;
        return Interval<R>(std::min<R>(low, other.low), std::max<R>(high, other.high));
    }

    template<typename U>
    constexpr Interval<U> intersected(const Interval<U>& other) const noexcept {
        using R = std::common_type_t<T, U>;
        if (!is_intersecting(other)) {
            return Interval<R>();
        }
        return Interval<R>(std::max<R>(low, other.low), std::min<R>(high, other.high));
    }

};

} // namespace pbpt::math
