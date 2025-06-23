#pragma once
#include <cmath>
#include <stdexcept>
#include <type_traits>

namespace pbpt {
namespace math {

namespace detail {

// 使用牛顿法实现的 constexpr sqrt
template<typename T>
constexpr T sqrt_newton_raphson(T x, T curr, T prev) {
    if (curr == prev) return curr;
    T next = 0.5 * (curr + x / curr);
    return sqrt_newton_raphson(x, next, curr);
}

} // namespace detail

template<typename T>
constexpr T abs(T x) {
    if (x < 0) {
        return -x;
    }
    return x;
}

template<typename T>
constexpr T sqrt(T x) {
    if (x < 0) {
        // 在编译时和运行时都能触发错误
        if (std::is_constant_evaluated()) {
            throw "Compile-time error: sqrt of negative number";
        } else {
            // 在运行时可以抛出更具体的异常
            // return std::sqrt(x); // 这会产生 NaN
            throw std::runtime_error("sqrt of negative number");
        }
    }
    
    if (x == 0) {
        return 0;
    }

    if (std::is_constant_evaluated()) {
        // 编译时: 调用我们自己的 constexpr 实现
        return detail::sqrt_newton_raphson(x, x, T(0));
    } else {
        // 运行时: 调用标准库的高度优化的实现
        return std::sqrt(x);
    }
}

} // namespace math
} // namespace pbpt