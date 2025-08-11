#pragma once

#include <concepts>
#include <stdexcept>
#include <type_traits>

#include "type_alias.hpp"

namespace pbpt::math {

template <typename T>
using promote_int_to_float_t = std::conditional_t<std::is_integral_v<T>, Float, T>;

template <typename F>
concept AssertLambda = requires(F f) {
    { f() } -> std::convertible_to<bool>;
};

template <typename F>
    requires AssertLambda<F>
inline constexpr void assert_if(F condition_lambda, const char* message = "Assertion failed") {
    if (condition_lambda()) {
        if (std::is_constant_evaluated()) {
            throw message;
        } else {
            throw std::runtime_error(message);
        }
    }
}

template<typename F>
concept AssertCondition = std::is_convertible_v<F, bool>;

template <typename F>
    requires AssertCondition<F>
inline constexpr void assert_if(F condition_condition, const char* message = "Assertion failed") {
    if (static_cast<bool>(condition_condition)) {
        if (std::is_constant_evaluated()) {
            throw message;
        } else {
            throw std::runtime_error(message);
        }
    }
}



};  // namespace pbpt::math