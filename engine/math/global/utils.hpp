#pragma once

#include "type_alias.hpp"

#include <concepts>
#include <type_traits>
#include <stdexcept>

namespace pbpt::math {
    
template<typename T>
using promote_scalar_t = std::conditional_t<std::is_integral_v<T>, Float, T>;

template<typename F>
concept AssertCondition = requires(F f) {
    { f() } -> std::convertible_to<bool>;
};

template <typename F>
requires AssertCondition<F>
inline constexpr void assert_if(F condition_lambda, const char* message = "Assertion failed") {
    if (condition_lambda()) {
        if (std::is_constant_evaluated()) {
            throw message;
        } else {
            throw std::runtime_error(message);
        }
    }
}

};