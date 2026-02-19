#pragma once

#include <chrono>
#include <cstddef>
#include <functional>
#include <optional>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace pbpt::utils {

template <typename Duration, typename Result>
struct TimedExecutionResult {
    Duration duration{};
    Result value;
};

template <typename Duration>
struct TimedExecutionResult<Duration, void> {
    Duration duration{};
};

template <typename Duration, typename Result>
struct TimedBatchExecutionResult {
    Duration total_duration{};
    Duration average_duration{};
    std::size_t iterations{};
    Result value;
};

template <typename Duration>
struct TimedBatchExecutionResult<Duration, void> {
    Duration total_duration{};
    Duration average_duration{};
    std::size_t iterations{};
};

template <
    typename Duration = std::chrono::microseconds,
    typename Clock = std::chrono::steady_clock,
    typename Callable,
    typename... Args>
auto measure_once(Callable&& callable, Args&&... args) {
    using Result = std::invoke_result_t<Callable, Args...>;

    const auto start = Clock::now();

    if constexpr (std::is_void_v<Result>) {
        std::invoke(std::forward<Callable>(callable), std::forward<Args>(args)...);
        const auto end = Clock::now();
        return TimedExecutionResult<Duration, void>{std::chrono::duration_cast<Duration>(end - start)};
    } else {
        Result value = std::invoke(std::forward<Callable>(callable), std::forward<Args>(args)...);
        const auto end = Clock::now();
        return TimedExecutionResult<Duration, Result>{std::chrono::duration_cast<Duration>(end - start), std::move(value)};
    }
}

template <
    typename Duration = std::chrono::microseconds,
    typename Clock = std::chrono::steady_clock,
    typename Callable,
    typename... Args>
auto measure_n(std::size_t iterations, Callable&& callable, Args&&... args) {
    if (iterations == 0) {
        throw std::invalid_argument("measure_n requires iterations > 0");
    }

    using Result = std::invoke_result_t<Callable&, Args&...>;

    const auto start = Clock::now();

    if constexpr (std::is_void_v<Result>) {
        for (std::size_t i = 0; i < iterations; ++i) {
            std::invoke(callable, args...);
        }
        const auto end = Clock::now();
        const auto total = std::chrono::duration_cast<Duration>(end - start);
        return TimedBatchExecutionResult<Duration, void>{
            total,
            total / static_cast<typename Duration::rep>(iterations),
            iterations
        };
    } else {
        std::optional<Result> value;
        for (std::size_t i = 0; i < iterations; ++i) {
            value = std::invoke(callable, args...);
        }
        const auto end = Clock::now();
        const auto total = std::chrono::duration_cast<Duration>(end - start);
        return TimedBatchExecutionResult<Duration, Result>{
            total,
            total / static_cast<typename Duration::rep>(iterations),
            iterations,
            std::move(*value),
        };
    }
}

}  // namespace pbpt::utils
