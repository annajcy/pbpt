#pragma once

#include <chrono>
#include <cstddef>
#include <functional>
#include <iostream>
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

template <typename Duration>
struct RunningProfilerStats {
    std::size_t sample_count{};
    Duration last_duration{};
    Duration min_duration{};
    Duration max_duration{};
    Duration mean_duration{};
    Duration ema_duration{};
};

template <
    typename Duration = std::chrono::nanoseconds,
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
    typename Duration = std::chrono::nanoseconds,
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

template <typename Duration = std::chrono::nanoseconds, typename Clock = std::chrono::steady_clock>
class RunningProfiler {
private:
    using FloatingDuration = std::chrono::duration<long double, typename Duration::period>;

    std::size_t m_sample_count{};
    Duration m_last_duration{};
    Duration m_min_duration{};
    Duration m_max_duration{};
    long double m_mean_ticks{};
    long double m_ema_ticks{};
    long double m_ema_alpha{0.2L};
    bool m_has_samples{false};

    static Duration from_ticks(long double ticks) {
        return std::chrono::duration_cast<Duration>(FloatingDuration(ticks));
    }

    void update_stats(Duration duration) {
        const long double ticks = static_cast<long double>(duration.count());

        m_last_duration = duration;
        if (!m_has_samples) {
            m_has_samples = true;
            m_sample_count = 1;
            m_min_duration = duration;
            m_max_duration = duration;
            m_mean_ticks = ticks;
            m_ema_ticks = ticks;
            return;
        }

        ++m_sample_count;
        if (duration < m_min_duration) {
            m_min_duration = duration;
        }
        if (duration > m_max_duration) {
            m_max_duration = duration;
        }

        m_mean_ticks += (ticks - m_mean_ticks) / static_cast<long double>(m_sample_count);
        m_ema_ticks = m_ema_alpha * ticks + (1.0L - m_ema_alpha) * m_ema_ticks;
    }

public:
    explicit RunningProfiler(long double ema_alpha = 0.2L) : m_ema_alpha(ema_alpha) {
        if (!(ema_alpha > 0.0L && ema_alpha <= 1.0L)) {
            throw std::invalid_argument("RunningProfiler requires 0 < ema_alpha <= 1");
        }
    }

    void reset() {
        m_sample_count = 0;
        m_last_duration = Duration{};
        m_min_duration = Duration{};
        m_max_duration = Duration{};
        m_mean_ticks = 0.0L;
        m_ema_ticks = 0.0L;
        m_has_samples = false;
    }

    std::size_t sample_count() const { return m_sample_count; }

    long double ema_alpha() const { return m_ema_alpha; }

    void set_ema_alpha(long double ema_alpha) {
        if (!(ema_alpha > 0.0L && ema_alpha <= 1.0L)) {
            throw std::invalid_argument("RunningProfiler requires 0 < ema_alpha <= 1");
        }
        m_ema_alpha = ema_alpha;
    }

    template <typename Callable, typename... Args>
    auto measure(Callable&& callable, Args&&... args) {
        auto result = measure_once<Duration, Clock>(std::forward<Callable>(callable), std::forward<Args>(args)...);
        update_stats(result.duration);
        return result;
    }

    void add_sample(Duration duration) { update_stats(duration); }

    RunningProfilerStats<Duration> stats() const {
        if (!m_has_samples) {
            return RunningProfilerStats<Duration>{};
        }

        return RunningProfilerStats<Duration>{
            m_sample_count,
            m_last_duration,
            m_min_duration,
            m_max_duration,
            from_ticks(m_mean_ticks),
            from_ticks(m_ema_ticks),
        };
    }
};

}  // namespace pbpt::utils

#ifndef PBPT_PROFILE_RUNNING_AVG_NS_RETURN
#define PBPT_PROFILE_RUNNING_AVG_NS_RETURN(PROFILER_NAME, LABEL, EXPR)                                                \
    ([&]() {                                                                                                           \
        static ::pbpt::utils::RunningProfiler<std::chrono::nanoseconds> PROFILER_NAME;                                \
        auto result_p = PROFILER_NAME.measure([&]() { return (EXPR); });                                              \
        const auto stats = PROFILER_NAME.stats();                                                                      \
        std::cout << LABEL << " average took " << stats.mean_duration.count() << " nanoseconds"                    \
                  << " (samples=" << stats.sample_count << ")" << std::endl;                                       \
        return result_p.value;                                                                                         \
    }())
#endif
