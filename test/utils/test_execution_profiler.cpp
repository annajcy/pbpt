#include <gtest/gtest.h>

#include <chrono>
#include <thread>
#include <type_traits>

#include "pbpt/utils/execution_profiler.hpp"

using namespace pbpt::utils;

TEST(ExecutionProfilerTest, MeasureOnceReturnsDurationAndValue) {
    auto result = measure_once([] { return 7; });

    EXPECT_EQ(result.value, 7);
    EXPECT_GE(result.duration.count(), 0);
    EXPECT_TRUE((std::is_same_v<decltype(result.duration), std::chrono::nanoseconds >));
}

TEST(ExecutionProfilerTest, MeasureOnceSupportsVoidCallable) {
    int counter = 0;

    auto result = measure_once([&counter] { ++counter; });

    EXPECT_EQ(counter, 1);
    EXPECT_GE(result.duration.count(), 0);
}

TEST(ExecutionProfilerTest, MeasureNReturnsTotalAverageAndLastValue) {
    int counter = 0;

    auto result = measure_n(5, [&counter] {
        ++counter;
        return counter;
    });

    EXPECT_EQ(counter, 5);
    EXPECT_EQ(result.iterations, 5);
    EXPECT_EQ(result.value, 5);
    EXPECT_GE(result.total_duration.count(), result.average_duration.count());
}

TEST(ExecutionProfilerTest, MeasureNSupportsVoidCallable) {
    int counter = 0;

    auto result = measure_n(4, [&counter] { ++counter; });

    EXPECT_EQ(counter, 4);
    EXPECT_EQ(result.iterations, 4);
    EXPECT_GE(result.total_duration.count(), result.average_duration.count());
}

TEST(ExecutionProfilerTest, MeasureNThrowsOnZeroIterations) {
    EXPECT_THROW(
        {
            measure_n(0, [] {});
        },
        std::invalid_argument);
}

TEST(ExecutionProfilerTest, MeasureOnceAllowsDurationOverride) {
    auto result = measure_once<std::chrono::nanoseconds>([] {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        return 1;
    });

    EXPECT_EQ(result.value, 1);
    EXPECT_GE(result.duration.count(), 1'000'000);
}

TEST(ExecutionProfilerTest, RunningProfilerTracksBasicStats) {
    RunningProfiler<std::chrono::microseconds> profiler;

    auto first = profiler.measure([] {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        return 11;
    });
    auto second = profiler.measure([] {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        return 22;
    });

    EXPECT_EQ(first.value, 11);
    EXPECT_EQ(second.value, 22);

    const auto stats = profiler.stats();
    EXPECT_EQ(stats.sample_count, 2);
    EXPECT_GE(stats.last_duration.count(), 1500);
    EXPECT_GE(stats.max_duration.count(), stats.min_duration.count());
    EXPECT_GE(stats.mean_duration.count(), stats.min_duration.count());
    EXPECT_LE(stats.mean_duration.count(), stats.max_duration.count());
}

TEST(ExecutionProfilerTest, RunningProfilerEmaAlphaAffectsEma) {
    RunningProfiler<std::chrono::nanoseconds> profiler(1.0L);

    profiler.add_sample(std::chrono::nanoseconds(100));
    profiler.add_sample(std::chrono::nanoseconds(300));

    const auto stats = profiler.stats();
    EXPECT_EQ(stats.sample_count, 2);
    EXPECT_EQ(stats.last_duration.count(), 300);
    EXPECT_EQ(stats.ema_duration.count(), 300);
}

TEST(ExecutionProfilerTest, RunningProfilerResetClearsStats) {
    RunningProfiler<std::chrono::nanoseconds> profiler;
    profiler.add_sample(std::chrono::nanoseconds(50));
    profiler.reset();

    const auto stats = profiler.stats();
    EXPECT_EQ(stats.sample_count, 0);
    EXPECT_EQ(stats.last_duration.count(), 0);
    EXPECT_EQ(stats.mean_duration.count(), 0);
    EXPECT_EQ(stats.ema_duration.count(), 0);
}

TEST(ExecutionProfilerTest, RunningProfilerRejectsInvalidEmaAlpha) {
    EXPECT_THROW((RunningProfiler<std::chrono::nanoseconds>(0.0L)), std::invalid_argument);

    RunningProfiler<std::chrono::nanoseconds> profiler;
    EXPECT_THROW(profiler.set_ema_alpha(1.5L), std::invalid_argument);
}
