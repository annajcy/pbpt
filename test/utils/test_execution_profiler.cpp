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
    EXPECT_TRUE((std::is_same_v<decltype(result.duration), std::chrono::microseconds>));
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
