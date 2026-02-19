#include <gtest/gtest.h>

#include "pbpt/pbpt.h"

namespace pbpt::sampler::testing {

using pbpt::math::Float;

TEST(SamplerUtilsTest, GeneratesStratifiedOffsets) {
    constexpr int N = 4;
    auto samples = generate_strified_array<Float, N>(0.1f);

    EXPECT_NEAR(samples[0], 0.1f, 1e-6f);
    EXPECT_NEAR(samples[1], 0.35f, 1e-6f);
    EXPECT_NEAR(samples[2], 0.6f, 1e-6f);
    EXPECT_NEAR(samples[3], 0.85f, 1e-6f);
}

TEST(SamplerUtilsTest, WrapsAroundUpperBound) {
    constexpr int N = 3;
    auto samples = generate_strified_array<Float, N>(0.9f);

    EXPECT_NEAR(samples[0], 0.9f, 1e-6f);
    EXPECT_NEAR(samples[1], 0.233333f, 1e-6f);
    EXPECT_NEAR(samples[2], 0.566667f, 1e-6f);
}

}  // namespace pbpt::sampler::testing
