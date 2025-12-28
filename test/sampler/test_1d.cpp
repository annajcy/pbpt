#include <gtest/gtest.h>

#include <cmath>

#include "pbpt.h"

namespace pbpt::sampler::testing {

using pbpt::math::Float;

TEST(SamplingTest, UniformInterval) {
    EXPECT_FLOAT_EQ(sample_uniform(0.0f, -1.0f, 3.0f), -1.0f);
    EXPECT_FLOAT_EQ(sample_uniform(1.0f, -1.0f, 3.0f), 3.0f);
    EXPECT_FLOAT_EQ(sample_uniform(0.5f, 2.0f, 4.0f), 3.0f);

    EXPECT_FLOAT_EQ(sample_uniform_pdf(0.3f), 1.0f);
    EXPECT_FLOAT_EQ(sample_uniform_pdf(0.3f, -1.0f, 3.0f), 0.25f);
    EXPECT_FLOAT_EQ(sample_uniform_pdf(-0.1f), 0.0f);
    EXPECT_FLOAT_EQ(sample_uniform_pdf(1.1f), 0.0f);
}

TEST(SamplingTest, TentDistribution) {
    Float r = 2.0f;
    Float left_expected = -r + r * std::sqrt(2.0f * 0.25f);
    Float right_expected = r - r * std::sqrt(2.0f * (1.0f - 0.75f));

    EXPECT_NEAR(sample_tent(0.0f, r), -r, 1e-6f);
    EXPECT_NEAR(sample_tent(0.25f, r), left_expected, 1e-6f);
    EXPECT_NEAR(sample_tent(0.75f, r), right_expected, 1e-6f);
    EXPECT_NEAR(sample_tent_pdf(0.0f, r), 0.5f, 1e-6f);
    EXPECT_FLOAT_EQ(sample_tent_pdf(3.0f, r), 0.0f);
}

}  // namespace pbpt::sampler::testing
