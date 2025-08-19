#include <gtest/gtest.h>
#include "math/homogeneous.hpp"

using namespace pbpt;
using namespace pbpt::math;

// Test new Homogeneous functionality added in the refactoring

namespace pbpt::math::testing {

TEST(HomogeneousNewFeaturesTest, FactoryMethods) {
    // Test zeros factory
    auto h_zero = Homo3::zeros();
    EXPECT_TRUE(h_zero.is_all_zero());
    EXPECT_TRUE(h_zero.is_vector());
    
    // Test filled factory
    auto h_filled = Homo3::filled(2.5);
    for (int i = 0; i < 4; ++i) {
        EXPECT_DOUBLE_EQ(h_filled.to_vector_raw()[i], 2.5);
    }
    
    // Test from_point factory
    Point<Float, 3> p(1.0, 2.0, 3.0);
    auto h_point = Homo3::from_point(p);
    EXPECT_TRUE(h_point.is_point());
    EXPECT_DOUBLE_EQ(h_point.to_vector_raw()[3], 1.0);  // w should be 1
    
    // Test from_vector factory  
    Vector<Float, 3> v(1.0, 2.0, 3.0);
    auto h_vector = Homo3::from_vector(v);
    EXPECT_TRUE(h_vector.is_vector());
    EXPECT_DOUBLE_EQ(h_vector.to_vector_raw()[3], 0.0);  // w should be 0
}

TEST(HomogeneousNewFeaturesTest, EnhancedOperators) {
    Homo3 h1(1.0, 2.0, 3.0, 1.0);
    Homo3 h2(2.0, 3.0, 4.0, 1.0);
    
    // Test arithmetic operators
    auto h_sum = h1 + h2;
    EXPECT_DOUBLE_EQ(h_sum.to_vector_raw()[0], 3.0);
    EXPECT_DOUBLE_EQ(h_sum.to_vector_raw()[1], 5.0);
    EXPECT_DOUBLE_EQ(h_sum.to_vector_raw()[2], 7.0);
    EXPECT_DOUBLE_EQ(h_sum.to_vector_raw()[3], 2.0);
    
    // Test scalar multiplication
    auto h_scaled = h1 * 2.0;
    EXPECT_DOUBLE_EQ(h_scaled.to_vector_raw()[0], 2.0);
    EXPECT_DOUBLE_EQ(h_scaled.to_vector_raw()[1], 4.0);
    EXPECT_DOUBLE_EQ(h_scaled.to_vector_raw()[2], 6.0);
    EXPECT_DOUBLE_EQ(h_scaled.to_vector_raw()[3], 2.0);
    
    // Test comparison
    Homo3 h1_copy(1.0, 2.0, 3.0, 1.0);
    EXPECT_TRUE(h1 == h1_copy);
    EXPECT_FALSE(h1 == h2);
}

TEST(HomogeneousNewFeaturesTest, ApplyFunction) {
    Homo3 h(1.0, 2.0, 3.0, 1.0);
    
    // Apply function to modify elements (signature: f(T&, int))
    h.visit([](Float& x, int i) { x = x * 2.0; });
    
    EXPECT_DOUBLE_EQ(h.to_vector_raw()[0], 2.0);
    EXPECT_DOUBLE_EQ(h.to_vector_raw()[1], 4.0);
    EXPECT_DOUBLE_EQ(h.to_vector_raw()[2], 6.0);
    EXPECT_DOUBLE_EQ(h.to_vector_raw()[3], 2.0);
}

TEST(HomogeneousNewFeaturesTest, NormalizationAndChecks) {
    Homo3 h(2.0, 4.0, 6.0, 2.0);  // Point with w = 2
    
    // Test normalization
    auto h_norm = h.standardized();
    EXPECT_DOUBLE_EQ(h_norm.to_vector_raw()[0], 1.0);
    EXPECT_DOUBLE_EQ(h_norm.to_vector_raw()[1], 2.0);
    EXPECT_DOUBLE_EQ(h_norm.to_vector_raw()[2], 3.0);
    EXPECT_DOUBLE_EQ(h_norm.to_vector_raw()[3], 1.0);
    
    // Original should remain unchanged
    EXPECT_DOUBLE_EQ(h.to_vector_raw()[0], 2.0);
    EXPECT_DOUBLE_EQ(h.to_vector_raw()[3], 2.0);
    
    // Test normalization check
    EXPECT_TRUE(h_norm.is_standardized());
    EXPECT_FALSE(h.is_standardized());
}

TEST(HomogeneousNewFeaturesTest, ConstexprCapabilities) {
    // Test constexpr construction and type checking
    constexpr Homo3 h_point(1.0, 2.0, 3.0, 1.0);
    constexpr Homo3 h_vector(1.0, 2.0, 3.0, 0.0);
    
    // These should work at compile time
    static_assert(h_point.is_point(), "Point should be detected at compile time");
    static_assert(h_vector.is_vector(), "Vector should be detected at compile time");
    static_assert(!h_point.is_vector(), "Point should not be vector");
    static_assert(!h_vector.is_point(), "Vector should not be point");
}

TEST(HomogeneousNewFeaturesTest, ErrorHandling) {
    Homo3 h_point(1.0, 2.0, 3.0, 1.0);  // Point
    Homo3 h_vector(1.0, 2.0, 3.0, 0.0);  // Vector
    
    // Test invalid conversions throw proper exceptions
    EXPECT_THROW(h_point.to_vector(), std::domain_error);
    EXPECT_THROW(h_vector.to_point(), std::domain_error);
}

}