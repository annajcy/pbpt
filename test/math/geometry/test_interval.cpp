/**
 * @file test_interval.cpp
 * @brief Comprehensive unit tests for the Interval template class
 * 
 * This file contains thorough tests for the Interval class template,
 * covering all methods, edge cases, and mathematical properties.
 */

#include <gtest/gtest.h>
#include <type_traits>

#include "math/geometry/interval.hpp"
#include "math/global/type_alias.hpp"

namespace pbpt::math::testing {

// Test fixture for Interval tests
class IntervalTest : public ::testing::Test {
protected:
    // Common test values
    Float zero = 0.0f;
    Float one = 1.0f;
    Float two = 2.0f;
    Float three = 3.0f;
    Float negative_one = -1.0f;
    Float negative_two = -2.0f;
    Float small_epsilon = epsilon_v<Float>;
};

// Constructor Tests
TEST_F(IntervalTest, DefaultConstructor) {
    Interval<Float> interval;
    EXPECT_TRUE(interval.is_empty());
}

TEST_F(IntervalTest, SingleValueConstructor) {
    Interval<Float> interval(one);
    
    EXPECT_EQ(interval.m_low, one);
    EXPECT_EQ(interval.m_high, one);
    EXPECT_FALSE(interval.is_empty());
    EXPECT_TRUE(interval.contains(one));
    EXPECT_NEAR(interval.width(), zero, small_epsilon);
}

TEST_F(IntervalTest, ExplicitRangeConstructor) {
    Interval<Float> interval(one, three);
    
    EXPECT_EQ(interval.m_low, one);
    EXPECT_EQ(interval.m_high, three);
    EXPECT_FALSE(interval.is_empty());
}


TEST_F(IntervalTest, FromValueWithError) {
    Float value = 5.0f;
    Float error = 1.0f;
    
    auto interval = Interval<Float>::from_value_with_error(value, error);
    
    EXPECT_EQ(interval.m_low, 4.0f);
    EXPECT_EQ(interval.m_high, 6.0f);
    EXPECT_TRUE(interval.contains(value));
    EXPECT_NEAR(interval.midpoint(), value, small_epsilon);
}

// Basic Property Tests
TEST_F(IntervalTest, Midpoint) {
    Interval<Float> interval(one, three);
    
    EXPECT_NEAR(interval.midpoint(), two, small_epsilon);
    
    // Test with negative values
    Interval<Float> neg_interval(negative_two, negative_one);
    EXPECT_NEAR(neg_interval.midpoint(), -1.5f, small_epsilon);
    
    // Test with single value
    Interval<Float> point(one);
    EXPECT_NEAR(point.midpoint(), one, small_epsilon);
}

TEST_F(IntervalTest, Width) {
    Interval<Float> interval(one, three);
    
    EXPECT_NEAR(interval.width(), two, small_epsilon);
    
    // Test with single value
    Interval<Float> point(one);
    EXPECT_NEAR(point.width(), zero, small_epsilon);
    
    // Test with negative range
    Interval<Float> neg_interval(negative_two, one);
    EXPECT_NEAR(neg_interval.width(), three, small_epsilon);
}

TEST_F(IntervalTest, Contains) {
    Interval<Float> interval(one, three);
    
    // Should contain values within range
    EXPECT_TRUE(interval.contains(one));
    EXPECT_TRUE(interval.contains(two));
    EXPECT_TRUE(interval.contains(three));
    EXPECT_TRUE(interval.contains(1.5f));
    EXPECT_TRUE(interval.contains(2.5f));
    
    // Should not contain values outside range
    EXPECT_FALSE(interval.contains(zero));
    EXPECT_FALSE(interval.contains(4.0f));
    EXPECT_FALSE(interval.contains(negative_one));
}

TEST_F(IntervalTest, IsEmpty) {
    // Default constructor creates empty interval
    Interval<Float> empty;
    EXPECT_TRUE(empty.is_empty());
    
    // Single value interval is not empty
    Interval<Float> point(one);
    EXPECT_FALSE(point.is_empty());
    
    // Range interval is not empty
    Interval<Float> range(one, three);
    EXPECT_FALSE(range.is_empty());
    
    // Equal low and high should not be empty (it's a point)
    Interval<Float> equal(two, two);
    EXPECT_FALSE(equal.is_empty());
}

// Intersection Tests
TEST_F(IntervalTest, IsIntersectingOverlapping) {
    Interval<Float> interval1(one, three);
    Interval<Float> interval2(two, 4.0f);
    
    EXPECT_TRUE(interval1.is_intersected(interval2));
    EXPECT_TRUE(interval2.is_intersected(interval1));
}

TEST_F(IntervalTest, IsIntersectingTouching) {
    Interval<Float> interval1(one, two);
    Interval<Float> interval2(two, three);
    
    EXPECT_TRUE(interval1.is_intersected(interval2));
    EXPECT_TRUE(interval2.is_intersected(interval1));
}

TEST_F(IntervalTest, IsIntersectingNonOverlapping) {
    Interval<Float> interval1(one, two);
    Interval<Float> interval2(three, 4.0f);
    
    EXPECT_FALSE(interval1.is_intersected(interval2));
    EXPECT_FALSE(interval2.is_intersected(interval1));
}

TEST_F(IntervalTest, IsIntersectingContained) {
    Interval<Float> outer(one, 4.0f);
    Interval<Float> inner(two, three);
    
    EXPECT_TRUE(outer.is_intersected(inner));
    EXPECT_TRUE(inner.is_intersected(outer));
}

TEST_F(IntervalTest, IsIntersectingIdentical) {
    Interval<Float> interval1(one, three);
    Interval<Float> interval2(one, three);
    
    EXPECT_TRUE(interval1.is_intersected(interval2));
    EXPECT_TRUE(interval2.is_intersected(interval1));
}

TEST_F(IntervalTest, IsIntersectingWithEmpty) {
    Interval<Float> normal(one, three);
    Interval<Float> empty;
    
    EXPECT_FALSE(normal.is_intersected(empty));
    EXPECT_FALSE(empty.is_intersected(normal));
}

// Union Tests
TEST_F(IntervalTest, UnitedOverlapping) {
    Interval<Float> interval1(one, three);
    Interval<Float> interval2(two, 4.0f);
    
    auto united = interval1.united(interval2);
    
    EXPECT_EQ(united.m_low, one);
    EXPECT_EQ(united.m_high, 4.0f);
    EXPECT_TRUE(united.contains(one));
    EXPECT_TRUE(united.contains(4.0f));
    EXPECT_TRUE(united.contains(two));
}

TEST_F(IntervalTest, UnitedNonOverlapping) {
    Interval<Float> interval1(one, two);
    Interval<Float> interval2(three, 4.0f);
    
    auto united = interval1.united(interval2);
    
    EXPECT_EQ(united.m_low, one);
    EXPECT_EQ(united.m_high, 4.0f);
    EXPECT_TRUE(united.contains(one));
    EXPECT_TRUE(united.contains(4.0f));
    EXPECT_TRUE(united.contains(2.5f)); // Should contain gap
}

TEST_F(IntervalTest, UnitedContained) {
    Interval<Float> outer(one, 4.0f);
    Interval<Float> inner(two, three);
    
    auto united = outer.united(inner);
    
    EXPECT_EQ(united.m_low, outer.m_low);
    EXPECT_EQ(united.m_high, outer.m_high);
}

TEST_F(IntervalTest, UnitedIdentical) {
    Interval<Float> interval1(one, three);
    Interval<Float> interval2(one, three);
    
    auto united = interval1.united(interval2);
    
    EXPECT_EQ(united.m_low, one);
    EXPECT_EQ(united.m_high, three);
}

// Intersection Tests
TEST_F(IntervalTest, IntersectedOverlapping) {
    Interval<Float> interval1(one, three);
    Interval<Float> interval2(two, 4.0f);
    
    auto intersected = interval1.intersection(interval2);
    
    EXPECT_EQ(intersected.m_low, two);
    EXPECT_EQ(intersected.m_high, three);
    EXPECT_TRUE(intersected.contains(2.5f));
    EXPECT_FALSE(intersected.contains(one));
    EXPECT_FALSE(intersected.contains(4.0f));
}

TEST_F(IntervalTest, IntersectedTouching) {
    Interval<Float> interval1(one, two);
    Interval<Float> interval2(two, three);
    
    auto intersected = interval1.intersection(interval2);
    
    EXPECT_EQ(intersected.m_low, two);
    EXPECT_EQ(intersected.m_high, two);
    EXPECT_TRUE(intersected.contains(two));
    EXPECT_FALSE(intersected.contains(one));
    EXPECT_FALSE(intersected.contains(three));
}

TEST_F(IntervalTest, IntersectedNonOverlapping) {
    Interval<Float> interval1(one, two);
    Interval<Float> interval2(three, 4.0f);
    
    auto intersected = interval1.intersection(interval2);
    
    EXPECT_TRUE(intersected.is_empty());
}

TEST_F(IntervalTest, IntersectedContained) {
    Interval<Float> outer(one, 4.0f);
    Interval<Float> inner(two, three);
    
    auto intersected = outer.intersection(inner);
    
    EXPECT_EQ(intersected.m_low, inner.m_low);
    EXPECT_EQ(intersected.m_high, inner.m_high);
}

TEST_F(IntervalTest, IntersectedIdentical) {
    Interval<Float> interval1(one, three);
    Interval<Float> interval2(one, three);
    
    auto intersected = interval1.intersection(interval2);
    
    EXPECT_EQ(intersected.m_low, one);
    EXPECT_EQ(intersected.m_high, three);
}

// Edge Case Tests
TEST_F(IntervalTest, NegativeValues) {
    Interval<Float> interval(negative_two, negative_one);
    
    EXPECT_EQ(interval.m_low, negative_two);
    EXPECT_EQ(interval.m_high, negative_one);
    EXPECT_TRUE(interval.contains(-1.5f));
    EXPECT_FALSE(interval.contains(zero));
    EXPECT_NEAR(interval.width(), one, small_epsilon);
}

TEST_F(IntervalTest, ZeroSpanInterval) {
    Interval<Float> interval(two, two);
    
    EXPECT_EQ(interval.m_low, two);
    EXPECT_EQ(interval.m_high, two);
    EXPECT_TRUE(interval.contains(two));
    EXPECT_FALSE(interval.contains(one));
    EXPECT_FALSE(interval.contains(three));
    EXPECT_NEAR(interval.width(), zero, small_epsilon);
    EXPECT_FALSE(interval.is_empty());
}

TEST_F(IntervalTest, VerySmallInterval) {
    Float small_value = small_epsilon;
    Interval<Float> interval = Interval<Float>::from_value_with_error(zero, small_value);
    
    EXPECT_NEAR(interval.m_low, -small_value, small_epsilon);
    EXPECT_NEAR(interval.m_high, small_value, small_epsilon);
    EXPECT_TRUE(interval.contains(zero));
    EXPECT_NEAR(interval.width(), 2 * small_value, small_epsilon);
}

TEST_F(IntervalTest, LargeValues) {
    Float large = 1e6f;
    Interval<Float> interval(large, large * 2);
    
    EXPECT_EQ(interval.m_low, large);
    EXPECT_EQ(interval.m_high, large * 2);
    EXPECT_TRUE(interval.contains(large * 1.5f));
    EXPECT_FALSE(interval.contains(large * 0.5f));
    EXPECT_NEAR(interval.width(), large, large * small_epsilon);
}

// Mixed Type Tests
TEST_F(IntervalTest, MixedTypeUnion) {
    Interval<float> float_interval(1.0f, 3.0f);
    Interval<double> double_interval(2.0, 4.0);
    
    auto united = float_interval.united(double_interval);
    
    // Result should be double precision
    EXPECT_EQ(united.m_low, 1.0);
    EXPECT_EQ(united.m_high, 4.0);
    EXPECT_TRUE((std::is_same_v<decltype(united), Interval<double>>));
}

TEST_F(IntervalTest, MixedTypeIntersection) {
    Interval<float> float_interval(1.0f, 3.0f);
    Interval<double> double_interval(2.0, 4.0);
    
    auto intersected = float_interval.intersection(double_interval);
    
    // Result should be double precision
    EXPECT_EQ(intersected.m_low, 2.0);
    EXPECT_EQ(intersected.m_high, 3.0);
    EXPECT_TRUE((std::is_same_v<decltype(intersected), Interval<double>>));
}

// Double Precision Tests
TEST_F(IntervalTest, DoublePrecision) {
    Interval<double> interval(1.0, 3.0);
    
    EXPECT_EQ(interval.m_low, 1.0);
    EXPECT_EQ(interval.m_high, 3.0);
    EXPECT_TRUE(interval.contains(2.0));
    EXPECT_NEAR(interval.midpoint(), 2.0, epsilon_v<double>);
    EXPECT_NEAR(interval.width(), 2.0, epsilon_v<double>);
}

// Type Alias Tests
TEST_F(IntervalTest, TypeAliases) {
    // Test that we can create intervals with standard types
    Interval<float> float_interval(1.0f, 2.0f);
    Interval<double> double_interval(1.0, 2.0);
    
    EXPECT_FALSE(float_interval.is_empty());
    EXPECT_FALSE(double_interval.is_empty());
    
    EXPECT_TRUE(float_interval.contains(1.5f));
    EXPECT_TRUE(double_interval.contains(1.5));
    
    // Test project-specific type aliases
    Interv project_float_interval(1.0f, 2.0f);
    Interval<double> project_double_interval(1.0, 2.0);
    
    EXPECT_FALSE(project_float_interval.is_empty());
    EXPECT_FALSE(project_double_interval.is_empty());
    
    EXPECT_TRUE(project_float_interval.contains(1.5f));
    EXPECT_TRUE(project_double_interval.contains(1.5));
}

// Boundary Condition Tests
TEST_F(IntervalTest, BoundaryConditions) {
    Interval<Float> interval(one, three);
    
    // Test exact boundary values
    EXPECT_TRUE(interval.contains(one));
    EXPECT_TRUE(interval.contains(three));
    
    // Test values just outside boundaries
    Float just_below = one - small_epsilon * 10;
    Float just_above = three + small_epsilon * 10;
    
    EXPECT_FALSE(interval.contains(just_below));
    EXPECT_FALSE(interval.contains(just_above));
}

TEST_F(IntervalTest, FloatingPointPrecision) {
    // Test with values that might have floating point precision issues
    Float a = 0.1f;
    Float b = 0.2f;
    Float sum = a + b; // This might not exactly equal 0.3f due to floating point precision
    
    Interval<Float> interval = Interval<Float>::from_value_with_error(0.3f, small_epsilon * 10);
    
    EXPECT_TRUE(interval.contains(sum));
}

// Comprehensive Workflow Tests
TEST_F(IntervalTest, ComplexWorkflow) {
    // Create multiple intervals
    Interval<Float> interval1(1.0f, 3.0f);
    Interval<Float> interval2(2.0f, 4.0f);
    Interval<Float> interval3(5.0f, 6.0f);
    
    // Test union of overlapping intervals
    auto union12 = interval1.united(interval2);
    EXPECT_EQ(union12.m_low, 1.0f);
    EXPECT_EQ(union12.m_high, 4.0f);
    
    // Test union with non-overlapping interval
    auto union_all = union12.united(interval3);
    EXPECT_EQ(union_all.m_low, 1.0f);
    EXPECT_EQ(union_all.m_high, 6.0f);
    
    // Test intersection
    auto intersection12 = interval1.intersection(interval2);
    EXPECT_EQ(intersection12.m_low, 2.0f);
    EXPECT_EQ(intersection12.m_high, 3.0f);
    
    // Test intersection with non-overlapping
    auto intersection13 = interval1.intersection(interval3);
    EXPECT_TRUE(intersection13.is_empty());
}

}  // namespace pbpt::math::testing
