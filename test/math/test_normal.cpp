/**
 * @file test_normal.cpp
 * @brief Comprehensive unit tests for the Normal template class
 * 
 * This file contains thorough tests for the Normal class template,
 * covering all methods, operators, and edge cases.
 */

#include <gtest/gtest.h>

#include <type_traits>

#include "pbpt.h"

namespace pbpt::math::testing {

// Test fixture for Normal tests
class NormalTest : public ::testing::Test {
protected:
    Normal3 n1{1.0, 0.0, 0.0};  // X-axis normal
    Normal3 n2{0.0, 1.0, 0.0};  // Y-axis normal
    Normal3 n3{0.0, 0.0, 1.0};  // Z-axis normal
    Normal3 n_normalized{0.6, 0.8, 0.0};  // Pre-normalized normal
    Vec3 v1{1.0, 2.0, 3.0};
    Vec3 v2{-1.0, 1.0, -1.0};
};

TEST_F(NormalTest, DefaultConstruction) {
    Normal3 n;
    EXPECT_FLOAT_EQ(n.x(), 0.0);
    EXPECT_FLOAT_EQ(n.y(), 0.0);
    EXPECT_FLOAT_EQ(n.z(), 0.0);
}

TEST_F(NormalTest, VariadicConstruction) {
    Normal3 n(1.0, 2.0, 3.0);
    EXPECT_FLOAT_EQ(n.x(), 1.0);
    EXPECT_FLOAT_EQ(n.y(), 2.0);
    EXPECT_FLOAT_EQ(n.z(), 3.0);
}

TEST_F(NormalTest, StaticFactoryMethods) {
    // Test zeros
    auto n_zeros = Normal3::zeros();
    EXPECT_FLOAT_EQ(n_zeros.x(), 0.0);
    EXPECT_FLOAT_EQ(n_zeros.y(), 0.0);
    EXPECT_FLOAT_EQ(n_zeros.z(), 0.0);
    
    // Test ones
    auto n_ones = Normal3::ones();
    EXPECT_FLOAT_EQ(n_ones.x(), 1.0);
    EXPECT_FLOAT_EQ(n_ones.y(), 1.0);
    EXPECT_FLOAT_EQ(n_ones.z(), 1.0);
    
    // Test filled
    auto n_filled = Normal3::filled(2.5);
    EXPECT_FLOAT_EQ(n_filled.x(), 2.5);
    EXPECT_FLOAT_EQ(n_filled.y(), 2.5);
    EXPECT_FLOAT_EQ(n_filled.z(), 2.5);
    
    // Test from_array
    std::array<Float, 3> arr = {1.5, 2.5, 3.5};
    auto n_from_arr = Normal3::from_array(arr);
    EXPECT_FLOAT_EQ(n_from_arr.x(), 1.5);
    EXPECT_FLOAT_EQ(n_from_arr.y(), 2.5);
    EXPECT_FLOAT_EQ(n_from_arr.z(), 3.5);
}

TEST_F(NormalTest, VectorConversion) {
    // Test from_vector
    Vec3 v(1.0, 2.0, 3.0);
    auto n_from_vec = Normal3::from_vector(v);
    EXPECT_FLOAT_EQ(n_from_vec.x(), 1.0);
    EXPECT_FLOAT_EQ(n_from_vec.y(), 2.0);
    EXPECT_FLOAT_EQ(n_from_vec.z(), 3.0);
    
    // Test to_vector
    Normal3 n(4.0, 5.0, 6.0);
    auto v_from_normal = n.to_vector();
    EXPECT_FLOAT_EQ(v_from_normal.x(), 4.0);
    EXPECT_FLOAT_EQ(v_from_normal.y(), 5.0);
    EXPECT_FLOAT_EQ(v_from_normal.z(), 6.0);
    
    // Verify types
    static_assert(std::is_same_v<decltype(n_from_vec), Normal3>);
    static_assert(std::is_same_v<decltype(v_from_normal), Vec3>);
}

TEST_F(NormalTest, DotProductWithVector) {
    Normal3 n(1.0, 2.0, 3.0);
    Vec3 v(4.0, 5.0, 6.0);

    auto normal_dot = n.dot(v);
    EXPECT_DOUBLE_EQ(normal_dot, 32.0);

    auto free_dot = dot(v, n);
    EXPECT_DOUBLE_EQ(free_dot, 32.0);

    const Normal3 n_const(1.0, -2.0, 0.5);
    Vec3 v2(2.0, 3.0, 4.0);
    auto const_dot = n_const.dot(v2);
    EXPECT_DOUBLE_EQ(const_dot, -2.0);

    Normal<float, 3> n_mixed(1.0f, 2.0f, 3.0f);
    Vector<double, 3> v_mixed(0.5, 1.5, -2.0);
    auto mixed_dot = n_mixed.dot(v_mixed);
    static_assert(std::is_same_v<decltype(mixed_dot), double>);
    EXPECT_DOUBLE_EQ(mixed_dot, -2.5);

    auto mixed_free = dot(v_mixed, n_mixed);
    EXPECT_DOUBLE_EQ(mixed_free, -2.5);
}

TEST_F(NormalTest, FaceForward) {
    Normal3 n(0.0, 0.0, 1.0);  // Normal pointing in +Z direction
    
    // Test with vector pointing in same direction (positive dot product)
    Vec3 v_same(0.0, 0.0, 2.0);  // Pointing in +Z
    auto n_same = n.face_forward(v_same);
    EXPECT_FLOAT_EQ(n_same.x(), 0.0);
    EXPECT_FLOAT_EQ(n_same.y(), 0.0);
    EXPECT_FLOAT_EQ(n_same.z(), 1.0);  // Should remain unchanged
    
    // Test with vector pointing in opposite direction (negative dot product)
    Vec3 v_opposite(0.0, 0.0, -2.0);  // Pointing in -Z
    auto n_opposite = n.face_forward(v_opposite);
    EXPECT_FLOAT_EQ(n_opposite.x(), 0.0);
    EXPECT_FLOAT_EQ(n_opposite.y(), 0.0);
    EXPECT_FLOAT_EQ(n_opposite.z(), -1.0);  // Should be flipped
    
    // Test with perpendicular vector (zero dot product)
    Vec3 v_perp(1.0, 0.0, 0.0);  // Perpendicular to normal
    auto n_perp = n.face_forward(v_perp);
    EXPECT_FLOAT_EQ(n_perp.x(), 0.0);
    EXPECT_FLOAT_EQ(n_perp.y(), 0.0);
    EXPECT_FLOAT_EQ(n_perp.z(), 1.0);  // Should remain unchanged (dot = 0, not < 0)
}

TEST_F(NormalTest, FaceForwardComplexCases) {
    Normal3 n(1.0, 1.0, 0.0);  // Diagonal normal in XY plane
    
    // Test with vector that has negative dot product
    Vec3 v_negative(-1.0, -1.0, 0.0);
    auto n_negative = n.face_forward(v_negative);
    EXPECT_FLOAT_EQ(n_negative.x(), -1.0);
    EXPECT_FLOAT_EQ(n_negative.y(), -1.0);
    EXPECT_FLOAT_EQ(n_negative.z(), 0.0);
    
    // Test with vector that has positive dot product
    Vec3 v_positive(2.0, 3.0, 0.0);
    auto n_positive = n.face_forward(v_positive);
    EXPECT_FLOAT_EQ(n_positive.x(), 1.0);
    EXPECT_FLOAT_EQ(n_positive.y(), 1.0);
    EXPECT_FLOAT_EQ(n_positive.z(), 0.0);
}

TEST_F(NormalTest, Accessors) {
    Normal3 n(1.0, 2.0, 3.0);
    
    // Test coordinate accessors
    EXPECT_FLOAT_EQ(n.x(), 1.0);
    EXPECT_FLOAT_EQ(n.y(), 2.0);
    EXPECT_FLOAT_EQ(n.z(), 3.0);
    
    // Test index accessors
    EXPECT_FLOAT_EQ(n[0], 1.0);
    EXPECT_FLOAT_EQ(n[1], 2.0);
    EXPECT_FLOAT_EQ(n[2], 3.0);
    
    // Test at() method
    EXPECT_FLOAT_EQ(n.at(0), 1.0);
    EXPECT_FLOAT_EQ(n.at(1), 2.0);
    EXPECT_FLOAT_EQ(n.at(2), 3.0);
    
    // Test modification
    n.x() = 10.0;
    n[1] = 20.0;
    EXPECT_FLOAT_EQ(n.x(), 10.0);
    EXPECT_FLOAT_EQ(n.y(), 20.0);
}

TEST_F(NormalTest, ArrayConversion) {
    Normal3 n(1.0, 2.0, 3.0);
    auto arr = n.to_array();
    
    EXPECT_FLOAT_EQ(arr[0], 1.0);
    EXPECT_FLOAT_EQ(arr[1], 2.0);
    EXPECT_FLOAT_EQ(arr[2], 3.0);
}

TEST_F(NormalTest, Dimensions) {
    Normal3 n;
    EXPECT_EQ(n.dims(), 3);
    
    Normal<Float, 2> n2d;
    EXPECT_EQ(n2d.dims(), 2);
    
    Normal<Float, 4> n4d;
    EXPECT_EQ(n4d.dims(), 4);
}

TEST_F(NormalTest, EqualityComparison) {
    Normal3 n1(1.0, 2.0, 3.0);
    Normal3 n2(1.0, 2.0, 3.0);
    Normal3 n3(1.0, 2.0, 3.1);
    
    EXPECT_TRUE(n1 == n2);
    EXPECT_FALSE(n1 == n3);
    EXPECT_FALSE(n1 != n2);
    EXPECT_TRUE(n1 != n3);
}

TEST_F(NormalTest, TypeConversion) {
    Normal3 n(1.0, 2.0, 3.0);
    
    // Test type casting
    auto n_double = n.type_cast<double>();
    static_assert(std::is_same_v<decltype(n_double), Normal<double, 3>>);
    EXPECT_DOUBLE_EQ(n_double.x(), 1.0);
    EXPECT_DOUBLE_EQ(n_double.y(), 2.0);
    EXPECT_DOUBLE_EQ(n_double.z(), 3.0);
    
    // Test dimension casting
    auto n_2d = n.dim_cast<2>();
    static_assert(std::is_same_v<decltype(n_2d), Normal<Float, 2>>);
    EXPECT_EQ(n_2d.dims(), 2);
    EXPECT_FLOAT_EQ(n_2d.x(), 1.0);
    EXPECT_FLOAT_EQ(n_2d.y(), 2.0);
    
    auto n_4d = n.dim_cast<4>();
    static_assert(std::is_same_v<decltype(n_4d), Normal<Float, 4>>);
    EXPECT_EQ(n_4d.dims(), 4);
    EXPECT_FLOAT_EQ(n_4d.x(), 1.0);
    EXPECT_FLOAT_EQ(n_4d.y(), 2.0);
    EXPECT_FLOAT_EQ(n_4d.z(), 3.0);
    EXPECT_FLOAT_EQ(n_4d.w(), 0.0);  // Default initialized
}

TEST_F(NormalTest, GeneralCasting) {
    Normal3 n(1.0, 2.0, 3.0);
    
    // Test combined type and dimension casting
    auto n_int_2d = n.cast<int, 2>();
    static_assert(std::is_same_v<decltype(n_int_2d), Normal<int, 2>>);
    EXPECT_EQ(n_int_2d.dims(), 2);
    EXPECT_EQ(n_int_2d.x(), 1);
    EXPECT_EQ(n_int_2d.y(), 2);
    
    auto n_double_4d = n.cast<double, 4>();
    static_assert(std::is_same_v<decltype(n_double_4d), Normal<double, 4>>);
    EXPECT_EQ(n_double_4d.dims(), 4);
    EXPECT_DOUBLE_EQ(n_double_4d.x(), 1.0);
    EXPECT_DOUBLE_EQ(n_double_4d.y(), 2.0);
    EXPECT_DOUBLE_EQ(n_double_4d.z(), 3.0);
    EXPECT_DOUBLE_EQ(n_double_4d.w(), 0.0);
}

// Test different dimensions
TEST(NormalDimensionsTest, VariousDimensions) {
    // 2D Normal
    Normal2 n2d(1.0, 2.0);
    EXPECT_FLOAT_EQ(n2d.x(), 1.0);
    EXPECT_FLOAT_EQ(n2d.y(), 2.0);
    EXPECT_EQ(n2d.dims(), 2);
    
    // 4D Normal
    Normal4 n4d(1.0, 2.0, 3.0, 4.0);
    EXPECT_FLOAT_EQ(n4d.x(), 1.0);
    EXPECT_FLOAT_EQ(n4d.y(), 2.0);
    EXPECT_FLOAT_EQ(n4d.z(), 3.0);
    EXPECT_FLOAT_EQ(n4d.w(), 4.0);
    EXPECT_EQ(n4d.dims(), 4);
}

// Test edge cases
TEST_F(NormalTest, EdgeCases) {
    // Test zero normal
    Normal3 zero_normal = Normal3::zeros();
    Vec3 test_vector(1.0, 0.0, 0.0);
    auto faced = zero_normal.face_forward(test_vector);
    // Zero dot product should not flip the normal
    EXPECT_TRUE(faced == zero_normal);
    
    // Test very small values
    Normal3 tiny_normal(1e-10, 1e-10, 1e-10);
    auto tiny_vector = tiny_normal.to_vector();
    auto back_to_normal = Normal3::from_vector(tiny_vector);
    EXPECT_FLOAT_EQ(back_to_normal.x(), tiny_normal.x());
    EXPECT_FLOAT_EQ(back_to_normal.y(), tiny_normal.y());
    EXPECT_FLOAT_EQ(back_to_normal.z(), tiny_normal.z());
}

// Test floating point requirements
TEST(NormalConstraintsTest, FloatingPointOnly) {
    // Verify that Normal only accepts floating point types
    static_assert(std::is_floating_point_v<Float>);
    
    // These should compile
    Normal<float, 3> n_float;
    Normal<double, 3> n_double;
    
    // These should NOT compile (but we can't test compilation failures easily)
    // Normal<int, 3> n_int;     // Should fail
    // Normal<long, 3> n_long;   // Should fail
}


}  // namespace pbpt::math::testing
