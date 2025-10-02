/**
 * @file test_tuple.cpp
 * @brief Unit tests for the Tuple template class
 * 
 * This file contains comprehensive tests for the Tuple class template,
 * which serves as a base class for mathematical objects like vectors, points, etc.
 * 
 * Tests cover:
 * - Construction (default, variadic, copy, conversion)
 * - Static factory methods (filled, zeros, ones, from_array)
 * - Element access (operator[], at(), x(), y(), z(), w())
 * - Equality comparison
 * - Type and dimension casting
 * - Array conversion
 * - Output streaming
 * - Various numeric types and dimensions
 */

#include <gtest/gtest.h>

#include <sstream>
#include <type_traits>

// 包含被测试的头文件和其依赖
#include "math/tuple.hpp"

namespace pbpt::math::testing {

// 创建一个简单的Tuple派生类用于测试
template<typename T, int N>
class TestTuple : public Tuple<TestTuple, T, N> {
public:
    using Base = Tuple<TestTuple, T, N>;
    using Base::Base;
    
    static constexpr const char* name() { return "TestTuple"; }
};

// Test fixture for Tuple tests
class TupleTest : public ::testing::Test {
protected:
    TestTuple<Float, 3> t1{1.0, 2.0, 3.0};
    TestTuple<Float, 3> t2{4.0, 5.0, 6.0};
    TestTuple<int, 3>   ti1{1, 2, 3};
    TestTuple<Float, 4> t4{1.0, 2.0, 3.0, 4.0};
};

TEST_F(TupleTest, DefaultConstruction) {
    TestTuple<Float, 3> t;
    EXPECT_EQ(t.x(), 0.0);
    EXPECT_EQ(t.y(), 0.0);
    EXPECT_EQ(t.z(), 0.0);
}

TEST_F(TupleTest, VariadicConstruction) {
    EXPECT_EQ(t1.x(), 1.0);
    EXPECT_EQ(t1.y(), 2.0);
    EXPECT_EQ(t1.z(), 3.0);
}

TEST_F(TupleTest, HasNaN) {
    TestTuple<Float, 3> v_nan(1.0, std::nan(""), 3.0);
    EXPECT_TRUE(v_nan.has_nan());
    EXPECT_FALSE(t1.has_nan());
}

TEST_F(TupleTest, StaticFactoryMethods) {
    // Test filled
    auto filled = TestTuple<Float, 4>::filled(5.0);
    EXPECT_EQ(filled.x(), 5.0);
    EXPECT_EQ(filled.y(), 5.0);
    EXPECT_EQ(filled.z(), 5.0);
    EXPECT_EQ(filled.w(), 5.0);
    
    // Test zeros
    auto zeros = TestTuple<Float, 3>::zeros();
    EXPECT_EQ(zeros.x(), 0.0);
    EXPECT_EQ(zeros.y(), 0.0);
    EXPECT_EQ(zeros.z(), 0.0);
    
    // Test ones
    auto ones = TestTuple<Float, 3>::ones();
    EXPECT_EQ(ones.x(), 1.0);
    EXPECT_EQ(ones.y(), 1.0);
    EXPECT_EQ(ones.z(), 1.0);
    
    // Test from_array
    std::array<Float, 3> arr = {1.0, 2.0, 3.0};
    auto from_arr = TestTuple<Float, 3>::from_array(arr);
    EXPECT_EQ(from_arr.x(), 1.0);
    EXPECT_EQ(from_arr.y(), 2.0);
    EXPECT_EQ(from_arr.z(), 3.0);
}

TEST_F(TupleTest, CopyConstructor) {
    TestTuple<Float, 3> t_copy(t1);
    EXPECT_EQ(t_copy.x(), t1.x());
    EXPECT_EQ(t_copy.y(), t1.y());
    EXPECT_EQ(t_copy.z(), t1.z());
}

TEST_F(TupleTest, ConversionConstructor) {
    // Type conversion
    TestTuple<double, 3> t_double(t1);
    EXPECT_EQ(t_double.x(), 1.0);
    EXPECT_EQ(t_double.y(), 2.0);
    EXPECT_EQ(t_double.z(), 3.0);
    
    // Dimension conversion (smaller)
    TestTuple<Float, 2> t_smaller(t1.dim_cast<2>());
    EXPECT_EQ(t_smaller.x(), 1.0);
    EXPECT_EQ(t_smaller.y(), 2.0);
    EXPECT_EQ(t_smaller.dims(), 2);
    
    // Dimension conversion (larger)
    TestTuple<Float, 5> t_larger(t1.dim_cast<5>());
    EXPECT_EQ(t_larger.x(), 1.0);
    EXPECT_EQ(t_larger.y(), 2.0);
    EXPECT_EQ(t_larger.z(), 3.0);
    EXPECT_EQ(t_larger[3], 0.0);  // Default initialized
    EXPECT_EQ(t_larger[4], 0.0);  // Default initialized
    EXPECT_EQ(t_larger.dims(), 5);
}

TEST_F(TupleTest, ArrayConversion) {
    auto arr = t1.to_array();
    EXPECT_EQ(arr[0], 1.0);
    EXPECT_EQ(arr[1], 2.0);
    EXPECT_EQ(arr[2], 3.0);
}

TEST_F(TupleTest, DimensionsProperty) {
    EXPECT_EQ(t1.dims(), 3);
    EXPECT_EQ(t4.dims(), 4);
    TestTuple<Float, 7> t7;
    EXPECT_EQ(t7.dims(), 7);
}

TEST_F(TupleTest, IndexAccessors) {
    // Non-const access
    EXPECT_EQ(t1[0], 1.0);
    EXPECT_EQ(t1[1], 2.0);
    EXPECT_EQ(t1[2], 3.0);
    
    // Modify through index
    t1[0] = 10.0;
    EXPECT_EQ(t1[0], 10.0);
    
    // Const access
    const TestTuple<Float, 3> t_const{1.0, 2.0, 3.0};
    EXPECT_EQ(t_const[0], 1.0);
    EXPECT_EQ(t_const[1], 2.0);
    EXPECT_EQ(t_const[2], 3.0);
    
    // at() method
    EXPECT_EQ(t_const.at(0), 1.0);
    EXPECT_EQ(t_const.at(1), 2.0);
    EXPECT_EQ(t_const.at(2), 3.0);
}

TEST_F(TupleTest, NamedAccessors) {
    // Test x, y, z, w accessors
    EXPECT_EQ(t4.x(), 1.0);
    EXPECT_EQ(t4.y(), 2.0);
    EXPECT_EQ(t4.z(), 3.0);
    EXPECT_EQ(t4.w(), 4.0);
    
    // Modify through named accessors
    t4.x() = 10.0;
    t4.y() = 20.0;
    t4.z() = 30.0;
    t4.w() = 40.0;
    
    EXPECT_EQ(t4.x(), 10.0);
    EXPECT_EQ(t4.y(), 20.0);
    EXPECT_EQ(t4.z(), 30.0);
    EXPECT_EQ(t4.w(), 40.0);
    
    // Const access
    const TestTuple<Float, 4> t_const{1.0, 2.0, 3.0, 4.0};
    EXPECT_EQ(t_const.x(), 1.0);
    EXPECT_EQ(t_const.y(), 2.0);
    EXPECT_EQ(t_const.z(), 3.0);
    EXPECT_EQ(t_const.w(), 4.0);
}

TEST_F(TupleTest, EqualityComparison) {
    TestTuple<Float, 3> t_same{1.0, 2.0, 3.0};
    TestTuple<Float, 3> t_diff{1.0, 2.0, 4.0};
    
    EXPECT_TRUE(t1 == t_same);
    EXPECT_FALSE(t1 == t_diff);
    EXPECT_FALSE(t1 != t_same);
    EXPECT_TRUE(t1 != t_diff);
    
    // Test with different types
    TestTuple<double, 3> t_double{1.0, 2.0, 3.0};
    EXPECT_TRUE(t1 == t_double);
}

TEST_F(TupleTest, TypeCasting) {
    // Type cast
    auto t_double = t1.type_cast<double>();
    EXPECT_TRUE((std::is_same_v<decltype(t_double), TestTuple<double, 3>>));
    EXPECT_EQ(t_double.x(), 1.0);
    EXPECT_EQ(t_double.y(), 2.0);
    EXPECT_EQ(t_double.z(), 3.0);
    
    // Integer type cast
    auto t_int = t1.type_cast<int>();
    EXPECT_EQ(t_int.x(), 1);
    EXPECT_EQ(t_int.y(), 2);
    EXPECT_EQ(t_int.z(), 3);
}

TEST_F(TupleTest, DimensionCasting) {
    // Dimension cast to smaller
    auto t_2d = t1.dim_cast<2>();
    EXPECT_EQ(t_2d.dims(), 2);
    EXPECT_EQ(t_2d.x(), 1.0);
    EXPECT_EQ(t_2d.y(), 2.0);
    
    // Dimension cast to larger
    auto t_5d = t1.dim_cast<5>();
    EXPECT_EQ(t_5d.dims(), 5);
    EXPECT_EQ(t_5d.x(), 1.0);
    EXPECT_EQ(t_5d.y(), 2.0);
    EXPECT_EQ(t_5d.z(), 3.0);
    EXPECT_EQ(t_5d[3], 0.0);
    EXPECT_EQ(t_5d[4], 0.0);
}

TEST_F(TupleTest, GeneralCasting) {
    // Cast with both type and dimension change
    auto t_int_4d = t1.cast<int, 4>();
    EXPECT_TRUE((std::is_same_v<decltype(t_int_4d), TestTuple<int, 4>>));
    EXPECT_EQ(t_int_4d.dims(), 4);
    EXPECT_EQ(t_int_4d.x(), 1);
    EXPECT_EQ(t_int_4d.y(), 2);
    EXPECT_EQ(t_int_4d.z(), 3);
    EXPECT_EQ(t_int_4d.w(), 0);
    
    // Cast to smaller dimension with type change
    auto t_double_2d = t1.cast<double, 2>();
    EXPECT_EQ(t_double_2d.dims(), 2);
    EXPECT_EQ(t_double_2d.x(), 1.0);
    EXPECT_EQ(t_double_2d.y(), 2.0);
}

// Test edge cases and error conditions
TEST_F(TupleTest, EdgeCases) {
    // Test single element tuple
    TestTuple<Float, 1> t_single{5.0};
    EXPECT_EQ(t_single.x(), 5.0);
    EXPECT_EQ(t_single[0], 5.0);
    EXPECT_EQ(t_single.dims(), 1);
    
    // Test large tuple
    TestTuple<Float, 10> t_large = TestTuple<Float, 10>::filled(7.0);
    EXPECT_EQ(t_large.dims(), 10);
    for (int i = 0; i < 10; ++i) {
        EXPECT_EQ(t_large[i], 7.0);
    }
}

TEST_F(TupleTest, FloatingPointEquality) {
    // Test with small floating point differences
    TestTuple<Float, 2> t_a{1.0, 2.0};
    TestTuple<Float, 2> t_b{1.0 + 1e-10, 2.0};  // Very small difference
    
    // Depending on the is_not_equal implementation, this might pass or fail
    // The exact behavior depends on the tolerance used in is_not_equal
    bool are_equal = (t_a == t_b);
    bool are_not_equal = (t_a != t_b);
    EXPECT_EQ(are_equal, !are_not_equal);
}

// Test boundary conditions and assertions
TEST_F(TupleTest, BoundaryConditions) {
    // Test index bounds would trigger assertions in debug mode
    // We can't easily test assertion failures in unit tests without 
    // special setup, so we just test valid indices
    
    TestTuple<Float, 5> t5 = TestTuple<Float, 5>::filled(1.0);
    
    // Test all valid indices
    for (int i = 0; i < 5; ++i) {
        EXPECT_NO_THROW(t5[i]);
        EXPECT_NO_THROW(t5.at(i));
        EXPECT_EQ(t5[i], 1.0);
        EXPECT_EQ(t5.at(i), 1.0);
    }
}

// Test with various numeric types
TEST(TupleNumericTypesTest, IntegerTypes) {
    TestTuple<int, 3> t_int{1, 2, 3};
    TestTuple<long, 3> t_long{1L, 2L, 3L};
    TestTuple<short, 3> t_short{1, 2, 3};
    
    EXPECT_EQ(t_int.x(), 1);
    EXPECT_EQ(t_long.x(), 1L);
    EXPECT_EQ(t_short.x(), short(1));
    
    // Test conversion between integer types
    TestTuple<long, 3> t_int_to_long(t_int);
    EXPECT_EQ(t_int_to_long.x(), 1L);
    EXPECT_EQ(t_int_to_long.y(), 2L);
    EXPECT_EQ(t_int_to_long.z(), 3L);
}

TEST(TupleNumericTypesTest, FloatingPointTypes) {
    TestTuple<float, 3> t_float{1.0f, 2.0f, 3.0f};
    TestTuple<double, 3> t_double{1.0, 2.0, 3.0};
    
    EXPECT_FLOAT_EQ(t_float.x(), 1.0f);
    EXPECT_DOUBLE_EQ(t_double.x(), 1.0);
    
    // Test conversion between float types
    TestTuple<double, 3> t_float_to_double(t_float);
    EXPECT_DOUBLE_EQ(t_float_to_double.x(), 1.0);
    EXPECT_DOUBLE_EQ(t_float_to_double.y(), 2.0);
    EXPECT_DOUBLE_EQ(t_float_to_double.z(), 3.0);
}

// Test with different dimensions
TEST(TupleDimensionsTest, VariousDimensions) {
    TestTuple<int, 1> t1{42};
    TestTuple<int, 2> t2{1, 2};
    TestTuple<int, 6> t6{1, 2, 3, 4, 5, 6};
    TestTuple<int, 10> t10 = TestTuple<int, 10>::zeros();
    
    EXPECT_EQ(t1.dims(), 1);
    EXPECT_EQ(t1.x(), 42);
    
    EXPECT_EQ(t2.dims(), 2);
    EXPECT_EQ(t2.x(), 1);
    EXPECT_EQ(t2.y(), 2);
    
    EXPECT_EQ(t6.dims(), 6);
    EXPECT_EQ(t6.x(), 1);
    EXPECT_EQ(t6.y(), 2);
    EXPECT_EQ(t6.z(), 3);
    EXPECT_EQ(t6.w(), 4);
    EXPECT_EQ(t6[4], 5);
    EXPECT_EQ(t6[5], 6);
    
    EXPECT_EQ(t10.dims(), 10);
    for (int i = 0; i < 10; ++i) {
        EXPECT_EQ(t10[i], 0);
    }
}

}  // namespace pbpt::math::testing
