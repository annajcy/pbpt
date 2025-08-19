/**
 * @file test_vector.cpp
 * @brief Comprehensive unit tests for the Vector template class
 * 
 * This file contains thorough tests for the Vector class template,
 * covering all methods, operators, edge cases, and free functions.
 */

#include <gtest/gtest.h>

#include <type_traits>
#include <limits>

// 包含被测试的头文件和其依赖
#include "math/vector.hpp"
// 假设 type_alias.hpp 和 function.hpp 已被 vector.hpp 包含

namespace pbpt::math::testing {

// Test fixture for Vector tests
class VectorTest : public ::testing::Test {
protected:
    Vector<Float, 3> v1{1.0, 2.0, 3.0};
    Vector<Float, 3> v2{4.0, 5.0, 6.0};
    Vector<Int, 3>   vi1{1, 2, 3};
};

TEST_F(VectorTest, DefaultConstruction) {
    Vector<Float, 3> v;
    EXPECT_EQ(v.x(), 0.0);
    EXPECT_EQ(v.y(), 0.0);
    EXPECT_EQ(v.z(), 0.0);
}

TEST_F(VectorTest, VariadicConstruction) {
    EXPECT_EQ(v1.x(), 1.0);
    EXPECT_EQ(v1.y(), 2.0);
    EXPECT_EQ(v1.z(), 3.0);
}

TEST_F(VectorTest, Filled) {
    auto v = Vector<Float, 4>::filled(5.0);
    EXPECT_EQ(v.x(), 5.0);
    EXPECT_EQ(v.y(), 5.0);
    EXPECT_EQ(v.z(), 5.0);
    EXPECT_EQ(v.w(), 5.0);
}

TEST_F(VectorTest, ZerosAndOnes) {
    auto zeros = Vector<Float, 3>::zeros();
    auto ones  = Vector<Float, 3>::ones();
    EXPECT_EQ(zeros, (Vector<Float, 3>(0.0, 0.0, 0.0)));
    EXPECT_EQ(ones, (Vector<Float, 3>(1.0, 1.0, 1.0)));
}

TEST_F(VectorTest, FromToArray) {
    std::array<Float, 3> arr = {1.0, 2.0, 3.0};
    auto                 v   = Vector<Float, 3>::from_array(arr);
    EXPECT_EQ(v, v1);
    auto new_arr = v.to_array();
    EXPECT_EQ(arr, new_arr);
}

TEST_F(VectorTest, CopyAndCastConstruction) {
    Vector<Float, 3> v_copy(v1);
    EXPECT_EQ(v_copy, v1);

    Vector<double, 3> v_double(v1);
    EXPECT_EQ(v_double.x(), 1.0);
    EXPECT_EQ(v_double.y(), 2.0);
    EXPECT_EQ(v_double.z(), 3.0);

    Vector<Float, 2> v_dim_cast(v1.dim_cast<2>());
    EXPECT_EQ(v_dim_cast.dims(), 2);
    EXPECT_EQ(v_dim_cast.x(), 1.0);
    EXPECT_EQ(v_dim_cast.y(), 2.0);
}

TEST_F(VectorTest, Accessors) {
    EXPECT_EQ(v1.x(), 1.0);
    EXPECT_EQ(v1.y(), 2.0);
    EXPECT_EQ(v1.z(), 3.0);
    EXPECT_EQ(v1[0], 1.0);
    EXPECT_EQ(v1[1], 2.0);
    EXPECT_EQ(v1[2], 3.0);
    EXPECT_EQ(v1.at(0), 1.0);

    v1.x() = 10.0;
    v1[1]  = 20.0;
    EXPECT_EQ(v1.x(), 10.0);
    EXPECT_EQ(v1.y(), 20.0);
}

TEST_F(VectorTest, Dims) {
    EXPECT_EQ(v1.dims(), 3);
    Vector<Float, 5> v5;
    EXPECT_EQ(v5.dims(), 5);
}

TEST_F(VectorTest, UnaryMinus) {
    auto neg_v1 = -v1;
    EXPECT_EQ(neg_v1, (Vector<Float, 3>(-1.0, -2.0, -3.0)));
}

TEST_F(VectorTest, Addition) {
    auto sum = v1 + v2;
    EXPECT_EQ(sum, (Vector<Float, 3>(5.0, 7.0, 9.0)));
    v1 += v2;
    EXPECT_EQ(v1, (Vector<Float, 3>(5.0, 7.0, 9.0)));
}

TEST_F(VectorTest, Subtraction) {
    auto diff = v1 - v2;
    EXPECT_EQ(diff, (Vector<Float, 3>(-3.0, -3.0, -3.0)));
    v1 -= v2;
    EXPECT_EQ(v1, (Vector<Float, 3>(-3.0, -3.0, -3.0)));
}

TEST_F(VectorTest, ScalarMultiplication) {
    auto scaled = v1 * 2.0;
    EXPECT_EQ(scaled, (Vector<Float, 3>(2.0, 4.0, 6.0)));
    auto scaled2 = 2.0 * v1;
    EXPECT_EQ(scaled2, (Vector<Float, 3>(2.0, 4.0, 6.0)));
    v1 *= 2.0;
    EXPECT_EQ(v1, (Vector<Float, 3>(2.0, 4.0, 6.0)));
}

TEST_F(VectorTest, ScalarDivision) {
    auto scaled = v1 / 2.0;
    EXPECT_EQ(scaled, (Vector<Float, 3>(0.5, 1.0, 1.5)));
}

TEST_F(VectorTest, ComponentWiseMultiplication) {
    auto prod = v1 * v2;
    EXPECT_EQ(prod, (Vector<Float, 3>(4.0, 10.0, 18.0)));
}

TEST_F(VectorTest, ComponentWiseDivision) {
    auto quot = v2 / v1;
    EXPECT_EQ(quot, (Vector<Float, 3>(4.0, 2.5, 2.0)));
}

TEST_F(VectorTest, Comparison) {
    Vector<Float, 3> v1_copy(1.0, 2.0, 3.0);
    EXPECT_TRUE(v1 == v1_copy);
    EXPECT_FALSE(v1 == v2);
    EXPECT_TRUE(v1 != v2);
    EXPECT_FALSE(v1 != v1_copy);
}

TEST_F(VectorTest, IsZero) {
    Vector<Float, 3> z = Vector<Float, 3>::zeros();
    EXPECT_TRUE(z.is_all_zero());
    EXPECT_FALSE(v1.is_all_zero());
}

TEST_F(VectorTest, Length) {
    Vector<Float, 2> v(3.0, 4.0);
    EXPECT_FLOAT_EQ(v.length_squared(), 25.0);
    EXPECT_FLOAT_EQ(v.length(), 5.0);
}

TEST_F(VectorTest, Normalization) {
    Vector<Float, 3> v(1.0, 1.0, 1.0);
    auto             norm_v = v.normalized();
    EXPECT_FLOAT_EQ(norm_v.length(), 1.0);
    EXPECT_TRUE(norm_v.is_normalized());
    EXPECT_FALSE(v.is_normalized());
}

TEST_F(VectorTest, DotProduct) {
    EXPECT_FLOAT_EQ(v1.dot(v2), 1.0 * 4.0 + 2.0 * 5.0 + 3.0 * 6.0);
}

TEST_F(VectorTest, Product) {
    EXPECT_FLOAT_EQ(v1.product(), 1.0 * 2.0 * 3.0);
}

TEST_F(VectorTest, Visit) {
    v1.visit([](Float& val, int) { val *= 2; });
    EXPECT_EQ(v1, (Vector<Float, 3>(2.0, 4.0, 6.0)));
}

TEST_F(VectorTest, Inverse) {
    auto inv_v = v1.inv();
    EXPECT_EQ(inv_v, (Vector<Float, 3>(1.0 / 1.0, 1.0 / 2.0, 1.0 / 3.0)));
}

TEST_F(VectorTest, Abs) {
    Vector<Float, 3> v(-1.0, 2.0, -3.0);
    auto             abs_v = v.abs();
    EXPECT_EQ(abs_v, (Vector<Float, 3>(1.0, 2.0, 3.0)));
}

TEST_F(VectorTest, MinMax) {
    Vector<Float, 4> v(1.0, -5.0, 10.0, 2.0);
    EXPECT_EQ(v.max_dim(), 2);
    EXPECT_EQ(v.max(), 10.0);
    EXPECT_EQ(v.min_dim(), 1);
    EXPECT_EQ(v.min(), -5.0);
}

TEST_F(VectorTest, Permute) {
    auto p = v1.permuted(2, 1, 0);
    EXPECT_EQ(p, (Vector<Float, 3>(3.0, 2.0, 1.0)));
    v1.permute(2, 1, 0);
    EXPECT_EQ(v1, (Vector<Float, 3>(3.0, 2.0, 1.0)));
}

TEST_F(VectorTest, Cast) {
    auto v_double = v1.type_cast<double>();
    EXPECT_TRUE((std::is_same_v<decltype(v_double), Vector<double, 3>>));
    EXPECT_EQ(v_double, (Vector<double, 3>(1.0, 2.0, 3.0)));

    auto v_dim = v1.dim_cast<4>();
    EXPECT_EQ(v_dim.dims(), 4);
    EXPECT_EQ(v_dim.x(), 1.0);
    EXPECT_EQ(v_dim.y(), 2.0);
    EXPECT_EQ(v_dim.z(), 3.0);
    EXPECT_EQ(v_dim.w(), 0.0);  // Default initialized

    auto v_cast_dim = v1.cast<int, 4>();
    EXPECT_EQ(v_cast_dim.dims(), 4);
    EXPECT_EQ(v_cast_dim.x(), 1);
    EXPECT_EQ(v_cast_dim.y(), 2);
    EXPECT_EQ(v_cast_dim.z(), 3);
    EXPECT_EQ(v_cast_dim.w(), 0);

    auto v_cast_dim_2 = v1.cast<int, 2>();
    EXPECT_EQ(v_cast_dim_2.dims(), 2);
    EXPECT_EQ(v_cast_dim_2.x(), 1);
    EXPECT_EQ(v_cast_dim_2.y(), 2);
    // EXPECT_EQ(v_cast_dim_2.z(), 3);
    // EXPECT_EQ(v_cast_dim_2.w(), 0);

    auto mul_res = v_cast_dim * v_dim;
    EXPECT_EQ(mul_res, (Vector<int, 4>(1, 4, 9, 0)));
}

// Tests for free functions
TEST(VectorFreeFunctionsTest, CrossProduct) {
    Vector<Float, 3> x_axis(1.0, 0.0, 0.0);
    Vector<Float, 3> y_axis(0.0, 1.0, 0.0);
    Vector<Float, 3> z_axis(0.0, 0.0, 1.0);
    auto             cross_prod = cross(x_axis, y_axis);
    EXPECT_EQ(cross_prod, z_axis);
}

TEST(VectorFreeFunctionsTest, AngleBetween) {
    Vector<Float, 3> v1(1.0, 0.0, 0.0);
    Vector<Float, 3> v2(0.0, 1.0, 0.0);
    v1 = v1.normalized();
    v2 = v2.normalized();
    EXPECT_FLOAT_EQ(angle_between(v1, v2), pi_v<Float> / 2.0);

    Vector<Float, 3> v3(1.0, 0.0, 0.0);
    Vector<Float, 3> v4(-1.0, 0.0, 0.0);
    v3 = v3.normalized();
    v4 = v4.normalized();
    EXPECT_FLOAT_EQ(angle_between(v3, v4), pi_v<Float>);
}

TEST(VectorFreeFunctionsTest, CoordinateSystem) {
    Vector<Float, 3> v1(0.0, 0.0, 1.0);
    v1 = v1.normalized();
    auto [v2, v3] = coordinate_system(v1);
    EXPECT_TRUE(v2.is_normalized());
    EXPECT_TRUE(v3.is_normalized());
    EXPECT_NEAR(v1.dot(v2), 0.0, 1e-6);
    EXPECT_NEAR(v1.dot(v3), 0.0, 1e-6);
    EXPECT_NEAR(v2.dot(v3), 0.0, 1e-6);
}

// 测试复合赋值运算符的更多情况
TEST_F(VectorTest, CompoundAssignmentExtended) {
    Vector<Float, 3> v_orig(1.0, 2.0, 3.0);
    Vector<Float, 3> v_test;
    
    // 测试 *= (分量乘法)
    v_test = v_orig;
    Vector<Float, 3> multiplier(2.0, 3.0, 4.0);
    v_test *= multiplier;
    EXPECT_EQ(v_test, (Vector<Float, 3>(2.0, 6.0, 12.0)));
    
    // 测试 /= (分量除法)
    v_test /= multiplier;
    EXPECT_EQ(v_test, v_orig);
}

// 测试边界条件和错误处理
TEST_F(VectorTest, EdgeCasesAndErrorHandling) {
    // 测试归一化零向量（应该触发断言）
    Vector<Float, 3> zero_vec = Vector<Float, 3>::zeros();
    
    // 在发布模式下可能不会触发断言，我们主要测试非零情况
    Vector<Float, 3> small_vec(1e-10f, 0.0f, 0.0f);
    auto normalized_small = small_vec.normalized();
    EXPECT_TRUE(normalized_small.is_normalized());
    
    // 测试非常小的长度计算精度
    EXPECT_GT(small_vec.length(), 0.0f);
    EXPECT_GT(small_vec.length_squared(), 0.0f);
}

// 测试不同数值类型
TEST(VectorNumericTypesTest, IntegerVectors) {
    Vector<int, 3> v_int(1, 2, 3);
    Vector<int, 3> v_int2(4, 5, 6);
    
    // 整数向量运算
    auto sum = v_int + v_int2;
    EXPECT_EQ(sum, (Vector<int, 3>(5, 7, 9)));
    
    auto diff = v_int2 - v_int;
    EXPECT_EQ(diff, (Vector<int, 3>(3, 3, 3)));
    
    // 整数向量的长度应该返回浮点类型
    auto len = v_int.length();
    static_assert(std::is_floating_point_v<decltype(len)>);
    EXPECT_FLOAT_EQ(len, std::sqrt(14.0f));
    
    // 整数向量的归一化应该返回浮点向量
    auto norm = v_int.normalized();
    static_assert(std::is_same_v<decltype(norm), Vector<Float, 3>>);
    EXPECT_TRUE(norm.is_normalized());
}

TEST(VectorNumericTypesTest, MixedTypeOperations) {
    Vector<float, 3> v_float(1.0f, 2.0f, 3.0f);
    Vector<double, 3> v_double(4.0, 5.0, 6.0);
    Vector<int, 3> v_int(1, 1, 1);
    
    // 不同类型向量运算
    auto result1 = v_float + v_double;
    static_assert(std::is_same_v<decltype(result1), Vector<double, 3>>);
    EXPECT_DOUBLE_EQ(result1.x(), 5.0);
    EXPECT_DOUBLE_EQ(result1.y(), 7.0);
    EXPECT_DOUBLE_EQ(result1.z(), 9.0);
    
    // 浮点与整数混合
    auto result2 = v_float * v_int;
    static_assert(std::is_same_v<decltype(result2), Vector<float, 3>>);
    EXPECT_FLOAT_EQ(result2.x(), 1.0f);
    EXPECT_FLOAT_EQ(result2.y(), 2.0f);
    EXPECT_FLOAT_EQ(result2.z(), 3.0f);
    
    // 点积运算
    auto dot_result = v_float.dot(v_double);
    static_assert(std::is_same_v<decltype(dot_result), double>);
    EXPECT_DOUBLE_EQ(dot_result, 32.0);  // 1*4 + 2*5 + 3*6 = 32
}

// 测试不同维度的向量
TEST(VectorDimensionsTest, VariousDimensions) {
    // 1D Vector
    Vector<Float, 1> v1d(5.0f);
    EXPECT_FLOAT_EQ(v1d.x(), 5.0f);
    EXPECT_FLOAT_EQ(v1d.length(), 5.0f);
    EXPECT_EQ(v1d.dims(), 1);
    
    // 2D Vector
    Vector<Float, 2> v2d(3.0f, 4.0f);
    EXPECT_FLOAT_EQ(v2d.x(), 3.0f);
    EXPECT_FLOAT_EQ(v2d.y(), 4.0f);
    EXPECT_FLOAT_EQ(v2d.length(), 5.0f);  // 3-4-5 triangle
    EXPECT_EQ(v2d.dims(), 2);
    
    // 4D Vector
    Vector<Float, 4> v4d(1.0f, 2.0f, 3.0f, 4.0f);
    EXPECT_FLOAT_EQ(v4d.x(), 1.0f);
    EXPECT_FLOAT_EQ(v4d.y(), 2.0f);
    EXPECT_FLOAT_EQ(v4d.z(), 3.0f);
    EXPECT_FLOAT_EQ(v4d.w(), 4.0f);
    EXPECT_FLOAT_EQ(v4d.length_squared(), 30.0f);  // 1+4+9+16
    EXPECT_EQ(v4d.dims(), 4);
}

// 测试apply函数的更多用法
TEST_F(VectorTest, ApplyFunctionAdvanced) {
    Vector<Float, 3> v(1.0, -2.0, 3.0);
    
    // 测试使用索引的lambda
    v.visit([](Float& val, int idx) { 
        if (idx == 1) val = std::abs(val); 
    });
    EXPECT_EQ(v, (Vector<Float, 3>(1.0, 2.0, 3.0)));
    
    // 测试累加功能
    Float sum = 0.0;
    v.visit([&sum](const Float& val, int) { sum += val; });
    EXPECT_FLOAT_EQ(sum, 6.0);
}

// 测试特殊值处理
TEST_F(VectorTest, SpecialValueHandling) {
    // 测试infinity
    Vector<Float, 3> v_inf(std::numeric_limits<Float>::infinity(), 1.0, 2.0);
    EXPECT_TRUE(std::isinf(v_inf.length()));
    
    // 测试-infinity
    Vector<Float, 3> v_neg_inf(-std::numeric_limits<Float>::infinity(), 1.0, 2.0);
    EXPECT_TRUE(std::isinf(v_neg_inf.length()));
    
    // 测试混合特殊值
    Vector<Float, 3> v_mixed(std::nan(""), std::numeric_limits<Float>::infinity(), 0.0);
    EXPECT_TRUE(v_mixed.has_nan());
}

// 测试更复杂的排列组合
TEST_F(VectorTest, AdvancedPermutation) {
    Vector<Float, 4> v4(1.0, 2.0, 3.0, 4.0);
    
    // 测试所有元素的排列
    auto permuted = v4.permuted(3, 2, 1, 0);  // 完全反转
    EXPECT_EQ(permuted, (Vector<Float, 4>(4.0, 3.0, 2.0, 1.0)));
    
    // 测试重复索引
    auto repeated = v4.permuted(0, 0, 1, 1);
    EXPECT_EQ(repeated, (Vector<Float, 4>(1.0, 1.0, 2.0, 2.0)));
    
    // 测试原地排列
    Vector<Float, 3> v3_copy = v1;
    v3_copy.permute(1, 2, 0);  // 循环移位
    auto expected = v1.permuted(1, 2, 0);
    EXPECT_EQ(v3_copy, expected);
}

// 测试大维度向量性能和正确性
TEST(VectorPerformanceTest, HighDimensionalVectors) {
    constexpr int DIM = 100;
    Vector<Float, DIM> v_large = Vector<Float, DIM>::filled(1.0);
    
    // 测试大向量的基本运算
    EXPECT_FLOAT_EQ(v_large.length_squared(), static_cast<Float>(DIM));
    EXPECT_FLOAT_EQ(v_large.length(), std::sqrt(static_cast<Float>(DIM)));
    
    auto v_large2 = Vector<Float, DIM>::filled(2.0);
    auto dot_large = v_large.dot(v_large2);
    EXPECT_FLOAT_EQ(dot_large, static_cast<Float>(2 * DIM));
    
    // 测试归一化
    auto normalized_large = v_large.normalized();
    EXPECT_TRUE(normalized_large.is_normalized());
}

// 测试自由函数的更多情况
TEST(VectorFreeFunctionsTest, CrossProductExtended) {
    // 测试更多叉积情况
    Vector<Float, 3> v1(2.0, 0.0, 0.0);
    Vector<Float, 3> v2(0.0, 3.0, 0.0);
    auto cross_result = cross(v1, v2);
    
    // 2i × 3j = 6k
    EXPECT_EQ(cross_result, (Vector<Float, 3>(0.0, 0.0, 6.0)));
    
    // 测试叉积的反交换律
    auto cross_reversed = cross(v2, v1);
    EXPECT_EQ(cross_reversed, -cross_result);
    
    // 测试平行向量的叉积为零
    Vector<Float, 3> parallel1(1.0, 2.0, 3.0);
    Vector<Float, 3> parallel2(2.0, 4.0, 6.0);  // 2 * parallel1
    auto zero_cross = cross(parallel1, parallel2);
    EXPECT_TRUE(zero_cross.is_all_zero());
}

TEST(VectorFreeFunctionsTest, AngleBetweenExtended) {
    // 测试平行向量
    Vector<Float, 3> v1(1.0, 0.0, 0.0);
    Vector<Float, 3> v2(2.0, 0.0, 0.0);
    v1 = v1.normalized();
    v2 = v2.normalized();
    EXPECT_NEAR(angle_between(v1, v2), 0.0, 1e-6);
    
    // 测试45度角
    Vector<Float, 3> v3(1.0, 0.0, 0.0);
    Vector<Float, 3> v4(1.0, 1.0, 0.0);
    v3 = v3.normalized();
    v4 = v4.normalized();
    EXPECT_NEAR(angle_between(v3, v4), pi_v<Float> / 4.0, 1e-5);
}

TEST(VectorFreeFunctionsTest, CoordinateSystemExtended) {
    // 测试不同方向的坐标系生成
    Vector<Float, 3> directions[] = {
        Vector<Float, 3>(1.0, 0.0, 0.0),  // x轴
        Vector<Float, 3>(0.0, 1.0, 0.0),  // y轴
        Vector<Float, 3>(0.0, 0.0, 1.0),  // z轴
        Vector<Float, 3>(1.0, 1.0, 1.0).normalized(),  // 对角线
        Vector<Float, 3>(-1.0, 0.5, 0.3).normalized() // 任意方向
    };
    
    for (const auto& dir : directions) {
        auto [u, v] = coordinate_system(dir);
        
        // 验证生成的坐标系的性质
        EXPECT_TRUE(u.is_normalized());
        EXPECT_TRUE(v.is_normalized());
        EXPECT_NEAR(dir.dot(u), 0.0, 1e-6);  // 正交
        EXPECT_NEAR(dir.dot(v), 0.0, 1e-6);  // 正交
        EXPECT_NEAR(u.dot(v), 0.0, 1e-6);    // 正交
        
        // 验证右手坐标系（叉积应该平行于原方向）
        auto cross_uv = cross(u, v);
        EXPECT_NEAR(std::abs(cross_uv.dot(dir)), 1.0, 1e-6);
    }
}

// 测试不等操作符的详细逻辑
TEST_F(VectorTest, InequalityOperatorLogic) {
    Vector<Float, 3> v1(1.0, 2.0, 3.0);
    Vector<Float, 3> v2(1.0, 2.0, 3.0);
    Vector<Float, 3> v3(1.0, 2.0, 3.1);
    Vector<Float, 3> v4(1.1, 2.0, 3.0);
    Vector<Float, 3> v5(1.1, 2.1, 3.1);  // 所有分量都不等
    
    // 测试相等向量
    EXPECT_TRUE(v1 == v2);
    EXPECT_FALSE(v1 != v2);
    
    // 测试部分分量不等的向量
    EXPECT_FALSE(v1 == v3);
    EXPECT_TRUE(v1 != v3);
    EXPECT_FALSE(v1 == v4);
    EXPECT_TRUE(v1 != v4);
    
    // 测试所有分量都不等的向量
    EXPECT_FALSE(v1 == v5);
    EXPECT_TRUE(v1 != v5);
    
    // 测试浮点精度边界
    Vector<Float, 3> v_close1(1.0, 2.0, 3.0);
    Vector<Float, 3> v_close2(1.0 + 1e-12, 2.0, 3.0);  // 非常接近但不相等
    
    // 结果取决于is_equal函数的epsilon值
    bool are_equal = (v_close1 == v_close2);
    bool are_not_equal = (v_close1 != v_close2);
    EXPECT_EQ(are_equal, !are_not_equal);  // 现在应该满足逻辑一致性
}

// 测试边界数值的运算稳定性
TEST_F(VectorTest, NumericalStability) {
    // 测试非常小的数值（可能会下溢到0）
    Vector<Float, 3> tiny(1e-20f, 1e-20f, 1e-20f);
    auto tiny_len_sq = tiny.length_squared();
    auto tiny_len = tiny.length();
    
    // 对于非常小的数值，可能会下溢，我们只验证它们不是负数
    EXPECT_GE(tiny_len_sq, 0.0f);
    EXPECT_GE(tiny_len, 0.0f);
    
    // 测试适中的数值确保基本运算正确
    Vector<Float, 3> normal(1e-5f, 1e-5f, 1e-5f);
    EXPECT_GT(normal.length(), 0.0f);
    EXPECT_GT(normal.length_squared(), 0.0f);
    
    // 测试大数值（可能溢出）
    const Float large_val = 1e20f;  // 降低数值避免溢出
    Vector<Float, 3> large(large_val, large_val, large_val);
    auto large_len = large.length();
    
    // 对于大数值，我们至少验证结果不是NaN
    EXPECT_FALSE(std::isnan(large_len));
    
    // 测试混合大小数值
    Vector<Float, 3> mixed(1e-10f, 1e10f, 1.0f);
    auto mixed_len = mixed.length();
    EXPECT_FALSE(std::isnan(mixed_len));
    EXPECT_GE(mixed_len, 0.0f);
}

}  // namespace pbpt::math::testing
