#include <gtest/gtest.h>

#include <sstream>
#include <stdexcept>
#include <type_traits>

// 包含被测试的头文件和其依赖
#include "math/geometry/vector.hpp"
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

    Vector<Float, 2> v_dim_cast(v1);
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
    EXPECT_TRUE(z.is_zero());
    EXPECT_FALSE(v1.is_zero());
}

TEST_F(VectorTest, HasNaN) {
    Vector<Float, 3> v_nan(1.0, std::nan(""), 3.0);
    EXPECT_TRUE(v_nan.has_nan());
    EXPECT_FALSE(v1.has_nan());
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

TEST_F(VectorTest, Apply) {
    v1.apply([](Float& val, int) { val *= 2; });
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

TEST_F(VectorTest, OutputStream) {
    std::stringstream ss;
    ss << v1;
    EXPECT_EQ(ss.str(), "Vector<f, 3>(1, 2, 3)");
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
    auto [v2, v3] = coordinate_system(v1);
    EXPECT_TRUE(v2.is_normalized());
    EXPECT_TRUE(v3.is_normalized());
    EXPECT_NEAR(v1.dot(v2), 0.0, 1e-6);
    EXPECT_NEAR(v1.dot(v3), 0.0, 1e-6);
    EXPECT_NEAR(v2.dot(v3), 0.0, 1e-6);
}

}  // namespace pbpt::math::testing
