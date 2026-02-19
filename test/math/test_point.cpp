/**
 * @file test_point.cpp
 * @brief Comprehensive unit tests for the Point template class
 *
 * This file contains thorough tests for the Point class template,
 * covering all methods, operators, and edge cases.
 */

#include <gtest/gtest.h>
#include <type_traits>
#include <vector>

#include "pbpt/pbpt.h"

namespace pbpt::math::testing {

// 测试 Point 类的基本构造和访问器
TEST(PointTest, ConstructionAndAccessors) {
    // 1. 默认构造函数
    Pt3 p_default;
    EXPECT_FLOAT_EQ(p_default.x(), 0.0);
    EXPECT_FLOAT_EQ(p_default.y(), 0.0);
    EXPECT_FLOAT_EQ(p_default.z(), 0.0);

    // 2. 单值构造函数
    Pt3 p_uniform = Pt3::filled(1.2);
    EXPECT_FLOAT_EQ(p_uniform.x(), 1.2);
    EXPECT_FLOAT_EQ(p_uniform.y(), 1.2);
    EXPECT_FLOAT_EQ(p_uniform.z(), 1.2);

    // 3. 多参数构造函数
    Pt3 p_multi(1.0, 2.0, 3.0);
    EXPECT_FLOAT_EQ(p_multi.x(), 1.0);
    EXPECT_FLOAT_EQ(p_multi.y(), 2.0);
    EXPECT_FLOAT_EQ(p_multi.z(), 3.0);

    // 4. 从 Vec 构造
    Vec3 v_source(4.0, 5.0, 6.0);
    Pt3 p_from_vec = Pt3::from_vector(v_source);
    EXPECT_FLOAT_EQ(p_from_vec.x(), 4.0);
    EXPECT_FLOAT_EQ(p_from_vec.y(), 5.0);
    EXPECT_FLOAT_EQ(p_from_vec.z(), 6.0);

    // 5. 下标访问器 (const)
    const Pt3 p_const = p_multi;
    EXPECT_FLOAT_EQ(p_const[0], 1.0);
    EXPECT_FLOAT_EQ(p_const[1], 2.0);
    EXPECT_FLOAT_EQ(p_const[2], 3.0);

    // 6. 下标访问器 (non-const)
    Pt3 p_mutable(0, 0, 0);
    p_mutable[0] = 7.0;
    p_mutable[1] = 8.0;
    p_mutable[2] = 9.0;
    EXPECT_FLOAT_EQ(p_mutable.x(), 7.0);
    EXPECT_FLOAT_EQ(p_mutable.y(), 8.0);
    EXPECT_FLOAT_EQ(p_mutable.z(), 9.0);
}

// 测试核心的代数运算规则
TEST(PointTest, AlgebraicOperations) {
    Pt3 p1(1.0, 2.0, 3.0);
    Pt3 p2(5.0, 7.0, 9.0);
    Vec3 v(4.0, 5.0, 6.0);

    // 规则 1: Point - Point = Vector
    auto displacement = p2 - p1;
    // 检查返回类型是否为 Vec3
    static_assert(std::is_same_v<decltype(displacement), Vec3>, "Point - Point should yield a Vector");
    EXPECT_FLOAT_EQ(displacement.x(), 4.0);
    EXPECT_FLOAT_EQ(displacement.y(), 5.0);
    EXPECT_FLOAT_EQ(displacement.z(), 6.0);

    // 规则 2: Point + Vector = Point
    auto p_new = p1 + v;
    // 检查返回类型是否为 Point3
    static_assert(std::is_same_v<decltype(p_new), Pt3>, "Point + Vector should yield a Point");
    EXPECT_FLOAT_EQ(p_new.x(), p2.x());
    EXPECT_FLOAT_EQ(p_new.y(), p2.y());
    EXPECT_FLOAT_EQ(p_new.z(), p2.z());

    // 规则 3: Point - Vector = Point
    auto p_subtracted = p2 - v;
    static_assert(std::is_same_v<decltype(p_subtracted), Pt3>, "Point - Vector should yield a Point");
    EXPECT_FLOAT_EQ(p_subtracted.x(), p1.x());
    EXPECT_FLOAT_EQ(p_subtracted.y(), p1.y());
    EXPECT_FLOAT_EQ(p_subtracted.z(), p1.z());
}

// 测试复合赋值运算符
TEST(PointTest, CompoundAssignment) {
    Pt3 p(1.0, 2.0, 3.0);
    const Vec3 v(4.0, 5.0, 6.0);

    // 测试 +=
    p += v;
    EXPECT_FLOAT_EQ(p.x(), 5.0);
    EXPECT_FLOAT_EQ(p.y(), 7.0);
    EXPECT_FLOAT_EQ(p.z(), 9.0);

    // 测试 -=
    p -= v;
    EXPECT_FLOAT_EQ(p.x(), 1.0);
    EXPECT_FLOAT_EQ(p.y(), 2.0);
    EXPECT_FLOAT_EQ(p.z(), 3.0);
}

// 测试辅助函数 mid_point
TEST(PointTest, MidPointFunction) {
    Pt3 p1(0.0, 0.0, 0.0);
    Pt3 p2(10.0, -20.0, 30.0);

    Pt3 mid = p1.mid(p2);

    EXPECT_FLOAT_EQ(mid.x(), 5.0);
    EXPECT_FLOAT_EQ(mid.y(), -10.0);
    EXPECT_FLOAT_EQ(mid.z(), 15.0);
}

// 测试显式类型转换
TEST(PointTest, ExplicitConversion) {
    Pt3 p(1, 2, 3);

    // 显式转换
    Vec3 v = p.to_vector();
    EXPECT_FLOAT_EQ(v.x(), 1.0);
    EXPECT_FLOAT_EQ(v.y(), 2.0);
    EXPECT_FLOAT_EQ(v.z(), 3.0);

    // 注意：隐式转换应该失败，因为从 Vec 构造 Point 的构造函数是 explicit 的。
    // Vec3 implicit_v = p; // 这行代码如果取消注释，应该会导致编译失败
}

// --- 编译时测试（文档性质） ---
// 下面的代码块通过注释的形式展示了哪些操作会因为我们的设计而编译失败。
// 这是对代数正确性设计的一种“文档化测试”。
TEST(PointTest, CompileTimeAlgebraicChecks) {
    Pt3 p1(1, 2, 3);
    Pt3 p2(4, 5, 6);
    Vec3 v(1, 1, 1);

    // 以下操作是代数上非法的，并且应该会导致编译错误。
    // 将它们注释掉，以使测试能够通过编译。

    // // 错误: 两个点不能相加
    // Point3 p_sum = p1 + p2;

    // // 错误: 点不能与标量相乘
    // Point3 p_scaled = p1 * 2.0;

    // // 错误: 点没有一元负号
    // Vec3 v_neg_p = -p1;

    // // 错误: 点没有长度
    // Float len = p1.length(); // .length() 是 Vec 的方法，Point 没有暴露

    // // 错误: 点没有点积运算
    // Float d = p1.dot(p2);

    SUCCEED() << "Compile-time checks are documented and expected to fail if "
                 "uncommented.";
}

// 测试距离计算
TEST(PointTest, DistanceCalculation) {
    Pt3 p1(0.0, 0.0, 0.0);
    Pt3 p2(3.0, 4.0, 0.0);  // 3-4-5 triangle
    Pt3 p3(1.0, 1.0, 1.0);

    // 测试 distance_squared
    EXPECT_FLOAT_EQ(p1.distance_squared(p2), 25.0);
    EXPECT_FLOAT_EQ(p2.distance_squared(p1), 25.0);  // 对称性
    EXPECT_FLOAT_EQ(p1.distance_squared(p1), 0.0);   // 自距离为0

    // 测试 distance
    EXPECT_FLOAT_EQ(p1.distance(p2), 5.0);
    EXPECT_FLOAT_EQ(p2.distance(p1), 5.0);  // 对称性
    EXPECT_FLOAT_EQ(p1.distance(p1), 0.0);  // 自距离为0

    // 测试 3D 距离
    EXPECT_FLOAT_EQ(p1.distance_squared(p3), 3.0);
    EXPECT_FLOAT_EQ(p1.distance(p3), std::sqrt(3.0));
}

// 测试中点计算
TEST(PointTest, MidpointCalculation) {
    Pt3 p1(1.0, 2.0, 3.0);
    Pt3 p2(5.0, 6.0, 7.0);

    // 测试两点中点
    auto mid = p1.mid(p2);
    EXPECT_FLOAT_EQ(mid.x(), 3.0);
    EXPECT_FLOAT_EQ(mid.y(), 4.0);
    EXPECT_FLOAT_EQ(mid.z(), 5.0);

    // 测试中点的对称性
    auto mid_reversed = p2.mid(p1);
    EXPECT_FLOAT_EQ(mid.x(), mid_reversed.x());
    EXPECT_FLOAT_EQ(mid.y(), mid_reversed.y());
    EXPECT_FLOAT_EQ(mid.z(), mid_reversed.z());

    // 测试多点中点（静态方法）
    std::vector<Pt3> points = {Pt3(0.0, 0.0, 0.0), Pt3(3.0, 3.0, 3.0), Pt3(6.0, 6.0, 6.0)};
    auto center = Pt3::mid(points);
    EXPECT_FLOAT_EQ(center.x(), 3.0);
    EXPECT_FLOAT_EQ(center.y(), 3.0);
    EXPECT_FLOAT_EQ(center.z(), 3.0);

    // 测试单点的中点
    std::vector<Pt3> single_point = {Pt3(1.0, 2.0, 3.0)};
    auto single_mid = Pt3::mid(single_point);
    EXPECT_FLOAT_EQ(single_mid.x(), 1.0);
    EXPECT_FLOAT_EQ(single_mid.y(), 2.0);
    EXPECT_FLOAT_EQ(single_mid.z(), 3.0);
}

// 测试夹位（clamp）功能
TEST(PointTest, ClampFunction) {
    Pt3 p(1.5, -2.5, 10.5);
    Pt3 low(0.0, 0.0, 0.0);
    Pt3 high(2.0, 2.0, 8.0);

    auto clamped = p.clamp(low, high);
    EXPECT_FLOAT_EQ(clamped.x(), 1.5);  // 在范围内，不变
    EXPECT_FLOAT_EQ(clamped.y(), 0.0);  // 低于下限，夹到下限
    EXPECT_FLOAT_EQ(clamped.z(), 8.0);  // 高于上限，夹到上限

    // 测试边界情况
    Pt3 p_on_boundary(0.0, 2.0, 4.0);
    auto clamped_boundary = p_on_boundary.clamp(low, high);
    EXPECT_FLOAT_EQ(clamped_boundary.x(), 0.0);
    EXPECT_FLOAT_EQ(clamped_boundary.y(), 2.0);
    EXPECT_FLOAT_EQ(clamped_boundary.z(), 4.0);
}

// 测试类型转换
TEST(PointTest, TypeConversion) {
    // 测试不同类型之间的转换
    Point<int, 3> p_int(1, 2, 3);
    Point<float, 3> p_float = p_int.type_cast<float>();

    EXPECT_FLOAT_EQ(p_float.x(), 1.0f);
    EXPECT_FLOAT_EQ(p_float.y(), 2.0f);
    EXPECT_FLOAT_EQ(p_float.z(), 3.0f);

    // 测试维度转换
    auto p_2d = p_int.dim_cast<2>();
    EXPECT_EQ(p_2d.dims(), 2);
    EXPECT_EQ(p_2d.x(), 1);
    EXPECT_EQ(p_2d.y(), 2);

    auto p_4d = p_int.dim_cast<4>();
    EXPECT_EQ(p_4d.dims(), 4);
    EXPECT_EQ(p_4d.x(), 1);
    EXPECT_EQ(p_4d.y(), 2);
    EXPECT_EQ(p_4d.z(), 3);
    EXPECT_EQ(p_4d.w(), 0);  // 默认值
}

// 测试混合类型运算
TEST(PointTest, MixedTypeOperations) {
    Point<float, 3> p_float(1.0f, 2.0f, 3.0f);
    Point<double, 3> p_double(4.0, 5.0, 6.0);
    Vector<int, 3> v_int(1, 1, 1);

    // 不同类型的点之间运算
    auto diff = p_double - p_float;
    static_assert(std::is_same_v<decltype(diff), Vector<double, 3>>);
    EXPECT_DOUBLE_EQ(diff.x(), 3.0);
    EXPECT_DOUBLE_EQ(diff.y(), 3.0);
    EXPECT_DOUBLE_EQ(diff.z(), 3.0);

    // 点与不同类型向量运算
    auto p_result = p_float + v_int;
    static_assert(std::is_same_v<decltype(p_result), Point<float, 3>>);
    EXPECT_FLOAT_EQ(p_result.x(), 2.0f);
    EXPECT_FLOAT_EQ(p_result.y(), 3.0f);
    EXPECT_FLOAT_EQ(p_result.z(), 4.0f);
}

// 测试交换律（Vector + Point = Point + Vector）
TEST(PointTest, CommutativeProperty) {
    Pt3 p(1.0, 2.0, 3.0);
    Vec3 v(4.0, 5.0, 6.0);

    auto result1 = p + v;
    auto result2 = v + p;

    EXPECT_FLOAT_EQ(result1.x(), result2.x());
    EXPECT_FLOAT_EQ(result1.y(), result2.y());
    EXPECT_FLOAT_EQ(result1.z(), result2.z());
}

// 测试不同维度的 Point
TEST(PointTest, DifferentDimensions) {
    // 1D Point
    Point<float, 1> p1d(5.0f);
    EXPECT_FLOAT_EQ(p1d.x(), 5.0f);
    EXPECT_EQ(p1d.dims(), 1);

    // 2D Point
    Pt2 p2d(1.0, 2.0);
    EXPECT_FLOAT_EQ(p2d.x(), 1.0);
    EXPECT_FLOAT_EQ(p2d.y(), 2.0);
    EXPECT_EQ(p2d.dims(), 2);

    // 4D Point
    Pt4 p4d(1.0, 2.0, 3.0, 4.0);
    EXPECT_FLOAT_EQ(p4d.x(), 1.0);
    EXPECT_FLOAT_EQ(p4d.y(), 2.0);
    EXPECT_FLOAT_EQ(p4d.z(), 3.0);
    EXPECT_FLOAT_EQ(p4d.w(), 4.0);
    EXPECT_EQ(p4d.dims(), 4);
}

// 测试整数类型的 Point
TEST(PointTest, IntegerPoints) {
    Pt3i p1(1, 2, 3);
    Pt3i p2(4, 5, 6);

    // 整数点之间的运算
    auto diff = p2 - p1;
    EXPECT_EQ(diff.x(), 3);
    EXPECT_EQ(diff.y(), 3);
    EXPECT_EQ(diff.z(), 3);

    // 距离计算（应该返回浮点数类型）
    auto dist = p1.distance(p2);
    static_assert(std::is_floating_point_v<decltype(dist)>);
    EXPECT_FLOAT_EQ(dist, std::sqrt(27.0f));
}

// 测试静态工厂方法
TEST(PointTest, StaticFactoryMethods) {
    // 测试 zeros
    auto p_zeros = Pt3::zeros();
    EXPECT_FLOAT_EQ(p_zeros.x(), 0.0);
    EXPECT_FLOAT_EQ(p_zeros.y(), 0.0);
    EXPECT_FLOAT_EQ(p_zeros.z(), 0.0);

    // 测试 ones
    auto p_ones = Pt3::ones();
    EXPECT_FLOAT_EQ(p_ones.x(), 1.0);
    EXPECT_FLOAT_EQ(p_ones.y(), 1.0);
    EXPECT_FLOAT_EQ(p_ones.z(), 1.0);

    // 测试 from_array
    std::array<Float, 3> arr = {1.5, 2.5, 3.5};
    auto p_from_arr = Pt3::from_array(arr);
    EXPECT_FLOAT_EQ(p_from_arr.x(), 1.5);
    EXPECT_FLOAT_EQ(p_from_arr.y(), 2.5);
    EXPECT_FLOAT_EQ(p_from_arr.z(), 3.5);
}

// 测试相等性比较（继承自 Tuple）
TEST(PointTest, EqualityComparison) {
    Pt3 p1(1.0, 2.0, 3.0);
    Pt3 p2(1.0, 2.0, 3.0);
    Pt3 p3(1.0, 2.0, 3.1);

    EXPECT_TRUE(p1 == p2);
    EXPECT_FALSE(p1 == p3);
    EXPECT_FALSE(p1 != p2);
    EXPECT_TRUE(p1 != p3);
}

// 测试边界情况
TEST(PointTest, EdgeCases) {
    // 测试零点
    Pt3 origin = Pt3::zeros();
    Vec3 zero_vec = Vec3::zeros();

    auto result = origin + zero_vec;
    EXPECT_TRUE(result == origin);

    auto result2 = origin - zero_vec;
    EXPECT_TRUE(result2 == origin);

    // 测试自减
    auto self_diff = origin - origin;
    EXPECT_TRUE(self_diff == zero_vec);
}

// 测试错误边界情况和异常处理
TEST(PointTest, ErrorCases) {
    // 测试空数组会触发断言（在调试模式下）
    // 在发布模式下可能不会触发断言，所以我们只测试非空情况

    // 测试单元素数组的中点
    std::vector<Pt3> single = {Pt3(1.0, 2.0, 3.0)};
    auto mid_single = Pt3::mid(single);
    EXPECT_FLOAT_EQ(mid_single.x(), 1.0);
    EXPECT_FLOAT_EQ(mid_single.y(), 2.0);
    EXPECT_FLOAT_EQ(mid_single.z(), 3.0);

    // 测试大量点的中点
    std::vector<Pt3> many_points;
    for (int i = 0; i < 100; ++i) {
        many_points.push_back(Pt3(static_cast<Float>(i), static_cast<Float>(i * 2), static_cast<Float>(i * 3)));
    }
    auto mid_many = Pt3::mid(many_points);
    EXPECT_FLOAT_EQ(mid_many.x(), 49.5);   // (0+99)/2
    EXPECT_FLOAT_EQ(mid_many.y(), 99.0);   // (0+198)/2
    EXPECT_FLOAT_EQ(mid_many.z(), 148.5);  // (0+297)/2
}
}  // namespace pbpt::math::testing