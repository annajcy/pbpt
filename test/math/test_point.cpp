#include <gtest/gtest.h>
#include <sstream> 
#include <type_traits> 

#include "test_type_alias.hpp"
#include "point.hpp"

using namespace pbpt::math;

// 测试 Point 类的基本构造和访问器
TEST(PointTest, ConstructionAndAccessors) {
    // 1. 默认构造函数
    Point3 p_default;
    EXPECT_FLOAT_EQ(p_default.x(), 0.0);
    EXPECT_FLOAT_EQ(p_default.y(), 0.0);
    EXPECT_FLOAT_EQ(p_default.z(), 0.0);

    // 2. 单值构造函数
    Point3 p_uniform(1.2);
    EXPECT_FLOAT_EQ(p_uniform.x(), 1.2);
    EXPECT_FLOAT_EQ(p_uniform.y(), 1.2);
    EXPECT_FLOAT_EQ(p_uniform.z(), 1.2);

    // 3. 多参数构造函数
    Point3 p_multi(1.0, 2.0, 3.0);
    EXPECT_FLOAT_EQ(p_multi.x(), 1.0);
    EXPECT_FLOAT_EQ(p_multi.y(), 2.0);
    EXPECT_FLOAT_EQ(p_multi.z(), 3.0);

    // 4. 从 Vec 构造
    Vec3 v_source(4.0, 5.0, 6.0);
    Point3 p_from_vec(v_source);
    EXPECT_FLOAT_EQ(p_from_vec.x(), 4.0);
    EXPECT_FLOAT_EQ(p_from_vec.y(), 5.0);
    EXPECT_FLOAT_EQ(p_from_vec.z(), 6.0);

    // 5. 下标访问器 (const)
    const Point3 p_const = p_multi;
    EXPECT_FLOAT_EQ(p_const[0], 1.0);
    EXPECT_FLOAT_EQ(p_const[1], 2.0);
    EXPECT_FLOAT_EQ(p_const[2], 3.0);
    
    // 6. 下标访问器 (non-const)
    Point3 p_mutable(0, 0, 0);
    p_mutable[0] = 7.0;
    p_mutable[1] = 8.0;
    p_mutable[2] = 9.0;
    EXPECT_FLOAT_EQ(p_mutable.x(), 7.0);
    EXPECT_FLOAT_EQ(p_mutable.y(), 8.0);
    EXPECT_FLOAT_EQ(p_mutable.z(), 9.0);
}

// 测试核心的代数运算规则
TEST(PointTest, AlgebraicOperations) {
    Point3 p1(1.0, 2.0, 3.0);
    Point3 p2(5.0, 7.0, 9.0);
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
    static_assert(std::is_same_v<decltype(p_new), Point3>, "Point + Vector should yield a Point");
    EXPECT_FLOAT_EQ(p_new.x(), p2.x());
    EXPECT_FLOAT_EQ(p_new.y(), p2.y());
    EXPECT_FLOAT_EQ(p_new.z(), p2.z());
    
    // 规则 2 (交换律): Vector + Point = Point
    auto p_new_commutative = v + p1;
    static_assert(std::is_same_v<decltype(p_new_commutative), Point3>, "Vector + Point should yield a Point");
    EXPECT_FLOAT_EQ(p_new_commutative.x(), p2.x());
    EXPECT_FLOAT_EQ(p_new_commutative.y(), p2.y());
    EXPECT_FLOAT_EQ(p_new_commutative.z(), p2.z());

    // 规则 3: Point - Vector = Point
    auto p_subtracted = p2 - v;
    static_assert(std::is_same_v<decltype(p_subtracted), Point3>, "Point - Vector should yield a Point");
    EXPECT_FLOAT_EQ(p_subtracted.x(), p1.x());
    EXPECT_FLOAT_EQ(p_subtracted.y(), p1.y());
    EXPECT_FLOAT_EQ(p_subtracted.z(), p1.z());
}

// 测试复合赋值运算符
TEST(PointTest, CompoundAssignment) {
    Point3 p(1.0, 2.0, 3.0);
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
    Point3 p1(0.0, 0.0, 0.0);
    Point3 p2(10.0, -20.0, 30.0);
    
    Point3 mid = mid_point(p1, p2);
    
    EXPECT_FLOAT_EQ(mid.x(), 5.0);
    EXPECT_FLOAT_EQ(mid.y(), -10.0);
    EXPECT_FLOAT_EQ(mid.z(), 15.0);
}

// 测试显式类型转换
TEST(PointTest, ExplicitConversion) {
    Point3 p(1, 2, 3);

    // 显式转换
    Vec3 v = static_cast<Vec3>(p);
    EXPECT_FLOAT_EQ(v.x(), 1.0);
    EXPECT_FLOAT_EQ(v.y(), 2.0);
    EXPECT_FLOAT_EQ(v.z(), 3.0);

    // 注意：隐式转换应该失败，因为从 Vec 构造 Point 的构造函数是 explicit 的。
    // Vec3 implicit_v = p; // 这行代码如果取消注释，应该会导致编译失败
}

// 测试流输出运算符
TEST(PointTest, StreamOutput) {
    Point3 p(1.1, -2.2, 3.3);
    std::stringstream ss;
    ss << p;

    // Point 的流输出利用了 Vec 的流输出
    // 假设 Vec 的输出格式是 "(x, y, z)"
    EXPECT_EQ(ss.str(), "(1.1, -2.2, 3.3)");
}


// --- 编译时测试（文档性质） ---
// 下面的代码块通过注释的形式展示了哪些操作会因为我们的设计而编译失败。
// 这是对代数正确性设计的一种“文档化测试”。
TEST(PointTest, CompileTimeAlgebraicChecks) {
    Point3 p1(1, 2, 3);
    Point3 p2(4, 5, 6);
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

    SUCCEED() << "Compile-time checks are documented and expected to fail if uncommented.";
}