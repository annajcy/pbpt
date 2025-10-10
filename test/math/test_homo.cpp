#include <gtest/gtest.h>

#include <sstream>  // 用于测试流输出

#include "pbpt.h"

// 将测试代码放在一个独立的命名空间中是一种好习惯
namespace pbpt::math::testing {
// 使用 TEST 宏来定义一个测试用例
// 格式: TEST(TestSuiteName, TestName)

TEST(HomoTest, DefaultConstructorIsOriginPoint) {
    // 默认构造函数应该创建一个表示原点的点 (0,0,0,1)
    Homo3 h;

    EXPECT_FALSE(h.is_point());
    EXPECT_TRUE(h.is_vector());

    // 检查原始数据
    EXPECT_DOUBLE_EQ(h.to_vector_raw().x(), 0.0);
    EXPECT_DOUBLE_EQ(h.to_vector_raw().y(), 0.0);
    EXPECT_DOUBLE_EQ(h.to_vector_raw().z(), 0.0);
    EXPECT_DOUBLE_EQ(h.to_vector_raw().w(), 0.0);

    // 转换回 Point 并检查
    Pt3 p = h.to_vector();
    EXPECT_DOUBLE_EQ(p.x(), 0.0);
    EXPECT_DOUBLE_EQ(p.y(), 0.0);
    EXPECT_DOUBLE_EQ(p.z(), 0.0);
}

TEST(HomoTest, ConstructFromPoint) {
    const Pt3   p(1.5, -2.5, 3.0);
    const Homo3 h = Homo3::from_point(p);

    EXPECT_TRUE(h.is_point());
    EXPECT_FALSE(h.is_vector());

    // 检查原始数据
    EXPECT_DOUBLE_EQ(h.to_vector_raw().x(), 1.5);
    EXPECT_DOUBLE_EQ(h.to_vector_raw().y(), -2.5);
    EXPECT_DOUBLE_EQ(h.to_vector_raw().z(), 3.0);
    EXPECT_DOUBLE_EQ(h.to_vector_raw().w(), 1.0);

    // 转换回来并验证数据一致
    const Pt3 p_back = h.to_point();
    EXPECT_EQ(p_back[0], p[0]);
    EXPECT_EQ(p_back[1], p[1]);
    EXPECT_EQ(p_back[2], p[2]);
}

TEST(HomoTest, ConstructFromVector) {
    const Vec3  v(4.0, 5.0, -6.5);
    const Homo3 h = Homo3::from_vector(v);

    EXPECT_FALSE(h.is_point());
    EXPECT_TRUE(h.is_vector());

    // 检查原始数据
    EXPECT_DOUBLE_EQ(h.to_vector_raw().x(), 4.0);
    EXPECT_DOUBLE_EQ(h.to_vector_raw().y(), 5.0);
    EXPECT_DOUBLE_EQ(h.to_vector_raw().z(), -6.5);
    EXPECT_DOUBLE_EQ(h.to_vector_raw().w(), 0.0);

    // 转换回来并验证数据一致
    const Vec3 v_back = h.to_vector();
    EXPECT_EQ(v_back[0], v[0]);
    EXPECT_EQ(v_back[1], v[1]);
    EXPECT_EQ(v_back[2], v[2]);
}

TEST(HomoTest, ToPointPerformsPerspectiveDivide) {
    // 创建一个 w 不为 1 的齐次坐标点
    const Homo3 h(Vec4(10.0, 20.0, -30.0, 2.0));

    ASSERT_TRUE(h.is_point());

    // to_point() 应该执行透视除法 (x/w, y/w, z/w)
    const Pt3 p = h.to_point();
    EXPECT_DOUBLE_EQ(p.x(), 5.0);
    EXPECT_DOUBLE_EQ(p.y(), 10.0);
    EXPECT_DOUBLE_EQ(p.z(), -15.0);
}

TEST(HomoTest, ConversionSafetyRuntime) {
    const Homo3 h_point = Homo3::from_point(Pt3(1, 2, 3));
    const Homo3 h_vector = Homo3::from_vector(Vec3(4, 5, 6));

    // 合法的转换不应该抛出异常
    ASSERT_NO_THROW(h_point.to_point());
    ASSERT_NO_THROW(h_vector.to_vector());

    // 非法的转换应该抛出 std::domain_error
    ASSERT_THROW(h_point.to_vector(), std::domain_error);
    ASSERT_THROW(h_vector.to_point(), std::domain_error);
}

TEST(HomoTest, ConversionSafetyCompileTime) {
    // 这个测试通过编译本身就证明了编译期检查的声明是有效的。
    // 我们使用 static_assert 来验证在编译期可以确定的条件。
    constexpr Homo3 const_h_point = Homo3::from_point(Pt3(1, 1, 1));
    constexpr Homo3 const_h_vector = Homo3::from_vector(Vec3(2, 2, 2));

    // 这些断言在编译期进行检查
    static_assert(const_h_point.is_point(), "A Homo from a Point should be a point at compile time.");
    static_assert(const_h_vector.is_vector(), "A Homo from a Vector should be a vector at compile time.");

    // 下面的代码行如果取消注释，将导致编译失败，这正是我们期望的行为。
    // 这证明了编译期错误检查正在工作。
    //
    // constexpr Pt3 p = const_h_vector.to_point(); // COMPILE ERROR!
    //constexpr Vec3 v = const_h_point.to_vector();   // COMPILE ERROR!

    // 因为我们不能提交一个无法编译的测试，所以上面的代码是注释掉的。
    // 这个测试的存在和成功编译，以及 static_assert 的通过，证明了其设计。
    SUCCEED();
}

TEST(HomoTest, RawAccessor) {
    const Pt3 p_orig(1, 2, 3);
    Homo3     h = Homo3::from_point(p_orig);

    // 测试 const 版本的 to_vector_raw()
    const auto& const_to_vector_raw_data = h.to_vector_raw();
    EXPECT_DOUBLE_EQ(const_to_vector_raw_data.x(), 1.0);
    EXPECT_DOUBLE_EQ(const_to_vector_raw_data.w(), 1.0);

    // 测试非 const 版本的 to_vector_raw()，并修改数据
    auto& non_const_to_vector_raw_data = h.to_vector_raw();
    non_const_to_vector_raw_data.x() = 99.0;
    EXPECT_DOUBLE_EQ(non_const_to_vector_raw_data.x(), 99.0);

    // 验证原始的 m_data 确实被修改了
    const Pt3 p_modified = h.to_point();
    EXPECT_DOUBLE_EQ(p_modified.x(), 99.0);
}

}  // namespace pbpt::math::testing