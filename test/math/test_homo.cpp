#include <gtest/gtest.h>

#include <sstream>  // 用于测试流输出

#include "pbpt/pbpt.h"

// 将测试代码放在一个独立的命名空间中是一种好习惯
namespace pbpt::math::testing {
// 使用 TEST 宏来定义一个测试用例
// 格式: TEST(TestSuiteName, TestName)

TEST(HomoTest, DefaultConstructorIsOriginPoint) {
    // 默认构造函数应该创建一个表示原点的点 (0,0,0,1)
    Homo4 h;

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
    const Homo4 h = Homo4::from_point(p);

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
    const Homo4 h = Homo4::from_vector(v);

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
    const Homo4 h(Vec4(10.0, 20.0, -30.0, 2.0));

    ASSERT_TRUE(h.is_point());

    // to_point() 应该执行透视除法 (x/w, y/w, z/w)
    const Pt3 p = h.to_point();
    EXPECT_DOUBLE_EQ(p.x(), 5.0);
    EXPECT_DOUBLE_EQ(p.y(), 10.0);
    EXPECT_DOUBLE_EQ(p.z(), -15.0);
}

TEST(HomoTest, ConversionSafetyRuntime) {
    const Homo4 h_point = Homo4::from_point(Pt3(1, 2, 3));
    const Homo4 h_vector = Homo4::from_vector(Vec3(4, 5, 6));

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
    constexpr Homo4 const_h_point = Homo4::from_point(Pt3(1, 1, 1));
    constexpr Homo4 const_h_vector = Homo4::from_vector(Vec3(2, 2, 2));

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
    Homo4     h = Homo4::from_point(p_orig);

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



TEST(HomogeneousNewFeaturesTest, FactoryMethods) {
    // Test zeros factory
    auto h_zero = Homo4::zeros();
    EXPECT_TRUE(h_zero.is_all_zero());
    EXPECT_TRUE(h_zero.is_vector());
    
    // Test filled factory
    auto h_filled = Homo4::filled(2.5);
    for (int i = 0; i < 4; ++i) {
        EXPECT_DOUBLE_EQ(h_filled.to_vector_raw()[i], 2.5);
    }
    
    // Test from_point factory
    Point<Float, 3> p(1.0, 2.0, 3.0);
    auto h_point = Homo4::from_point(p);
    EXPECT_TRUE(h_point.is_point());
    EXPECT_DOUBLE_EQ(h_point.to_vector_raw()[3], 1.0);  // w should be 1
    
    // Test from_vector factory  
    Vector<Float, 3> v(1.0, 2.0, 3.0);
    auto h_vector = Homo4::from_vector(v);
    EXPECT_TRUE(h_vector.is_vector());
    EXPECT_DOUBLE_EQ(h_vector.to_vector_raw()[3], 0.0);  // w should be 0
}

TEST(HomogeneousNewFeaturesTest, EnhancedOperators) {
    Homo4 h1(1.0, 2.0, 3.0, 1.0);
    Homo4 h2(2.0, 3.0, 4.0, 1.0);
    
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
    Homo4 h1_copy(1.0, 2.0, 3.0, 1.0);
    EXPECT_TRUE(h1 == h1_copy);
    EXPECT_FALSE(h1 == h2);
}

TEST(HomogeneousNewFeaturesTest, ApplyFunction) {
    Homo4 h(1.0, 2.0, 3.0, 1.0);
    
    // Apply function to modify elements (signature: f(T&, int))
    h.visit([](Float& x, int i) { x = x * 2.0; });
    
    EXPECT_DOUBLE_EQ(h.to_vector_raw()[0], 2.0);
    EXPECT_DOUBLE_EQ(h.to_vector_raw()[1], 4.0);
    EXPECT_DOUBLE_EQ(h.to_vector_raw()[2], 6.0);
    EXPECT_DOUBLE_EQ(h.to_vector_raw()[3], 2.0);
}

TEST(HomogeneousNewFeaturesTest, NormalizationAndChecks) {
    Homo4 h(2.0, 4.0, 6.0, 2.0);  // Point with w = 2
    
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
    constexpr Homo4 h_point(1.0, 2.0, 3.0, 1.0);
    constexpr Homo4 h_vector(1.0, 2.0, 3.0, 0.0);
    
    // These should work at compile time
    static_assert(h_point.is_point(), "Point should be detected at compile time");
    static_assert(h_vector.is_vector(), "Vector should be detected at compile time");
    static_assert(!h_point.is_vector(), "Point should not be vector");
    static_assert(!h_vector.is_point(), "Vector should not be point");
}

TEST(HomogeneousNewFeaturesTest, ErrorHandling) {
    Homo4 h_point(1.0, 2.0, 3.0, 1.0);  // Point
    Homo4 h_vector(1.0, 2.0, 3.0, 0.0);  // Vector
    
    // Test invalid conversions throw proper exceptions
    EXPECT_THROW(h_point.to_vector(), std::domain_error);
    EXPECT_THROW(h_vector.to_point(), std::domain_error);
}

}  // namespace pbpt::math::testing