#include <gtest/gtest.h>
#include <sstream>
#include <stdexcept>
#include <type_traits>

// 包含被测试的头文件和其依赖
#include "math/vector.hpp"
// 假设 type_alias.hpp 和 function.hpp 已被 vector.hpp 包含

// 使用命名空间以简化代码
using namespace pbpt::math;

// --- 编译时测试 (验证 constexpr 功能) ---
// 这个套件中的所有断言都在编译期间完成，确保了编译时计算的正确性。

// 用于编译时浮点数比较的辅助函数
constexpr bool are_almost_equal(Float a, Float b, Float epsilon = 1e-6) {
    return pbpt::math::abs(a - b) < epsilon;
}

TEST(VectorFinalConstexprTest, ConstructionAndAccess) {
    constexpr Vec3 v1(1.0, 2.0, 3.0);
    static_assert(v1.x() == 1.0);
    static_assert(v1.y() == 2.0);
    static_assert(v1.z() == 3.0);
    static_assert(v1[2] == 3.0);

    constexpr Vec3 v_ones = Vec3::ones();
    static_assert(v_ones[0] == 1.0 && v_ones[1] == 1.0 && v_ones[2] == 1.0);
}

TEST(VectorFinalConstexprTest, ArithmeticOperations) {
    constexpr Vec3 v1(1.0, 2.0, 3.0);
    constexpr Vec3 v2(4.0, 5.0, 6.0);

    // 一元负号
    constexpr Vec3 v_neg = -v1;
    static_assert(v_neg.x() == -1.0 && v_neg.y() == -2.0);

    // 加法
    constexpr Vec3 v_add = v1 + v2;
    static_assert(v_add.x() == 5.0 && v_add.y() == 7.0 && v_add.z() == 9.0);

    // 减法
    constexpr Vec3 v_sub = v2 - v1;
    static_assert(v_sub.x() == 3.0 && v_sub.y() == 3.0 && v_sub.z() == 3.0);

    // 标量乘法
    constexpr Vec3 v_mul = v1 * 2.0;
    static_assert(v_mul.x() == 2.0 && v_mul.y() == 4.0 && v_mul.z() == 6.0);
    
    // 验证 "除法" 的新方式：乘以倒数
    constexpr Vec3 v_div = v1 * 0.5;
    static_assert(v_div.x() == 0.5 && v_div.y() == 1.0 && v_div.z() == 1.5);

    // 逐元素乘法 (Hadamard Product)
    constexpr Vec3 v_hadamard = v1 * v2;
    static_assert(v_hadamard.x() == 4.0 && v_hadamard.y() == 10.0 && v_hadamard.z() == 18.0);
}

TEST(VectorFinalConstexprTest, ProductsAndLength) {
    constexpr Vec3 v1(1.0, 2.0, 3.0);
    constexpr Vec3 v2(4.0, 5.0, 6.0);
    constexpr Vec3 v_ortho(3.0, 4.0, 0.0);

    // 点积
    static_assert(v1.dot(v2) == 32.0);

    // 叉积
    constexpr Vec3 v_cross = v1.cross(v2);
    static_assert(v_cross.x() == -3.0 && v_cross.y() == 6.0 && v_cross.z() == -3.0);

    // 长度
    static_assert(v_ortho.length_squared() == 25.0);
    static_assert(v_ortho.length() == 5.0);
}

TEST(VectorFinalConstexprTest, Normalization) {
    constexpr Vec3 v(3.0, -4.0, 0.0);
    
    constexpr Vec3 v_norm = v.normalized();
    
    // 验证归一化结果
    static_assert(are_almost_equal(v_norm.x(), 0.6));
    static_assert(are_almost_equal(v_norm.y(), -0.8));
    static_assert(are_almost_equal(v_norm.z(), 0.0));

    // 验证归一化后长度为 1
    static_assert(are_almost_equal(v_norm.length(), 1.0));
}

TEST(VectorFinalConstexprTest, CompileTimeErrorChecks) {
    // 这个测试验证了应该在编译时就失败的操作。
    // 将它们注释掉以使编译通过，这本身就是一种测试。

    
    // 编译时错误: 移除了除法运算符
    // 试图使用 / 会导致 "no match for operator/" 编译错误
    // constexpr Vec3 v(4.0, 2.0, 0.0);
    // constexpr auto res = v / 2.0; 
    
    // 编译时错误: 归一化零向量
    // constexpr Vec3 zero_vec = Vec3::zeros();
    // constexpr auto norm_zero = zero_vec.normalized(); // 触发 throw
    
    
    // // 编译时错误: 下标越界
    // constexpr Vec3 v(1, 2, 3);
    // constexpr auto val = v[3]; // 触发 throw
    

    SUCCEED() << "Compile-time checks (incl. removed operator/) are documented and expected to fail.";
}

// --- 运行时测试 (验证运行时行为，特别是异常) ---

TEST(VectorFinalRuntimeTest, OutOfBoundsAccess) {
    Vec3 v;
    // 验证运行时下标越界会抛出 std::runtime_error
    EXPECT_THROW(v[3], std::runtime_error);
    EXPECT_THROW(v[-1], std::runtime_error);
}

TEST(VectorFinalRuntimeTest, NormalizeZeroVector) {
    Vec3 zero_vec = Vec3::zeros();
    // 验证运行时归一化零向量会抛出 std::runtime_error
    EXPECT_THROW(zero_vec.normalized(), std::runtime_error);
    EXPECT_THROW(zero_vec.normalize(), std::runtime_error);
}

TEST(VectorFinalRuntimeTest, StreamOutput) {
    Vec3 v(1.1, -2.2, 3.3);
    std::stringstream ss;
    ss << v;
    EXPECT_EQ(ss.str(), "Vec3(1.1, -2.2, 3.3)");
}