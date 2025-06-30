#include <gtest/gtest.h>
#include <stdexcept> // 用于 std::runtime_error

#include "math/geometry/ray.hpp"

// 使用命名空间以简化代码

namespace pbpt::math::testing {


// 测试 Ray 类的默认构造函数
TEST(RayTest, DefaultConstruction) {
    Ray3 r{};

    // 默认构造的 Point 和 Vec 应该都是零
    EXPECT_FLOAT_EQ(r.origin().x(), 0.0);
    EXPECT_FLOAT_EQ(r.origin().y(), 0.0);
    EXPECT_FLOAT_EQ(r.origin().z(), 0.0);

    EXPECT_FLOAT_EQ(r.direction().x(), 1.0);
    EXPECT_FLOAT_EQ(r.direction().y(), 0.0);
    EXPECT_FLOAT_EQ(r.direction().z(), 0.0);
}

// 测试从原点和方向向量构造
TEST(RayTest, OriginDirectionConstruction) {
    Pt3 origin(1.0, 2.0, 3.0);
    Vec3 direction_non_unit(5.0, 0.0, 0.0); // 一个非单位向量

    // 构造 Ray
    Ray3 r(origin, direction_non_unit);

    // 1. 验证原点是否被正确设置
    EXPECT_EQ(r.origin().x(), origin.x());
    EXPECT_EQ(r.origin().y(), origin.y());
    EXPECT_EQ(r.origin().z(), origin.z());

    // 2. 验证方向向量是否被归一化
    EXPECT_FLOAT_EQ(r.direction().x(), 1.0);
    EXPECT_FLOAT_EQ(r.direction().y(), 0.0);
    EXPECT_FLOAT_EQ(r.direction().z(), 0.0);
    
    // 3. 验证方向向量的模长确实为 1
    EXPECT_FLOAT_EQ(r.direction().length(), 1.0);
}

// 测试从原点和目标点构造
TEST(RayTest, OriginTargetConstruction) {
    Pt3 origin(1.0, 1.0, 1.0);
    Pt3 target(1.0, 5.0, 1.0);

    // 构造 Ray
    Ray3 r(origin, target);

    // 1. 验证原点
    EXPECT_EQ(r.origin().x(), origin.x());
    EXPECT_EQ(r.origin().y(), origin.y());
    EXPECT_EQ(r.origin().z(), origin.z());

    // 2. 验证方向向量是否正确计算并归一化
    // (target - origin) 是 (0, 4, 0)，归一化后是 (0, 1, 0)
    EXPECT_FLOAT_EQ(r.direction().x(), 0.0);
    EXPECT_FLOAT_EQ(r.direction().y(), 1.0);
    EXPECT_FLOAT_EQ(r.direction().z(), 0.0);

    // 3. 验证模长
    EXPECT_FLOAT_EQ(r.direction().length(), 1.0);
}

// 测试 at(t) 方法
TEST(RayTest, AtMethod) {
    Pt3 origin(10.0, 20.0, 30.0);
    Vec3 direction(0.0, 0.0, 1.0); // 使用一个单位向量以简化计算
    Ray3 r(origin, direction);

    // 1. 当 t = 0 时，应该返回原点
    Pt3 p_at_0 = r.at(0.0);
    EXPECT_EQ(p_at_0.x(), origin.x());
    EXPECT_EQ(p_at_0.y(), origin.y());
    EXPECT_EQ(p_at_0.z(), origin.z());

    // 2. 当 t > 0 时，应该在光线方向上前进
    Pt3 p_at_5 = r.at(5.0);
    EXPECT_FLOAT_EQ(p_at_5.x(), 10.0);
    EXPECT_FLOAT_EQ(p_at_5.y(), 20.0);
    EXPECT_FLOAT_EQ(p_at_5.z(), 35.0);

    // 3. 当 t < 0 时，应该在光线反方向上后退
    Pt3 p_at_neg_2 = r.at(-2.0);
    EXPECT_FLOAT_EQ(p_at_neg_2.x(), 10.0);
    EXPECT_FLOAT_EQ(p_at_neg_2.y(), 20.0);
    EXPECT_FLOAT_EQ(p_at_neg_2.z(), 28.0);
}

// 测试构造时的边缘情况
TEST(RayTest, EdgeCases) {
    Pt3 origin(1.0, 2.0, 3.0);
    Vec3 zero_direction(0.0, 0.0, 0.0);

    // 1. 使用零向量作为方向来构造 Ray
    // Vec::normalized() 会对零向量抛出异常，这里验证该行为
    EXPECT_THROW({
        Ray3 r(origin, zero_direction);
    }, std::runtime_error);

    // 2. 使用相同的原点和目标点来构造 Ray
    // (target - origin) 会得到一个零向量，同样应该抛出异常
    EXPECT_THROW({
        Ray3 r(origin, origin);
    }, std::runtime_error);
}

}
