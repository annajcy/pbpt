#include <gtest/gtest.h>

#include "math/normal.hpp"
#include "math/point.hpp"
#include "geometry/ray.hpp"
#include "core/interaction.hpp"

using namespace pbpt::core;
using namespace pbpt::math;
using namespace pbpt::geometry;

class InteractionTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 创建测试用的点和方向
        pi = Pt3Interv{
            Interval<Float>(0.9f, 1.1f),
            Interval<Float>(1.9f, 2.1f), 
            Interval<Float>(2.9f, 3.1f)
        };
        
        wo = Vec3(0, 0, 1);
        n = Normal3(0, 0, 1);
        uv = Pt2(0.5f, 0.5f);
        dpdu = Vec3(1, 0, 0);
        dpdv = Vec3(0, 1, 0);
        dndu = Normal3(0, 0, 0);
        dndv = Normal3(0, 0, 0);
    }

    Pt3Interv pi;
    Vec3 wo;
    Normal3 n;
    Pt2 uv;
    Vec3 dpdu, dpdv;
    Normal3 dndu, dndv;
};

// 测试 offset_ray_origin 函数
TEST_F(InteractionTest, OffsetRayOrigin) {
    Vec3 wi(0, 0, 1);  // 入射方向
    
    // 测试正常情况
    
    Pt3 offset_origin = offset_ray_origin(pi, wi, n);
    
    // 验证偏移后的原点不等于原始点
    Pt3 original_point = to_point(pi);
    EXPECT_NE(offset_origin, original_point);
    
    // 验证偏移是沿法线方向的
    Vec3 offset_vec = offset_origin - original_point;
    Float dot_product = offset_vec.dot(n.to_vector());
    
    // 由于wi和n同向，偏移应该是正向的
    EXPECT_GT(dot_product, 0.0f);
}

TEST_F(InteractionTest, OffsetRayOriginOppositeDirection) {
    Vec3 wi(0, 0, -1);  // 反向入射
    
    Pt3 offset_origin = offset_ray_origin(pi, wi, n);
    Pt3 original_point = to_point(pi);
    
    Vec3 offset_vec = offset_origin - original_point;
    Float dot_product = offset_vec.dot(n.to_vector());
    
    // 由于wi和n反向，偏移应该是负向的
    EXPECT_LT(dot_product, 0.0f);
}

TEST_F(InteractionTest, OffsetRayOriginWithDifferentNormals) {
    Vec3 wi(1, 0, 0);
    Normal3 side_normal(1, 0, 0);
    
    Pt3 offset_origin = offset_ray_origin(pi, wi, side_normal);
    Pt3 original_point = to_point(pi);
    
    // 验证有偏移
    EXPECT_NE(offset_origin, original_point);
}

// 测试 SurfaceInteraction 构造函数
TEST_F(InteractionTest, SurfaceInteractionConstruction) {
    SurfaceInteraction si(pi, wo, n, uv, dpdu, dpdv, dndu, dndv);
    
    // 验证基本访问器
    EXPECT_EQ(si.pi(), pi);
    EXPECT_EQ(si.wo(), wo);
    EXPECT_EQ(si.n(), n);
    EXPECT_EQ(si.uv(), uv);
    EXPECT_EQ(si.dpdu(), dpdu);
    EXPECT_EQ(si.dpdv(), dpdv);
    EXPECT_EQ(si.dndu(), dndu);
    EXPECT_EQ(si.dndv(), dndv);
    
    // 验证 to_point 转换
    Pt3 point = si.p();
    Pt3 expected = to_point(pi);
    EXPECT_EQ(point, expected);
}

// 测试 spawn_ray 方法
TEST_F(InteractionTest, SpawnRay) {
    SurfaceInteraction si(pi, wo, n, uv, dpdu, dpdv, dndu, dndv);
    
    Vec3 wi(0, 0, 1);
    Ray3 ray = si.spawn_ray(wi);
    
    // 验证射线方向
    EXPECT_EQ(ray.direction(), wi);
    
    // 验证射线原点被偏移了
    Pt3 original_point = to_point(pi);
    EXPECT_NE(ray.origin(), original_point);
    
    // 验证射线有默认的t_max
    EXPECT_GT(ray.t_max(), 0.0);
}

// 测试 spawn_ray_to(Point) 方法
TEST_F(InteractionTest, SpawnRayToPoint) {
    SurfaceInteraction si(pi, wo, n, uv, dpdu, dpdv, dndu, dndv);
    
    Pt3 target(5.0f, 5.0f, 5.0f);
    Ray3 ray = si.spawn_ray_to(target);
    
    // 验证射线方向指向目标点
    Pt3 from_point = to_point(pi);
    Vec3 expected_dir = (target - from_point).normalized();
    Vec3 actual_dir = ray.direction();
    
    // 由于有偏移，方向可能略有不同，但应该大致相同
    Float dot_product = actual_dir.dot(expected_dir);
    EXPECT_GT(dot_product, 0.9f);  // 接近平行
    
    // 验证t_max被正确设置
    EXPECT_GT(ray.t_max(), 0.0);
    EXPECT_LT(ray.t_max(), std::numeric_limits<Float>::max());
}

// 测试 spawn_ray_to(SurfaceInteraction) 方法
TEST_F(InteractionTest, SpawnRayToSurfaceInteraction) {
    SurfaceInteraction si1(pi, wo, n, uv, dpdu, dpdv, dndu, dndv);
    
    // 创建第二个交点
    Pt3Interv pi2{
        Interval<Float>(4.9f, 5.1f),
        Interval<Float>(4.9f, 5.1f),
        Interval<Float>(4.9f, 5.1f)
    };
    Normal3 n2(0, 1, 0);  // 不同的法线
    SurfaceInteraction si2(pi2, wo, n2, uv, dpdu, dpdv, dndu, dndv);
    
    Ray3 ray = si1.spawn_ray_to(si2);
    
    // 验证射线原点被偏移了
    Pt3 from_point = to_point(pi);
    EXPECT_NE(ray.origin(), from_point);
    
    // 验证t_max被正确设置
    EXPECT_GT(ray.t_max(), 0.0);
    EXPECT_LT(ray.t_max(), std::numeric_limits<Float>::max());
    
    // 验证射线长度合理
    EXPECT_GT(ray.direction().length(), 0.0f);
}

// 测试边界情况：相近点之间的射线
TEST_F(InteractionTest, SpawnRayToSamePoint) {
    SurfaceInteraction si1(pi, wo, n, uv, dpdu, dpdv, dndu, dndv);
    
    // 创建一个明显不同的交点
    Pt3Interv pi2{
        Interval<Float>(1.49f, 1.51f),  
        Interval<Float>(2.49f, 2.51f),
        Interval<Float>(3.49f, 3.51f)
    };
    SurfaceInteraction si2(pi2, wo, n, uv, dpdu, dpdv, dndu, dndv);
    
    Ray3 ray = si1.spawn_ray_to(si2);

    std::cout << "Ray Origin: " << ray.origin() << ", Direction: " << ray.direction() << std::endl;
    
    // 应该有有效的射线
    EXPECT_GT(ray.t_max(), 0.0);
}

// 测试不同法线方向的影响
TEST_F(InteractionTest, DifferentNormalDirections) {
    // 测试不同的法线方向
    std::vector<Normal3> normals = {
        Normal3(1, 0, 0),   // X轴正方向
        Normal3(-1, 0, 0),  // X轴负方向
        Normal3(0, 1, 0),   // Y轴正方向
        Normal3(0, -1, 0),  // Y轴负方向
        Normal3(0, 0, 1),   // Z轴正方向
        Normal3(0, 0, -1)   // Z轴负方向
    };
    
    Vec3 wi(0, 0, 1);  // 固定入射方向
    
    for (const auto& normal : normals) {
        SurfaceInteraction si(pi, wo, normal, uv, dpdu, dpdv, dndu, dndv);
        Ray3 ray = si.spawn_ray(wi);
        
        // 所有情况都应该产生有效的射线
        EXPECT_EQ(ray.direction(), wi);
        EXPECT_GT(ray.t_max(), 0.0);
        
        // 验证偏移量合理
        Pt3 original_point = to_point(pi);
        Vec3 offset_vec = ray.origin() - original_point;
        EXPECT_GT(offset_vec.length(), 0.0f);
    }
}

// 测试数值稳定性
TEST_F(InteractionTest, NumericalStability) {
    // 测试非常小的区间
    Pt3Interv small_pi{
        Interval<Float>(1.0f - 1e-6f, 1.0f + 1e-6f),
        Interval<Float>(2.0f - 1e-6f, 2.0f + 1e-6f),
        Interval<Float>(3.0f - 1e-6f, 3.0f + 1e-6f)
    };
    
    SurfaceInteraction si(small_pi, wo, n, uv, dpdu, dpdv, dndu, dndv);
    
    Vec3 wi(0, 0, 1);
    Ray3 ray = si.spawn_ray(wi);
    
    // 即使区间很小，也应该产生有效的射线
    EXPECT_EQ(ray.direction(), wi);
    EXPECT_GT(ray.t_max(), 0.0);
    
    // 验证偏移量不为零（即使很小）
    Pt3 original_point = to_point(small_pi);
    EXPECT_NE(ray.origin(), original_point);
}

// 测试极端情况：零方向向量
TEST_F(InteractionTest, ZeroDirection) {
    SurfaceInteraction si(pi, wo, n, uv, dpdu, dpdv, dndu, dndv);
    
    Vec3 zero_wi(0, 0, 0);
    
    // 零方向向量会导致Ray构造函数中的normalize失败
    // 这是一个预期的异常情况，我们应该捕获异常
    EXPECT_THROW(si.spawn_ray(zero_wi), std::exception);
}

// 测试射线到远点
TEST_F(InteractionTest, SpawnRayToDistantPoint) {
    SurfaceInteraction si(pi, wo, n, uv, dpdu, dpdv, dndu, dndv);
    
    Pt3 distant_point(1000.0f, 1000.0f, 1000.0f);
    Ray3 ray = si.spawn_ray_to(distant_point);
    
    // 验证射线被正确创建
    EXPECT_GT(ray.t_max(), 0.0);
    EXPECT_GT(ray.direction().length(), 0.0f);
    
    // 验证方向大致正确
    Pt3 from_point = to_point(pi);
    Vec3 expected_dir = (distant_point - from_point).normalized();
    Vec3 actual_dir = ray.direction();
    Float dot_product = actual_dir.dot(expected_dir);
    EXPECT_GT(dot_product, 0.5f);  // 应该大致同向
}
