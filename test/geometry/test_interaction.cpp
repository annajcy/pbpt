#include <gtest/gtest.h>

#include "pbpt.h"

using namespace pbpt::math;
using namespace pbpt::geometry;

class InteractionTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 创建测试用的点和方向
        pi = PointInterval<Float, 3>{
            Interval<Float>(Float(0.9), Float(1.1)),
            Interval<Float>(Float(1.9), Float(2.1)), 
            Interval<Float>(Float(2.9), Float(3.1))
        };
        
        wo = Vec3(0, 0, 1);
        n = Normal3(0, 0, 1);
        uv = Pt2(Float(0.5), Float(0.5));
        dpdu = Vec3(1, 0, 0);
        dpdv = Vec3(0, 1, 0);
        dndu = Normal3(0, 0, 0);
        dndv = Normal3(0, 0, 0);
    }

    PointInterval<Float, 3> pi;
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
    Point<Float, 3> offset_origin = offset_ray_origin(pi, wi, n);
    
    // 验证偏移后的原点不等于原始点
    Point<Float, 3> original_point = to_point(pi);
    EXPECT_NE(offset_origin, original_point);
    
    // 验证偏移是沿法线方向的
    Vector<Float, 3> offset_vec = offset_origin - original_point;
    Float dot_product = offset_vec.dot(n.to_vector());
    
    // 由于wi和n同向，偏移应该是正向的
    EXPECT_GT(dot_product, Float(0));
}

TEST_F(InteractionTest, OffsetRayOriginOppositeDirection) {
    Vec3 wi(0, 0, -1);  // 反向入射
    
    Point<Float, 3> offset_origin = offset_ray_origin(pi, wi, n);
    Point<Float, 3> original_point = to_point(pi);
    
    Vector<Float, 3> offset_vec = offset_origin - original_point;
    Float dot_product = offset_vec.dot(n.to_vector());
    
    // 由于wi和n反向，偏移应该是负向的
    EXPECT_LT(dot_product, Float(0));
}

TEST_F(InteractionTest, OffsetRayOriginWithDifferentNormals) {
    Vec3 wi(1, 0, 0);
    Normal3 side_normal(1, 0, 0);
    
    Point<Float, 3> offset_origin = offset_ray_origin(pi, wi, side_normal);
    Point<Float, 3> original_point = to_point(pi);
    
    // 验证有偏移
    EXPECT_NE(offset_origin, original_point);
}

// 测试 SurfaceInteraction 构造函数
TEST_F(InteractionTest, SurfaceInteractionConstruction) {
    SurfaceInteraction<Float> si(pi, wo, n, uv, dpdu, dpdv, dndu, dndv);
    
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
    Point<Float, 3> point = si.p();
    Point<Float, 3> expected = to_point(pi);
    EXPECT_EQ(point, expected);
}

// 测试 spawn_ray 方法
TEST_F(InteractionTest, SpawnRay) {
    SurfaceInteraction<Float> si(pi, wo, n, uv, dpdu, dpdv, dndu, dndv);
    
    Vec3 wi(0, 0, 1);
    Ray<Float, 3> ray = si.spawn_ray(wi);
    
    // 验证射线方向
    EXPECT_EQ(ray.direction(), wi);
    
    // 验证射线原点被偏移了
    Point<Float, 3> original_point = to_point(pi);
    EXPECT_NE(ray.origin(), original_point);
    
    // 验证射线有默认的t_max
    EXPECT_GT(ray.t_max(), Float(0));
}

// 测试 spawn_ray_to(Point) 方法
TEST_F(InteractionTest, SpawnRayToPoint) {
    SurfaceInteraction<Float> si(pi, wo, n, uv, dpdu, dpdv, dndu, dndv);
    
    Point<Float, 3> target(Float(5), Float(5), Float(5));
    Ray<Float, 3> ray = si.spawn_ray_to(target);
    
    // 验证射线方向指向目标点
    Point<Float, 3> from_point = to_point(pi);
    Vector<Float, 3> expected_dir = (target - from_point).normalized();
    Vector<Float, 3> actual_dir = ray.direction();
    
    // 由于有偏移，方向可能略有不同，但应该大致相同
    Float dot_product = actual_dir.dot(expected_dir);
    EXPECT_GT(dot_product, Float(0.9));  // 接近平行
    
    // 验证t_max被正确设置
    EXPECT_GT(ray.t_max(), Float(0));
    EXPECT_LT(ray.t_max(), std::numeric_limits<Float>::max());
}

// 测试 spawn_ray_to(SurfaceInteraction) 方法
TEST_F(InteractionTest, SpawnRayToSurfaceInteraction) {
    SurfaceInteraction<Float> si1(pi, wo, n, uv, dpdu, dpdv, dndu, dndv);
    
    // 创建第二个交点
    PointInterval<Float, 3> pi2{
        Interval<Float>(Float(4.9), Float(5.1)),
        Interval<Float>(Float(4.9), Float(5.1)),
        Interval<Float>(Float(4.9), Float(5.1))
    };
    Normal3 n2(0, 1, 0);  // 不同的法线
    SurfaceInteraction<Float> si2(pi2, wo, n2, uv, dpdu, dpdv, dndu, dndv);
    
    Ray<Float, 3> ray = si1.spawn_ray_to(si2);
    
    // 验证射线原点被偏移了
    Point<Float, 3> from_point = to_point(pi);
    EXPECT_NE(ray.origin(), from_point);
    
    // 验证t_max被正确设置
    EXPECT_GT(ray.t_max(), Float(0));
    EXPECT_LT(ray.t_max(), std::numeric_limits<Float>::max());
    
    // 验证射线长度合理
    EXPECT_GT(ray.direction().length(), Float(0));
}

// 测试边界情况：相近点之间的射线
TEST_F(InteractionTest, SpawnRayToSamePoint) {
    SurfaceInteraction<Float> si1(pi, wo, n, uv, dpdu, dpdv, dndu, dndv);
    
    // 创建一个明显不同的交点
    PointInterval<Float, 3> pi2{
        Interval<Float>(Float(1.49), Float(1.51)),  
        Interval<Float>(Float(2.49), Float(2.51)),
        Interval<Float>(Float(3.49), Float(3.51))
    };
    SurfaceInteraction<Float> si2(pi2, wo, n, uv, dpdu, dpdv, dndu, dndv);
    
    Ray<Float, 3> ray = si1.spawn_ray_to(si2);

    // 应该有有效的射线
    EXPECT_GT(ray.t_max(), Float(0));
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
        SurfaceInteraction<Float> si(pi, wo, normal, uv, dpdu, dpdv, dndu, dndv);
        Ray<Float, 3> ray = si.spawn_ray(wi);
        
        // 所有情况都应该产生有效的射线
        EXPECT_EQ(ray.direction(), wi);
        EXPECT_GT(ray.t_max(), Float(0));
        
        // 验证偏移量合理
        Point<Float, 3> original_point = to_point(pi);
        Vector<Float, 3> offset_vec = ray.origin() - original_point;
        EXPECT_GT(offset_vec.length(), Float(0));
    }
}

// 测试数值稳定性
TEST_F(InteractionTest, NumericalStability) {
    // 测试较小但现实的区间
    PointInterval<Float, 3> small_pi{
        Interval<Float>(Float(1.0) - Float(1e-4), Float(1.0) + Float(1e-4)),
        Interval<Float>(Float(2.0) - Float(1e-4), Float(2.0) + Float(1e-4)),
        Interval<Float>(Float(3.0) - Float(1e-4), Float(3.0) + Float(1e-4))
    };

    SurfaceInteraction<Float> si(small_pi, wo, n, uv, dpdu, dpdv, dndu, dndv);

    Vec3 wi(0, 0, 1);
    Ray<Float, 3> ray = si.spawn_ray(wi);
    
    // 即使区间较小，也应该产生有效的射线
    EXPECT_EQ(ray.direction(), wi);
    EXPECT_GT(ray.t_max(), Float(0));
    
    // 验证偏移量不为零（应该大于区间宽度）
    Point<Float, 3> original_point = to_point(small_pi);
    Vector<Float, 3> offset_vec = ray.origin() - original_point;
    EXPECT_GT(offset_vec.length(), Float(0));
    
    // 偏移应该足够大，至少比浮点精度大
    EXPECT_GT(offset_vec.length(), Float(1e-6));
}

// 测试基类 Interaction 的CRTP功能
TEST_F(InteractionTest, CRTPFunctionality) {
    SurfaceInteraction<Float> si(pi, wo, n, uv, dpdu, dpdv, dndu, dndv);
    
    // 测试通过基类接口调用派生类方法
    Interaction<Float, SurfaceInteraction<Float>>& base_ref = si;
    
    Vec3 wi(1, 0, 0);
    Ray<Float, 3> ray = base_ref.spawn_ray(wi);
    
    // 验证通过基类接口调用的方法工作正常
    EXPECT_EQ(ray.direction(), wi);
    EXPECT_GT(ray.t_max(), Float(0));
}

// 测试不同类型的SurfaceInteraction之间的射线生成
TEST_F(InteractionTest, CrossTypeSurfaceInteractionRays) {
    using SurfFloat = SurfaceInteraction<Float>;
    using SurfDouble = SurfaceInteraction<double>;
    
    SurfFloat si1(pi, wo, n, uv, dpdu, dpdv, dndu, dndv);
    
    // 创建一个double类型的SurfaceInteraction
    PointInterval<double, 3> pi2{
        Interval<double>(5.0, 5.2),
        Interval<double>(5.0, 5.2),
        Interval<double>(5.0, 5.2)
    };
    Vector<double, 3> wo2(0, 0, 1);
    Normal<double, 3> n2(0, 1, 0);
    Point<double, 2> uv2(0.3, 0.7);
    Vector<double, 3> dpdu2(1, 0, 0), dpdv2(0, 1, 0);
    Normal<double, 3> dndu2(0, 0, 0), dndv2(0, 0, 0);
    
    SurfDouble si2(pi2, wo2, n2, uv2, dpdu2, dpdv2, dndu2, dndv2);
    
    // 测试从Float型射向double型（这个应该通过模板推导工作）
    // 注意：这种跨类型操作可能需要显式类型转换
    Point<Float, 3> target = Point<Float, 3>(
        Float(to_point(pi2).x()),
        Float(to_point(pi2).y()),
        Float(to_point(pi2).z())
    );
    
    Ray<Float, 3> ray = si1.spawn_ray_to(target);
    EXPECT_GT(ray.t_max(), Float(0));
    EXPECT_GT(ray.direction().length(), Float(0));
}

// 测试offset_ray_origin函数的边界情况
TEST_F(InteractionTest, OffsetRayOriginBoundaryConditions) {
    // 测试法线和入射方向垂直的情况
    Vec3 wi_perpendicular(1, 0, 0);  // 垂直于Z轴法线
    Normal3 n_z(0, 0, 1);
    
    Point<Float, 3> offset_origin = offset_ray_origin(pi, wi_perpendicular, n_z);
    Point<Float, 3> original_point = to_point(pi);
    
    Vector<Float, 3> offset_vec = offset_origin - original_point;
    
    // 即使垂直，也应该有偏移（用于数值稳定性）
    EXPECT_GT(offset_vec.length(), Float(0));
    
    // 偏移应该主要在法线方向
    Float z_component = abs(offset_vec.z());
    Float xy_magnitude = sqrt(offset_vec.x() * offset_vec.x() + offset_vec.y() * offset_vec.y());
    EXPECT_GT(z_component, xy_magnitude * Float(0.1));  // Z方向分量应该占主导
}

// 测试不同区间宽度对偏移的影响
TEST_F(InteractionTest, OffsetScalesWithIntervalWidth) {
    // 创建不同宽度的区间
    PointInterval<Float, 3> narrow_pi{
        Interval<Float>(Float(1.0) - Float(0.01), Float(1.0) + Float(0.01)),
        Interval<Float>(Float(2.0) - Float(0.01), Float(2.0) + Float(0.01)),
        Interval<Float>(Float(3.0) - Float(0.01), Float(3.0) + Float(0.01))
    };
    
    PointInterval<Float, 3> wide_pi{
        Interval<Float>(Float(1.0) - Float(0.1), Float(1.0) + Float(0.1)),
        Interval<Float>(Float(2.0) - Float(0.1), Float(2.0) + Float(0.1)),
        Interval<Float>(Float(3.0) - Float(0.1), Float(3.0) + Float(0.1))
    };
    
    Vec3 wi(0, 0, 1);
    Normal3 n_test(0, 0, 1);
    
    Point<Float, 3> narrow_offset = offset_ray_origin(narrow_pi, wi, n_test);
    Point<Float, 3> wide_offset = offset_ray_origin(wide_pi, wi, n_test);
    
    Point<Float, 3> narrow_center = to_point(narrow_pi);
    Point<Float, 3> wide_center = to_point(wide_pi);
    
    Float narrow_offset_dist = (narrow_offset - narrow_center).length();
    Float wide_offset_dist = (wide_offset - wide_center).length();
    
    // 宽区间应该产生更大的偏移
    EXPECT_GT(wide_offset_dist, narrow_offset_dist);
}

// 测试spawn_ray_to方法的准确性
TEST_F(InteractionTest, SpawnRayToAccuracy) {
    SurfaceInteraction<Float> si1(pi, wo, n, uv, dpdu, dpdv, dndu, dndv);
    
    // 创建目标点
    Point<Float, 3> target(Float(10), Float(20), Float(30));
    Ray<Float, 3> ray = si1.spawn_ray_to(target);
    
    // 验证如果我们沿着射线传播足够远，应该接近目标点
    Point<Float, 3> ray_end = ray.at(ray.t_max());
    Vector<Float, 3> error_vec = ray_end - target;
    
    // 由于有偏移，不会完全精确，但应该相对较近
    EXPECT_LT(error_vec.length(), Float(1.0));  // 误差应该小于1个单位
}

// 测试两个SurfaceInteraction之间射线的对称性
TEST_F(InteractionTest, SurfaceInteractionRaySymmetry) {
    SurfaceInteraction<Float> si1(pi, wo, n, uv, dpdu, dpdv, dndu, dndv);
    
    PointInterval<Float, 3> pi2{
        Interval<Float>(Float(5), Float(5.2)),
        Interval<Float>(Float(6), Float(6.2)),
        Interval<Float>(Float(7), Float(7.2))
    };
    Normal3 n2(0, 0, -1);  // 相对的法线方向
    SurfaceInteraction<Float> si2(pi2, wo, n2, uv, dpdu, dpdv, dndu, dndv);
    
    Ray<Float, 3> ray1_to_2 = si1.spawn_ray_to(si2);
    Ray<Float, 3> ray2_to_1 = si2.spawn_ray_to(si1);
    
    // 两个射线的方向应该大致相反
    Float dot_product = ray1_to_2.direction().dot(ray2_to_1.direction());
    EXPECT_LT(dot_product, Float(-0.5));  // 应该大致相反
    
    // 两个射线的长度应该相似（但不完全相同，因为有偏移）
    Float length_ratio = ray1_to_2.t_max() / ray2_to_1.t_max();
    EXPECT_GT(length_ratio, Float(0.8));
    EXPECT_LT(length_ratio, Float(1.2));
}

// 测试极端情况：零方向向量
TEST_F(InteractionTest, ZeroDirection) {
    SurfaceInteraction<Float> si(pi, wo, n, uv, dpdu, dpdv, dndu, dndv);
    
    Vec3 zero_wi(0, 0, 0);
    
    // 零方向向量会导致Ray构造函数中的normalize失败
    // 这是一个预期的异常情况，我们应该捕获异常
    EXPECT_THROW(si.spawn_ray(zero_wi), std::exception);
}

// 测试射线到远点
TEST_F(InteractionTest, SpawnRayToDistantPoint) {
    SurfaceInteraction<Float> si(pi, wo, n, uv, dpdu, dpdv, dndu, dndv);
    
    Point<Float, 3> distant_point(Float(1000), Float(1000), Float(1000));
    Ray<Float, 3> ray = si.spawn_ray_to(distant_point);
    
    // 验证射线被正确创建
    EXPECT_GT(ray.t_max(), Float(0));
    EXPECT_GT(ray.direction().length(), Float(0));
    
    // 验证方向大致正确
    Point<Float, 3> from_point = to_point(pi);
    Vector<Float, 3> expected_dir = (distant_point - from_point).normalized();
    Vector<Float, 3> actual_dir = ray.direction();
    Float dot_product = actual_dir.dot(expected_dir);
    EXPECT_GT(dot_product, Float(0.5));  // 应该大致同向
}
