#include <gtest/gtest.h>

#include "pbpt.h"


namespace pbpt::geometry::testing{

class InteractionTest : public ::testing::Test {
protected:
    template<typename T>
    SurfaceInteraction<T> make_surface_interaction_with_shading(
        const math::Point<T, 3>& p_lower_in,
        const math::Point<T, 3>& p_upper_in,
        const math::Vector<T, 3>& wo_in,
        const math::Normal<T, 3>& n_in,
        const math::Point<T, 2>& uv_in,
        const math::Vector<T, 3>& dpdu_in,
        const math::Vector<T, 3>& dpdv_in,
        const math::Normal<T, 3>& dndu_in,
        const math::Normal<T, 3>& dndv_in
    ) const {
        return SurfaceInteraction<T>(
            p_lower_in,
            p_upper_in,
            wo_in,
            n_in,
            n_in,
            uv_in,
            dpdu_in,
            dpdv_in,
            dndu_in,
            dndv_in,
            dpdu_in,
            dpdv_in,
            dndu_in,
            dndv_in
        );
    }

    void SetUp() override {
        // 创建测试用的点和方向
        p_lower = Pt3(Float(0.9), Float(1.9), Float(2.9));
        p_upper = Pt3(Float(1.1), Float(2.1), Float(3.1));
        
        wo = Vec3(0, 0, 1);
        n = Normal3(0, 0, 1);
        uv = Pt2(Float(0.5), Float(0.5));
        dpdu = Vec3(1, 0, 0);
        dpdv = Vec3(0, 1, 0);
        dndu = Normal3(0, 0, 0);
        dndv = Normal3(0, 0, 0);
    }

    Pt3 midpoint() const {
        return p_lower.mid(p_upper);
    }

    SurfaceInteraction<Float> make_surface_interaction() const {
        return make_surface_interaction_with_shading<Float>(
            p_lower, p_upper, wo, n, uv, dpdu, dpdv, dndu, dndv
        );
    }

    Pt3 p_lower;
    Pt3 p_upper;
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
    Point<Float, 3> offset_origin = offset_ray_origin(p_lower, p_upper, wi, n);

    // 验证偏移后的原点不等于原始点
    Point<Float, 3> original_point = midpoint();
    EXPECT_NE(offset_origin, original_point);
    
    // 验证偏移是沿法线方向的
    Vector<Float, 3> offset_vec = offset_origin - original_point;
    Float dot_product = offset_vec.dot(n.to_vector());
    
    // 由于wi和n同向，偏移应该是正向的
    EXPECT_GT(dot_product, Float(0));
}

TEST_F(InteractionTest, OffsetRayOriginOppositeDirection) {
    Vec3 wi(0, 0, -1);  // 反向入射

    Point<Float, 3> offset_origin = offset_ray_origin(p_lower, p_upper, wi, n);
    Point<Float, 3> original_point = midpoint();
    
    Vector<Float, 3> offset_vec = offset_origin - original_point;
    Float dot_product = offset_vec.dot(n.to_vector());
    
    // 由于wi和n反向，偏移应该是负向的
    EXPECT_LT(dot_product, Float(0));
}

TEST_F(InteractionTest, OffsetRayOriginWithDifferentNormals) {
    Vec3 wi(1, 0, 0);
    Normal3 side_normal(1, 0, 0);
    
    Point<Float, 3> offset_origin = offset_ray_origin(p_lower, p_upper, wi, side_normal);
    Point<Float, 3> original_point = midpoint();
    
    // 验证有偏移
    EXPECT_NE(offset_origin, original_point);
}

// 测试 SurfaceInteraction 构造函数
TEST_F(InteractionTest, SurfaceInteractionConstruction) {
    SurfaceInteraction<Float> si = make_surface_interaction();

    // 验证点区间访问器
    EXPECT_EQ(si.p_lower(), p_lower);
    EXPECT_EQ(si.p_upper(), p_upper);
    EXPECT_EQ(si.point(), midpoint());

    // 验证基本访问器
    EXPECT_EQ(si.wo(), wo);
    EXPECT_EQ(si.n(), n);
    EXPECT_EQ(si.uv(), uv);
    EXPECT_EQ(si.dpdu(), dpdu);
    EXPECT_EQ(si.dpdv(), dpdv);
    EXPECT_EQ(si.dndu(), dndu);
    EXPECT_EQ(si.dndv(), dndv);
}

// 测试 spawn_ray 方法
TEST_F(InteractionTest, SpawnRay) {
    SurfaceInteraction<Float> si = make_surface_interaction();

    Vec3 wi(0, 0, 1);
    Ray<Float, 3> ray = si.spawn_ray(wi);
    
    // 验证射线方向
    EXPECT_EQ(ray.direction(), wi);
    
    // 验证射线原点被偏移了
    Point<Float, 3> original_point = midpoint();
    EXPECT_NE(ray.origin(), original_point);
    
    // 验证射线有默认的t_max
    EXPECT_GT(ray.t_max(), Float(0));
}

// 测试 spawn_ray_to(Point) 方法
TEST_F(InteractionTest, SpawnRayToPoint) {
    SurfaceInteraction<Float> si = make_surface_interaction();

    Point<Float, 3> target(Float(5), Float(5), Float(5));
    Ray<Float, 3> ray = si.spawn_ray_to(target);
    
    // 验证射线方向指向目标点
    Point<Float, 3> from_point = midpoint();
    Vector<Float, 3> expected_dir = (target - from_point).normalized();
    Vector<Float, 3> actual_dir = ray.direction();
    
    // 由于有偏移，方向可能略有不同，但应该大致相同
    Float dot_product = actual_dir.dot(expected_dir);
    EXPECT_GT(dot_product, Float(0.9));  // 接近平行
    
    // 验证t_max被正确设置
    EXPECT_GT(ray.t_max(), Float(0));
    EXPECT_LT(ray.t_max(), std::numeric_limits<Float>::max());
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
        SurfaceInteraction<Float> si = make_surface_interaction_with_shading<Float>(
            p_lower, p_upper, wo, normal, uv, dpdu, dpdv, dndu, dndv
        );
        Ray<Float, 3> ray = si.spawn_ray(wi);
        
        // 所有情况都应该产生有效的射线
        EXPECT_EQ(ray.direction(), wi);
        EXPECT_GT(ray.t_max(), Float(0));
        
        // 验证偏移量合理
        Point<Float, 3> original_point = midpoint();
        Vector<Float, 3> offset_vec = ray.origin() - original_point;
        EXPECT_GT(offset_vec.length(), Float(0));
    }
}

// 测试数值稳定性
TEST_F(InteractionTest, NumericalStability) {
    // 测试较小但现实的区间
    Pt3 small_lower(
        Float(1.0) - Float(1e-4),
        Float(2.0) - Float(1e-4),
        Float(3.0) - Float(1e-4)
    );
    Pt3 small_upper(
        Float(1.0) + Float(1e-4),
        Float(2.0) + Float(1e-4),
        Float(3.0) + Float(1e-4)
    );

    SurfaceInteraction<Float> si = make_surface_interaction_with_shading<Float>(
        small_lower, small_upper, wo, n, uv, dpdu, dpdv, dndu, dndv
    );

    Vec3 wi(0, 0, 1);
    Ray<Float, 3> ray = si.spawn_ray(wi);
    
    // 即使区间较小，也应该产生有效的射线
    EXPECT_EQ(ray.direction(), wi);
    EXPECT_GT(ray.t_max(), Float(0));
    
    // 验证偏移量不为零（应该大于区间宽度）
    Point<Float, 3> original_point = small_lower.mid(small_upper);
    Vector<Float, 3> offset_vec = ray.origin() - original_point;
    EXPECT_GT(offset_vec.length(), Float(0));
    
    // 偏移应该足够大，至少比浮点精度大
    EXPECT_GT(offset_vec.length(), Float(1e-6));
}

// 测试基类 Interaction 的CRTP功能
TEST_F(InteractionTest, CRTPFunctionality) {
    SurfaceInteraction<Float> si = make_surface_interaction();
    
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
    
    SurfFloat si1 = make_surface_interaction();
    
    // 创建一个double类型的SurfaceInteraction
    math::Point<double, 3> p2_lower(5.0, 5.0, 5.0);
    math::Point<double, 3> p2_upper(5.2, 5.2, 5.2);
    Vector<double, 3> wo2(0, 0, 1);
    Normal<double, 3> n2(0, 1, 0);
    Point<double, 2> uv2(0.3, 0.7);
    Vector<double, 3> dpdu2(1, 0, 0), dpdv2(0, 1, 0);
    Normal<double, 3> dndu2(0, 0, 0), dndv2(0, 0, 0);
    
    SurfDouble si2 = make_surface_interaction_with_shading<double>(
        p2_lower, p2_upper, wo2, n2, uv2, dpdu2, dpdv2, dndu2, dndv2
    );
    EXPECT_EQ(si2.p_lower(), p2_lower);
    EXPECT_EQ(si2.p_upper(), p2_upper);
    
    // 测试从Float型射向double型（这个应该通过模板推导工作）
    // 注意：这种跨类型操作可能需要显式类型转换
    auto target_double = p2_lower.mid(p2_upper);
    Point<Float, 3> target = Point<Float, 3>(
        Float(target_double.x()),
        Float(target_double.y()),
        Float(target_double.z())
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
    
    Point<Float, 3> offset_origin = offset_ray_origin(p_lower, p_upper, wi_perpendicular, n_z);
    Point<Float, 3> original_point = midpoint();
    
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
    Pt3 narrow_lower(Float(1.0) - Float(0.01), Float(2.0) - Float(0.01), Float(3.0) - Float(0.01));
    Pt3 narrow_upper(Float(1.0) + Float(0.01), Float(2.0) + Float(0.01), Float(3.0) + Float(0.01));
    
    Pt3 wide_lower(Float(1.0) - Float(0.1), Float(2.0) - Float(0.1), Float(3.0) - Float(0.1));
    Pt3 wide_upper(Float(1.0) + Float(0.1), Float(2.0) + Float(0.1), Float(3.0) + Float(0.1));
    
    Vec3 wi(0, 0, 1);
    Normal3 n_test(0, 0, 1);
    
    Point<Float, 3> narrow_offset = offset_ray_origin(narrow_lower, narrow_upper, wi, n_test);
    Point<Float, 3> wide_offset = offset_ray_origin(wide_lower, wide_upper, wi, n_test);
    
    Point<Float, 3> narrow_center = narrow_lower.mid(narrow_upper);
    Point<Float, 3> wide_center = wide_lower.mid(wide_upper);
    
    Float narrow_offset_dist = (narrow_offset - narrow_center).length();
    Float wide_offset_dist = (wide_offset - wide_center).length();
    
    // 宽区间应该产生更大的偏移
    EXPECT_GT(wide_offset_dist, narrow_offset_dist);
}

// 测试spawn_ray_to方法的准确性
TEST_F(InteractionTest, SpawnRayToAccuracy) {
    SurfaceInteraction<Float> si1 = make_surface_interaction();
    
    // 创建目标点
    Point<Float, 3> target(Float(10), Float(20), Float(30));
    Ray<Float, 3> ray = si1.spawn_ray_to(target);
    
    // 验证如果我们沿着射线传播足够远，应该接近目标点
    Point<Float, 3> ray_end = ray.at(ray.t_max());
    Vector<Float, 3> error_vec = ray_end - target;
    
    // 由于有偏移，不会完全精确，但应该相对较近
    EXPECT_LT(error_vec.length(), Float(1.0));  // 误差应该小于1个单位
}

// 测试极端情况：零方向向量
TEST_F(InteractionTest, ZeroDirection) {
    SurfaceInteraction<Float> si = make_surface_interaction();
    
    Vec3 zero_wi(0, 0, 0);
    
    // 零方向向量会导致Ray构造函数中的normalize失败
    // 这是一个预期的异常情况，我们应该捕获异常
    EXPECT_THROW(si.spawn_ray(zero_wi), std::exception);
}

// 测试射线到远点
TEST_F(InteractionTest, SpawnRayToDistantPoint) {
    SurfaceInteraction<Float> si = make_surface_interaction();
    
    Point<Float, 3> distant_point(Float(1000), Float(1000), Float(1000));
    Ray<Float, 3> ray = si.spawn_ray_to(distant_point);
    
    // 验证射线被正确创建
    EXPECT_GT(ray.t_max(), Float(0));
    EXPECT_GT(ray.direction().length(), Float(0));
    
    // 验证方向大致正确
    Point<Float, 3> from_point = midpoint();
    Vector<Float, 3> expected_dir = (distant_point - from_point).normalized();
    Vector<Float, 3> actual_dir = ray.direction();
    Float dot_product = actual_dir.dot(expected_dir);
    EXPECT_GT(dot_product, Float(0.5));  // 应该大致同向
}

}
