#include <gtest/gtest.h>
#include <stdexcept>
#include <array>
#include <cmath>

#include "pbpt.h"

using namespace pbpt::math;

namespace pbpt::geometry::testing {

// 测试 Ray 类的基础功能
class RayTest : public ::testing::Test {
protected:
    void SetUp() override {}
    void TearDown() override {}
};

// 测试 Ray 类的默认构造函数
TEST_F(RayTest, DefaultConstruction) {
    Ray3 r{};

    // 默认构造的原点应该是零点
    EXPECT_DOUBLE_EQ(r.origin().x(), 0.0);
    EXPECT_DOUBLE_EQ(r.origin().y(), 0.0);
    EXPECT_DOUBLE_EQ(r.origin().z(), 0.0);

    // 默认方向应该是 (1, 0, 0)
    EXPECT_DOUBLE_EQ(r.direction().x(), 1.0);
    EXPECT_DOUBLE_EQ(r.direction().y(), 0.0);
    EXPECT_DOUBLE_EQ(r.direction().z(), 0.0);
    
    // 方向向量应该是单位向量
    EXPECT_NEAR(r.direction().length(), 1.0, 1e-10);
}

// 测试从原点和方向向量构造
TEST_F(RayTest, OriginDirectionConstruction) {
    Pt3 origin(1.0, 2.0, 3.0);
    Vec3 direction_non_unit(5.0, 0.0, 0.0);  // 非单位向量

    Ray3 r(origin, direction_non_unit);

    // 验证原点是否正确设置
    EXPECT_DOUBLE_EQ(r.origin().x(), origin.x());
    EXPECT_DOUBLE_EQ(r.origin().y(), origin.y());
    EXPECT_DOUBLE_EQ(r.origin().z(), origin.z());

    // 验证方向向量是否被归一化
    EXPECT_DOUBLE_EQ(r.direction().x(), 1.0);
    EXPECT_DOUBLE_EQ(r.direction().y(), 0.0);
    EXPECT_DOUBLE_EQ(r.direction().z(), 0.0);

    // 验证方向向量的模长为 1
    EXPECT_NEAR(r.direction().length(), 1.0, 1e-10);
}

// 测试从原点和目标点构造
TEST_F(RayTest, OriginTargetConstruction) {
    Pt3 origin(1.0, 1.0, 1.0);
    Pt3 target(1.0, 5.0, 1.0);

    Ray3 r(origin, target);

    // 验证原点
    EXPECT_DOUBLE_EQ(r.origin().x(), origin.x());
    EXPECT_DOUBLE_EQ(r.origin().y(), origin.y());
    EXPECT_DOUBLE_EQ(r.origin().z(), origin.z());

    // 验证方向向量：(target - origin) = (0, 4, 0)，归一化后是 (0, 1, 0)
    EXPECT_DOUBLE_EQ(r.direction().x(), 0.0);
    EXPECT_DOUBLE_EQ(r.direction().y(), 1.0);
    EXPECT_DOUBLE_EQ(r.direction().z(), 0.0);

    // 验证模长
    EXPECT_NEAR(r.direction().length(), 1.0, 1e-10);
}

// 测试 at(t) 方法
TEST_F(RayTest, AtMethod) {
    Pt3 origin(10.0, 20.0, 30.0);
    Vec3 direction(0.0, 0.0, 1.0);  // 单位向量
    Ray3 r(origin, direction, std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity());

    // t = 0 时，应该返回原点
    auto p_at_0 = r.at(0.0);
    EXPECT_DOUBLE_EQ(p_at_0.x(), origin.x());
    EXPECT_DOUBLE_EQ(p_at_0.y(), origin.y());
    EXPECT_DOUBLE_EQ(p_at_0.z(), origin.z());

    // t > 0 时，沿方向前进
    auto p_at_5 = r.at(5.0);
    EXPECT_DOUBLE_EQ(p_at_5.x(), 10.0);
    EXPECT_DOUBLE_EQ(p_at_5.y(), 20.0);
    EXPECT_DOUBLE_EQ(p_at_5.z(), 35.0);

    // t < 0 时，沿反方向后退
    auto p_at_neg_2 = r.at(-2.0);
    EXPECT_DOUBLE_EQ(p_at_neg_2.x(), 10.0);
    EXPECT_DOUBLE_EQ(p_at_neg_2.y(), 20.0);
    EXPECT_DOUBLE_EQ(p_at_neg_2.z(), 28.0);
}

// 测试不同数据类型的 t 参数
TEST_F(RayTest, AtMethodWithDifferentTypes) {
    Pt3 origin(0.0, 0.0, 0.0);
    Vec3 direction(1.0, 1.0, 0.0);  // 会被归一化
    Ray3 r(origin, direction);

    // 测试整数参数
    auto p_at_int = r.at(2);
    EXPECT_NEAR(p_at_int.x(), sqrt(2.0), 1e-6);  // 放宽精度要求
    EXPECT_NEAR(p_at_int.y(), sqrt(2.0), 1e-6);
    EXPECT_DOUBLE_EQ(p_at_int.z(), 0.0);

    // 测试浮点数参数
    auto p_at_float = r.at(1.5f);
    EXPECT_NEAR(p_at_float.x(), 1.5 / sqrt(2.0), 1e-6);  // 放宽精度要求
    EXPECT_NEAR(p_at_float.y(), 1.5 / sqrt(2.0), 1e-6);
    EXPECT_DOUBLE_EQ(p_at_float.z(), 0.0);
}

// 测试 from_unit_direction 静态方法
TEST_F(RayTest, FromUnitDirection) {
    Pt3 origin(1.0, 2.0, 3.0);
    Vec3 unit_direction(0.0, 1.0, 0.0);  // 已经是单位向量
    
    auto r = Ray3::from_unit_direction(origin, unit_direction);
    
    EXPECT_DOUBLE_EQ(r.origin().x(), 1.0);
    EXPECT_DOUBLE_EQ(r.origin().y(), 2.0);
    EXPECT_DOUBLE_EQ(r.origin().z(), 3.0);
    
    EXPECT_DOUBLE_EQ(r.direction().x(), 0.0);
    EXPECT_DOUBLE_EQ(r.direction().y(), 1.0);
    EXPECT_DOUBLE_EQ(r.direction().z(), 0.0);
}

// 测试混合类型构造
TEST_F(RayTest, MixedTypeConstruction) {
    Point<float, 3> origin_f(1.0f, 2.0f, 3.0f);
    Vector<float, 3> direction_f(1.0f, 0.0f, 0.0f);
    
    // 使用 float 类型构造 double 类型的 Ray
    Ray3 r(origin_f, direction_f);
    
    EXPECT_DOUBLE_EQ(r.origin().x(), 1.0);
    EXPECT_DOUBLE_EQ(r.origin().y(), 2.0);
    EXPECT_DOUBLE_EQ(r.origin().z(), 3.0);
    
    EXPECT_DOUBLE_EQ(r.direction().x(), 1.0);
    EXPECT_DOUBLE_EQ(r.direction().y(), 0.0);
    EXPECT_DOUBLE_EQ(r.direction().z(), 0.0);
}

// 测试 2D Ray
TEST_F(RayTest, Ray2D) {
    Point<double, 2> origin(1.0, 2.0);
    Vector<double, 2> direction(3.0, 4.0);  // 模长为 5
    
    Ray2 r(origin, direction);
    
    EXPECT_DOUBLE_EQ(r.origin().x(), 1.0);
    EXPECT_DOUBLE_EQ(r.origin().y(), 2.0);
    
    // 方向应该被归一化为 (3/5, 4/5)
    EXPECT_DOUBLE_EQ(r.direction().x(), 0.6);
    EXPECT_DOUBLE_EQ(r.direction().y(), 0.8);
    
    // 测试 at 方法
    auto p = r.at(5.0);  // 前进 5 单位
    EXPECT_DOUBLE_EQ(p.x(), 1.0 + 5.0 * 0.6);
    EXPECT_DOUBLE_EQ(p.y(), 2.0 + 5.0 * 0.8);
}

// 测试数值稳定性
TEST_F(RayTest, NumericalStability) {
    // 测试非常小的向量
    Pt3 origin(0.0, 0.0, 0.0);
    Vec3 tiny_direction(1e-10, 0.0, 0.0);
    
    Ray3 r(origin, tiny_direction);
    
    // 即使输入很小，归一化后的方向应该是 (1, 0, 0)
    EXPECT_NEAR(r.direction().x(), 1.0, 1e-10);
    EXPECT_NEAR(r.direction().y(), 0.0, 1e-15);
    EXPECT_NEAR(r.direction().z(), 0.0, 1e-15);
    
    // 长距离求点应该仍然准确
    auto far_point = r.at(1e6);
    EXPECT_NEAR(far_point.x(), 1e6, 1e-6);
    EXPECT_NEAR(far_point.y(), 0.0, 1e-10);
    EXPECT_NEAR(far_point.z(), 0.0, 1e-10);
}

// 测试边缘情况
TEST_F(RayTest, EdgeCases) {
    Pt3 origin(1.0, 2.0, 3.0);
    Vec3 zero_direction(0.0, 0.0, 0.0);

    // 零向量方向应该抛出异常
    EXPECT_THROW({ Ray3 r(origin, zero_direction); }, std::runtime_error);

    // 相同原点和目标点应该抛出异常
    EXPECT_THROW({ Ray3 r(origin, origin); }, std::runtime_error);
}

// 测试 RayDifferential 类
class RayDifferentialTest : public ::testing::Test {
protected:
    void SetUp() override {}
    void TearDown() override {}
};

// 测试 RayDifferential 的默认构造
TEST_F(RayDifferentialTest, DefaultConstruction) {
    RayDiff3 rd;
    
    // 检查主光线的默认值
    EXPECT_DOUBLE_EQ(rd.main_ray().origin().x(), 0.0);
    EXPECT_DOUBLE_EQ(rd.main_ray().origin().y(), 0.0);
    EXPECT_DOUBLE_EQ(rd.main_ray().origin().z(), 0.0);
    
    EXPECT_DOUBLE_EQ(rd.main_ray().direction().x(), 1.0);
    EXPECT_DOUBLE_EQ(rd.main_ray().direction().y(), 0.0);
    EXPECT_DOUBLE_EQ(rd.main_ray().direction().z(), 0.0);
}

// 测试带参数的 RayDifferential 构造
TEST_F(RayDifferentialTest, ParameterizedConstruction) {
    Ray3 main_ray(Pt3(1.0, 2.0, 3.0), Vec3(1.0, 0.0, 0.0));
    
    std::array<Ray3, 2> diff_rays = {
        Ray3(Pt3(1.1, 2.0, 3.0), Vec3(1.0, 0.1, 0.0)),
        Ray3(Pt3(1.0, 2.1, 3.0), Vec3(1.0, 0.0, 0.1))
    };
    
    RayDiff3 rd(main_ray, diff_rays);
    
    // 检查主光线
    EXPECT_DOUBLE_EQ(rd.main_ray().origin().x(), 1.0);
    EXPECT_DOUBLE_EQ(rd.main_ray().origin().y(), 2.0);
    EXPECT_DOUBLE_EQ(rd.main_ray().origin().z(), 3.0);
    
    // 检查微分光线访问 (使用近似比较避免浮点精度问题)
    EXPECT_NEAR(rd.x().origin().x(), 1.1, 1e-6);
    EXPECT_NEAR(rd.y().origin().y(), 2.1, 1e-6);
}

// 测试微分光线的访问方法
TEST_F(RayDifferentialTest, DifferentialRayAccess) {
    RayDiff3 rd;
    
    // 测试 x() 方法（第一个微分光线）
    rd.x() = Ray3(Pt3(1.0, 0.0, 0.0), Vec3(0.0, 1.0, 0.0));
    EXPECT_DOUBLE_EQ(rd.x().origin().x(), 1.0);
    EXPECT_DOUBLE_EQ(rd.x().direction().y(), 1.0);
    
    // 测试 y() 方法（第二个微分光线）
    rd.y() = Ray3(Pt3(0.0, 1.0, 0.0), Vec3(0.0, 0.0, 1.0));
    EXPECT_DOUBLE_EQ(rd.y().origin().y(), 1.0);
    EXPECT_DOUBLE_EQ(rd.y().direction().z(), 1.0);
    
    // 测试通过索引访问
    rd.differential_ray(0) = Ray3(Pt3(2.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0));
    EXPECT_DOUBLE_EQ(rd.differential_ray(0).origin().x(), 2.0);
    
    rd.differential_ray(1) = Ray3(Pt3(0.0, 2.0, 0.0), Vec3(0.0, 1.0, 0.0));
    EXPECT_DOUBLE_EQ(rd.differential_ray(1).origin().y(), 2.0);
}

// 测试 RayDifferential 的边界检查
TEST_F(RayDifferentialTest, BoundaryChecks) {
    RayDiff3 rd;
    
    // 测试越界访问
    EXPECT_THROW(rd.differential_ray(-1), std::runtime_error);
    EXPECT_THROW(rd.differential_ray(2), std::runtime_error);  // 3D 只有 2 个微分光线 (N-1)
}

// 测试 2D RayDifferential
TEST_F(RayDifferentialTest, RayDiff2D) {
    RayDiff2 rd2;
    
    // 2D 只有一个微分光线 (N-1 = 1)
    rd2.x() = Ray2(Point<double, 2>(1.0, 0.0), Vector<double, 2>(0.0, 1.0));
    EXPECT_DOUBLE_EQ(rd2.x().origin().x(), 1.0);
    EXPECT_DOUBLE_EQ(rd2.x().direction().y(), 1.0);
    
    // 应该只能访问索引 0
    EXPECT_NO_THROW(rd2.differential_ray(0));
    EXPECT_THROW(rd2.differential_ray(1), std::runtime_error);
}

// 测试 RayDifferential 的完整性
TEST_F(RayDifferentialTest, ComprehensiveTest) {
    // 创建一个复杂的场景
    Ray3 camera_ray(Pt3(0.0, 0.0, 0.0), Pt3(1.0, 1.0, -1.0));
    
    std::array<Ray3, 2> pixel_diff = {
        Ray3(Pt3(0.001, 0.0, 0.0), Pt3(1.001, 1.0, -1.0)),  // x 方向像素差异
        Ray3(Pt3(0.0, 0.001, 0.0), Pt3(1.0, 1.001, -1.0))   // y 方向像素差异
    };
    
    RayDiff3 camera_ray_diff(camera_ray, pixel_diff);
    
    // 验证主光线
    EXPECT_DOUBLE_EQ(camera_ray_diff.main_ray().origin().x(), 0.0);
    EXPECT_DOUBLE_EQ(camera_ray_diff.main_ray().origin().y(), 0.0);
    EXPECT_DOUBLE_EQ(camera_ray_diff.main_ray().origin().z(), 0.0);
    
    // 验证方向向量被正确归一化 (放宽精度要求)
    auto main_dir = camera_ray_diff.main_ray().direction();
    EXPECT_NEAR(main_dir.length(), 1.0, 1e-6);
    
    // 验证微分光线的差异性 (使用近似比较)
    auto x_diff_origin = camera_ray_diff.x().origin();
    auto y_diff_origin = camera_ray_diff.y().origin();
    
    EXPECT_NEAR(x_diff_origin.x(), 0.001, 1e-6);
    EXPECT_DOUBLE_EQ(x_diff_origin.y(), 0.0);
    
    EXPECT_DOUBLE_EQ(y_diff_origin.x(), 0.0);
    EXPECT_NEAR(y_diff_origin.y(), 0.001, 1e-6);
}

// 测试 differential_rays() 方法
TEST_F(RayDifferentialTest, DifferentialRaysAccess) {
    RayDiff3 rd;
    
    // 获取 differential_rays 数组引用并修改
    auto& diff_rays = rd.differential_rays();
    diff_rays[0] = Ray3(Pt3(1.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0));
    diff_rays[1] = Ray3(Pt3(0.0, 1.0, 0.0), Vec3(0.0, 1.0, 0.0));
    
    // 验证修改是否成功
    EXPECT_DOUBLE_EQ(rd.x().origin().x(), 1.0);
    EXPECT_DOUBLE_EQ(rd.y().origin().y(), 1.0);
    
    // 测试 const 版本
    const RayDiff3& const_rd = rd;
    const auto& const_diff_rays = const_rd.differential_rays();
    EXPECT_DOUBLE_EQ(const_diff_rays[0].origin().x(), 1.0);
    EXPECT_DOUBLE_EQ(const_diff_rays[1].origin().y(), 1.0);
}

// 测试主光线访问
TEST_F(RayDifferentialTest, MainRayAccess) {
    RayDiff3 rd;
    
    // 修改主光线
    rd.main_ray() = Ray3(Pt3(5.0, 6.0, 7.0), Vec3(1.0, 1.0, 1.0));
    
    // 验证修改
    EXPECT_DOUBLE_EQ(rd.main_ray().origin().x(), 5.0);
    EXPECT_DOUBLE_EQ(rd.main_ray().origin().y(), 6.0);
    EXPECT_DOUBLE_EQ(rd.main_ray().origin().z(), 7.0);
    
    // 方向应该被归一化 (放宽精度要求)
    EXPECT_NEAR(rd.main_ray().direction().length(), 1.0, 1e-6);
    
    // 测试 const 版本
    const RayDiff3& const_rd = rd;
    EXPECT_DOUBLE_EQ(const_rd.main_ray().origin().x(), 5.0);
}

// 测试单一缩放
TEST_F(RayDifferentialTest, UniformScaling) {
    Ray3 main_ray(Pt3(0.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0));
    std::array<Ray3, 2> diff_rays = {
        Ray3(Pt3(0.1, 0.0, 0.0), Vec3(1.0, 0.1, 0.0)),
        Ray3(Pt3(0.0, 0.1, 0.0), Vec3(1.0, 0.0, 0.1))
    };
    
    RayDiff3 rd(main_ray, diff_rays);
    
    // 记录缩放前的值 (使用近似比较避免浮点精度问题)
    EXPECT_NEAR(rd.x().origin().x(), 0.1, 1e-6);
    EXPECT_NEAR(rd.y().origin().y(), 0.1, 1e-6);
    
    // 应用 2.0 倍缩放
    auto& result = rd.scale(2.0);
    
    // 验证返回值是同一个对象的引用
    EXPECT_EQ(&result, &rd);
    
    // 主光线原点和方向不变
    EXPECT_DOUBLE_EQ(rd.main_ray().origin().x(), 0.0);
    EXPECT_DOUBLE_EQ(rd.main_ray().origin().y(), 0.0);
    EXPECT_DOUBLE_EQ(rd.main_ray().origin().z(), 0.0);
    EXPECT_DOUBLE_EQ(rd.main_ray().direction().x(), 1.0);
    EXPECT_DOUBLE_EQ(rd.main_ray().direction().y(), 0.0);
    EXPECT_DOUBLE_EQ(rd.main_ray().direction().z(), 0.0);
    
    // 微分光线的差异应该被缩放
    // 原来 x 微分光线的原点偏移是 0.1，现在应该是 0.2
    EXPECT_NEAR(rd.x().origin().x(), 0.2, 1e-5);
    EXPECT_DOUBLE_EQ(rd.x().origin().y(), 0.0);
    EXPECT_DOUBLE_EQ(rd.x().origin().z(), 0.0);
    
    EXPECT_DOUBLE_EQ(rd.y().origin().x(), 0.0);
    EXPECT_NEAR(rd.y().origin().y(), 0.2, 1e-5);
    EXPECT_DOUBLE_EQ(rd.y().origin().z(), 0.0);
    
    // 方向向量的差异也应该被缩放 (考虑到归一化引入的误差，使用更宽松的容差)
    EXPECT_NEAR(rd.x().direction().x(), 1.0, 1e-2);  // 主方向分量不变
    EXPECT_NEAR(rd.x().direction().y(), 0.2, 1e-2);  // 差异分量被缩放
    EXPECT_NEAR(rd.x().direction().z(), 0.0, 1e-6);
    
    EXPECT_NEAR(rd.y().direction().x(), 1.0, 1e-2);  // 主方向分量不变
    EXPECT_NEAR(rd.y().direction().y(), 0.0, 1e-6);
    EXPECT_NEAR(rd.y().direction().z(), 0.2, 1e-2);  // 差异分量被缩放
}

// 测试按数组缩放
TEST_F(RayDifferentialTest, ArrayScaling) {
    Ray3 main_ray(Pt3(0.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0));
    std::array<Ray3, 2> diff_rays = {
        Ray3(Pt3(0.1, 0.0, 0.0), Vec3(1.0, 0.1, 0.0)),
        Ray3(Pt3(0.0, 0.1, 0.0), Vec3(1.0, 0.0, 0.1))
    };
    
    RayDiff3 rd(main_ray, diff_rays);
    
    // 使用不同的缩放因子
    std::array<double, 2> scales = {2.0, 3.0};
    auto& result = rd.scale(scales);
    
    // 验证返回值是同一个对象的引用
    EXPECT_EQ(&result, &rd);
    
    // 主光线不变
    EXPECT_DOUBLE_EQ(rd.main_ray().origin().x(), 0.0);
    EXPECT_DOUBLE_EQ(rd.main_ray().direction().x(), 1.0);
    
    // x 微分光线应该按 2.0 缩放
    EXPECT_NEAR(rd.x().origin().x(), 0.2, 1e-5);
    EXPECT_DOUBLE_EQ(rd.x().origin().y(), 0.0);
    EXPECT_NEAR(rd.x().direction().y(), 0.2, 1e-2);
    
    // y 微分光线应该按 3.0 缩放  
    EXPECT_DOUBLE_EQ(rd.y().origin().x(), 0.0);
    EXPECT_NEAR(rd.y().origin().y(), 0.3, 1e-5);
    EXPECT_NEAR(rd.y().direction().z(), 0.3, 1e-2);
}

// 测试缩放的数学正确性
TEST_F(RayDifferentialTest, ScalingMathematicalCorrectness) {
    // 创建一个更复杂的场景
    Ray3 main_ray(Pt3(10.0, 20.0, 30.0), Vec3(0.0, 0.0, 1.0));
    std::array<Ray3, 2> diff_rays = {
        Ray3(Pt3(10.5, 20.0, 30.0), Vec3(0.1, 0.0, 1.0)),
        Ray3(Pt3(10.0, 20.5, 30.0), Vec3(0.0, 0.1, 1.0))
    };
    
    RayDiff3 rd(main_ray, diff_rays);
    
    // 应用缩放
    rd.scale(0.5);
    
    // 验证数学公式: new_diff = main + (old_diff - main) * scale
    // x 微分光线原点: (10.5, 20, 30) -> 10 + (10.5 - 10) * 0.5 = (10.25, 20, 30)
    EXPECT_DOUBLE_EQ(rd.x().origin().x(), 10.25);
    EXPECT_DOUBLE_EQ(rd.x().origin().y(), 20.0);
    EXPECT_DOUBLE_EQ(rd.x().origin().z(), 30.0);
    
    // y 微分光线原点: (10, 20.5, 30) -> 10 + (20.5 - 20) * 0.5 = (10, 20.25, 30)
    EXPECT_DOUBLE_EQ(rd.y().origin().x(), 10.0);
    EXPECT_DOUBLE_EQ(rd.y().origin().y(), 20.25);
    EXPECT_DOUBLE_EQ(rd.y().origin().z(), 30.0);
    
    // 验证方向向量的缩放 (考虑归一化误差，使用更宽松的容差)
    // x 微分方向: (0.1, 0, 1) (归一化) -> (0, 0, 1) + ((0.1, 0, 1) - (0, 0, 1)) * 0.5
    EXPECT_NEAR(rd.x().direction().x(), 0.05, 1e-2);
    EXPECT_NEAR(rd.x().direction().y(), 0.0, 1e-6);
    EXPECT_NEAR(rd.x().direction().z(), 1.0, 1e-2);
}

// 测试不同数据类型的缩放
TEST_F(RayDifferentialTest, ScalingWithDifferentTypes) {
    Ray3 main_ray(Pt3(0.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0));
    std::array<Ray3, 2> diff_rays = {
        Ray3(Pt3(1.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0)),
        Ray3(Pt3(0.0, 1.0, 0.0), Vec3(1.0, 0.0, 0.0))
    };
    
    RayDiff3 rd(main_ray, diff_rays);
    
    // 测试整数缩放
    rd.scale(2);
    EXPECT_DOUBLE_EQ(rd.x().origin().x(), 2.0);
    EXPECT_DOUBLE_EQ(rd.y().origin().y(), 2.0);
    
    // 重置
    rd = RayDiff3(main_ray, diff_rays);
    
    // 测试浮点数缩放
    rd.scale(1.5f);
    EXPECT_DOUBLE_EQ(rd.x().origin().x(), 1.5);
    EXPECT_DOUBLE_EQ(rd.y().origin().y(), 1.5);
    
    // 重置并测试数组缩放
    rd = RayDiff3(main_ray, diff_rays);
    std::array<float, 2> float_scales = {2.5f, 3.5f};
    rd.scale(float_scales);
    EXPECT_DOUBLE_EQ(rd.x().origin().x(), 2.5);
    EXPECT_DOUBLE_EQ(rd.y().origin().y(), 3.5);
}

// 测试链式缩放
TEST_F(RayDifferentialTest, ChainedScaling) {
    Ray3 main_ray(Pt3(0.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0));
    std::array<Ray3, 2> diff_rays = {
        Ray3(Pt3(1.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0)),
        Ray3(Pt3(0.0, 1.0, 0.0), Vec3(1.0, 0.0, 0.0))
    };
    
    RayDiff3 rd(main_ray, diff_rays);
    
    // 链式缩放: 2.0 * 1.5 = 3.0
    rd.scale(2.0).scale(1.5);
    
    EXPECT_DOUBLE_EQ(rd.x().origin().x(), 3.0);
    EXPECT_DOUBLE_EQ(rd.y().origin().y(), 3.0);
}

// 测试边界情况的缩放
TEST_F(RayDifferentialTest, ScalingEdgeCases) {
    Ray3 main_ray(Pt3(5.0, 5.0, 5.0), Vec3(1.0, 0.0, 0.0));
    std::array<Ray3, 2> diff_rays = {
        Ray3(Pt3(6.0, 5.0, 5.0), Vec3(1.0, 0.1, 0.0)),
        Ray3(Pt3(5.0, 6.0, 5.0), Vec3(1.0, 0.0, 0.1))
    };
    
    RayDiff3 rd(main_ray, diff_rays);
    
    // 测试零缩放
    rd.scale(0.0);
    EXPECT_DOUBLE_EQ(rd.x().origin().x(), 5.0);  // 应该回到主光线位置
    EXPECT_DOUBLE_EQ(rd.x().origin().y(), 5.0);
    EXPECT_DOUBLE_EQ(rd.x().origin().z(), 5.0);
    
    EXPECT_DOUBLE_EQ(rd.y().origin().x(), 5.0);
    EXPECT_DOUBLE_EQ(rd.y().origin().y(), 5.0);
    EXPECT_DOUBLE_EQ(rd.y().origin().z(), 5.0);
    
    // 方向也应该回到主方向
    EXPECT_NEAR(rd.x().direction().x(), 1.0, 1e-6);
    EXPECT_NEAR(rd.x().direction().y(), 0.0, 1e-6);
    EXPECT_NEAR(rd.y().direction().x(), 1.0, 1e-6);
    EXPECT_NEAR(rd.y().direction().z(), 0.0, 1e-6);
    
    // 重置并测试负缩放
    rd = RayDiff3(main_ray, diff_rays);
    rd.scale(-1.0);
    
    // 微分应该在主光线的另一边
    EXPECT_DOUBLE_EQ(rd.x().origin().x(), 4.0);  // 5 + (6 - 5) * (-1) = 4
    EXPECT_DOUBLE_EQ(rd.y().origin().y(), 4.0);  // 5 + (6 - 5) * (-1) = 4
}

// 测试 scaled 方法（返回副本的版本）
TEST_F(RayDifferentialTest, ScaledUniform) {
    Ray3 main_ray(Pt3(0.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0));
    std::array<Ray3, 2> diff_rays = {
        Ray3(Pt3(0.1, 0.0, 0.0), Vec3(1.0, 0.1, 0.0)),
        Ray3(Pt3(0.0, 0.1, 0.0), Vec3(1.0, 0.0, 0.1))
    };
    
    const RayDiff3 original(main_ray, diff_rays);
    
    // 使用 scaled 方法创建缩放后的副本
    auto scaled_rd = original.scaled(2.0);
    
    // 验证原始对象没有被修改
    EXPECT_NEAR(original.x().origin().x(), 0.1, 1e-6);
    EXPECT_NEAR(original.y().origin().y(), 0.1, 1e-6);
    
    // 验证新对象被正确缩放
    EXPECT_NEAR(scaled_rd.x().origin().x(), 0.2, 1e-5);
    EXPECT_NEAR(scaled_rd.y().origin().y(), 0.2, 1e-5);
    
    // 验证主光线保持不变
    EXPECT_DOUBLE_EQ(scaled_rd.main_ray().origin().x(), 0.0);
    EXPECT_DOUBLE_EQ(scaled_rd.main_ray().direction().x(), 1.0);
    
    // 验证方向向量的差异被缩放
    EXPECT_NEAR(scaled_rd.x().direction().y(), 0.2, 1e-2);
    EXPECT_NEAR(scaled_rd.y().direction().z(), 0.2, 1e-2);
}

// 测试 scaled 方法的数组版本
TEST_F(RayDifferentialTest, ScaledArray) {
    Ray3 main_ray(Pt3(0.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0));
    std::array<Ray3, 2> diff_rays = {
        Ray3(Pt3(0.1, 0.0, 0.0), Vec3(1.0, 0.1, 0.0)),
        Ray3(Pt3(0.0, 0.1, 0.0), Vec3(1.0, 0.0, 0.1))
    };
    
    const RayDiff3 original(main_ray, diff_rays);
    
    // 使用不同的缩放因子
    std::array<double, 2> scales = {3.0, 4.0};
    auto scaled_rd = original.scaled(scales);
    
    // 验证原始对象没有被修改
    EXPECT_NEAR(original.x().origin().x(), 0.1, 1e-6);
    EXPECT_NEAR(original.y().origin().y(), 0.1, 1e-6);
    
    // 验证新对象被正确缩放
    EXPECT_NEAR(scaled_rd.x().origin().x(), 0.3, 1e-5);  // 0.1 * 3.0
    EXPECT_NEAR(scaled_rd.y().origin().y(), 0.4, 1e-5);  // 0.1 * 4.0
    
    // 验证主光线保持不变
    EXPECT_DOUBLE_EQ(scaled_rd.main_ray().origin().x(), 0.0);
    EXPECT_DOUBLE_EQ(scaled_rd.main_ray().direction().x(), 1.0);
    
    // 验证方向向量的差异被缩放
    EXPECT_NEAR(scaled_rd.x().direction().y(), 0.3, 1e-2);
    EXPECT_NEAR(scaled_rd.y().direction().z(), 0.4, 1e-2);
}

// 测试 scaled 方法的不变性（const 正确性）
TEST_F(RayDifferentialTest, ScaledConstCorrectness) {
    Ray3 main_ray(Pt3(1.0, 2.0, 3.0), Vec3(0.0, 1.0, 0.0));
    std::array<Ray3, 2> diff_rays = {
        Ray3(Pt3(1.5, 2.0, 3.0), Vec3(0.1, 1.0, 0.0)),
        Ray3(Pt3(1.0, 2.5, 3.0), Vec3(0.0, 1.0, 0.1))
    };
    
    const RayDiff3 original(main_ray, diff_rays);
    
    // 记录原始值
    auto orig_x_origin = original.x().origin();
    auto orig_y_origin = original.y().origin();
    auto orig_main_origin = original.main_ray().origin();
    
    // 执行多次 scaled 操作
    auto scaled1 = original.scaled(0.5);
    auto scaled2 = original.scaled(2.0);
    auto scaled3 = original.scaled(std::array<double, 2>{1.5, 2.5});
    
    // 验证原始对象始终没有改变
    EXPECT_EQ(original.x().origin().x(), orig_x_origin.x());
    EXPECT_EQ(original.x().origin().y(), orig_x_origin.y());
    EXPECT_EQ(original.x().origin().z(), orig_x_origin.z());
    
    EXPECT_EQ(original.y().origin().x(), orig_y_origin.x());
    EXPECT_EQ(original.y().origin().y(), orig_y_origin.y());
    EXPECT_EQ(original.y().origin().z(), orig_y_origin.z());
    
    EXPECT_EQ(original.main_ray().origin().x(), orig_main_origin.x());
    EXPECT_EQ(original.main_ray().origin().y(), orig_main_origin.y());
    EXPECT_EQ(original.main_ray().origin().z(), orig_main_origin.z());
    
    // 验证每个缩放版本都是不同的
    EXPECT_NE(scaled1.x().origin().x(), scaled2.x().origin().x());
    EXPECT_NE(scaled1.x().origin().x(), scaled3.x().origin().x());
    EXPECT_NE(scaled2.x().origin().x(), scaled3.x().origin().x());
}

// 测试 scaled 方法的链式操作
TEST_F(RayDifferentialTest, ScaledChaining) {
    Ray3 main_ray(Pt3(0.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0));
    std::array<Ray3, 2> diff_rays = {
        Ray3(Pt3(1.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0)),
        Ray3(Pt3(0.0, 1.0, 0.0), Vec3(1.0, 0.0, 0.0))
    };
    
    RayDiff3 original(main_ray, diff_rays);
    
    // 链式调用 scaled 方法
    auto result = original.scaled(2.0).scaled(1.5);
    
    // 应该等效于 2.0 * 1.5 = 3.0 的缩放
    EXPECT_DOUBLE_EQ(result.x().origin().x(), 3.0);
    EXPECT_DOUBLE_EQ(result.y().origin().y(), 3.0);
}

// 测试 scaled 方法与 scale 方法的等价性
TEST_F(RayDifferentialTest, ScaledVsScaleEquivalence) {
    Ray3 main_ray(Pt3(5.0, 10.0, 15.0), Vec3(0.0, 0.0, 1.0));
    std::array<Ray3, 2> diff_rays = {
        Ray3(Pt3(5.2, 10.0, 15.0), Vec3(0.1, 0.0, 1.0)),
        Ray3(Pt3(5.0, 10.3, 15.0), Vec3(0.0, 0.1, 1.0))
    };
    
    RayDiff3 original(main_ray, diff_rays);
    RayDiff3 copy_for_scale = original;
    
    // 一个使用 scale（修改自身）
    copy_for_scale.scale(0.75);
    
    // 另一个使用 scaled（返回副本）
    auto scaled_result = original.scaled(0.75);
    
    // 两个结果应该相同
    EXPECT_DOUBLE_EQ(copy_for_scale.main_ray().origin().x(), scaled_result.main_ray().origin().x());
    EXPECT_DOUBLE_EQ(copy_for_scale.main_ray().origin().y(), scaled_result.main_ray().origin().y());
    EXPECT_DOUBLE_EQ(copy_for_scale.main_ray().origin().z(), scaled_result.main_ray().origin().z());
    
    EXPECT_NEAR(copy_for_scale.x().origin().x(), scaled_result.x().origin().x(), 1e-10);
    EXPECT_NEAR(copy_for_scale.x().origin().y(), scaled_result.x().origin().y(), 1e-10);
    EXPECT_NEAR(copy_for_scale.x().origin().z(), scaled_result.x().origin().z(), 1e-10);
    
    EXPECT_NEAR(copy_for_scale.y().origin().x(), scaled_result.y().origin().x(), 1e-10);
    EXPECT_NEAR(copy_for_scale.y().origin().y(), scaled_result.y().origin().y(), 1e-10);
    EXPECT_NEAR(copy_for_scale.y().origin().z(), scaled_result.y().origin().z(), 1e-10);
    
    // 验证方向向量也相同
    EXPECT_NEAR(copy_for_scale.x().direction().x(), scaled_result.x().direction().x(), 1e-6);
    EXPECT_NEAR(copy_for_scale.x().direction().y(), scaled_result.x().direction().y(), 1e-6);
    EXPECT_NEAR(copy_for_scale.x().direction().z(), scaled_result.x().direction().z(), 1e-6);
    
    EXPECT_NEAR(copy_for_scale.y().direction().x(), scaled_result.y().direction().x(), 1e-6);
    EXPECT_NEAR(copy_for_scale.y().direction().y(), scaled_result.y().direction().y(), 1e-6);
    EXPECT_NEAR(copy_for_scale.y().direction().z(), scaled_result.y().direction().z(), 1e-6);
}

// 测试 scaled 方法的不同数据类型
TEST_F(RayDifferentialTest, ScaledDifferentTypes) {
    Ray3 main_ray(Pt3(0.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0));
    std::array<Ray3, 2> diff_rays = {
        Ray3(Pt3(2.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0)),
        Ray3(Pt3(0.0, 2.0, 0.0), Vec3(1.0, 0.0, 0.0))
    };
    
    const RayDiff3 original(main_ray, diff_rays);
    
    // 测试整数缩放
    auto scaled_int = original.scaled(3);
    EXPECT_DOUBLE_EQ(scaled_int.x().origin().x(), 6.0);  // 2.0 * 3
    EXPECT_DOUBLE_EQ(scaled_int.y().origin().y(), 6.0);
    
    // 测试浮点数缩放
    auto scaled_float = original.scaled(1.5f);
    EXPECT_DOUBLE_EQ(scaled_float.x().origin().x(), 3.0);  // 2.0 * 1.5
    EXPECT_DOUBLE_EQ(scaled_float.y().origin().y(), 3.0);
    
    // 测试数组缩放
    std::array<int, 2> int_scales = {2, 4};
    auto scaled_array = original.scaled(int_scales);
    EXPECT_DOUBLE_EQ(scaled_array.x().origin().x(), 4.0);  // 2.0 * 2
    EXPECT_DOUBLE_EQ(scaled_array.y().origin().y(), 8.0);  // 2.0 * 4
}

}