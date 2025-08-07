#include <gtest/gtest.h>
#include <sstream>
#include <cmath>
#include <type_traits>

#include "math/geometry/spherical.hpp"
#include "math/geometry/point.hpp"
#include "math/geometry/vector.hpp"
#include "math/global/function.hpp"

namespace pbpt::math::testing {


bool are_almost_equal(Float a, Float b, Float epsilon = epsilon_v<Float>) {
    return pbpt::math::abs(a - b) < epsilon;
}

template<typename PointType>
bool points_almost_equal(const PointType& p1, const PointType& p2, Float epsilon = epsilon_v<Float>) {
    for (int i = 0; i < p1.dims(); ++i) {
        if (!are_almost_equal(p1[i], p2[i], epsilon)) {
            return false;
        }
    }
    return true;
}

// --- 2D 球坐标测试 ---
TEST(SphericalPointTest, Construction2D) {
    // 测试从角度和半径构造
    Vector<Float, 1> angles(M_PI / 4);  // 45度
    Float radius = 5.0f;
    SphericalPoint<Float, 2> sphere(angles, radius);
    
    EXPECT_FLOAT_EQ(sphere.radius(), 5.0f);
    EXPECT_FLOAT_EQ(sphere.angle(0), M_PI / 4);
}

TEST(SphericalPointTest, CartesianToSpherical2D) {
    // 测试从笛卡尔坐标构造球坐标
    Point<Float, 2> cart(3.0f, 4.0f);
    SphericalPoint<Float, 2> sphere(cart);
    
    // 验证半径: sqrt(3^2 + 4^2) = 5
    EXPECT_FLOAT_EQ(sphere.radius(), 5.0f);
    
    // 验证角度: atan2(4, 3)
    Float expected_angle = atan2(4.0f, 3.0f);
    EXPECT_TRUE(are_almost_equal(sphere.angle(0), expected_angle));
}

TEST(SphericalPointTest, SphericalToCartesian2D) {
    // 测试球坐标转换为笛卡尔坐标
    Vector<Float, 1> angles(M_PI / 6);  // 30度
    Float radius = 2.0f;
    SphericalPoint<Float, 2> sphere(angles, radius);
    
    Point<Float, 2> cart = sphere.to_cartesian();
    
    // 验证转换结果
    Float expected_x = 2.0f * cos(M_PI / 6);  // 2 * sqrt(3)/2
    Float expected_y = 2.0f * sin(M_PI / 6);  // 2 * 1/2
    
    EXPECT_TRUE(are_almost_equal(cart.x(), expected_x));
    EXPECT_TRUE(are_almost_equal(cart.y(), expected_y));
}

TEST(SphericalPointTest, RoundTrip2D) {
    // 测试往返转换的精度
    Point<Float, 2> original(1.5f, 2.5f);
    SphericalPoint<Float, 2> sphere(original);
    Point<Float, 2> converted = sphere.to_cartesian();
    
    EXPECT_TRUE(points_almost_equal(original, converted));
}

// --- 3D 球坐标测试 ---
TEST(SphericalPointTest, Construction3D) {
    // 测试从角度和半径构造
    Vector<Float, 2> angles(M_PI / 3, M_PI / 4);  // 极角60度，方位角45度
    Float radius = 10.0f;
    SphericalPoint<Float, 3> sphere(angles, radius);
    
    EXPECT_FLOAT_EQ(sphere.radius(), 10.0f);
    EXPECT_FLOAT_EQ(sphere.angle(0), M_PI / 3);  // 极角
    EXPECT_FLOAT_EQ(sphere.angle(1), M_PI / 4);  // 方位角
}

TEST(SphericalPointTest, CartesianToSpherical3D) {
    // 测试从笛卡尔坐标构造球坐标
    Point<Float, 3> cart(1.0f, 1.0f, sqrt(2.0f));
    SphericalPoint<Float, 3> sphere(cart);
    
    // 验证半径: sqrt(1^2 + 1^2 + (sqrt(2))^2) = 2
    EXPECT_TRUE(are_almost_equal(sphere.radius(), 2.0f));
    
    // 验证极角: acos(sqrt(2)/2) = π/4
    EXPECT_TRUE(are_almost_equal(sphere.angle(0), M_PI / 4));
    
    // 验证方位角: atan2(1, 1) = π/4
    EXPECT_TRUE(are_almost_equal(sphere.angle(1), M_PI / 4));
}

TEST(SphericalPointTest, SphericalToCartesian3D) {
    // 测试球坐标转换为笛卡尔坐标
    Vector<Float, 2> angles(M_PI / 2, 0.0f);  // 极角90度，方位角0度
    Float radius = 3.0f;
    SphericalPoint<Float, 3> sphere(angles, radius);
    
    Point<Float, 3> cart = sphere.to_cartesian();
    
    // 在极角90度时，z = r*cos(π/2) = 0
    // x = r*sin(π/2)*cos(0) = 3
    // y = r*sin(π/2)*sin(0) = 0
    EXPECT_TRUE(are_almost_equal(cart.x(), 3.0f));
    EXPECT_TRUE(are_almost_equal(cart.y(), 0.0f));
    EXPECT_TRUE(are_almost_equal(cart.z(), 0.0f));
}

TEST(SphericalPointTest, RoundTrip3D) {
    // 测试往返转换的精度
    Point<Float, 3> original(2.0f, 3.0f, 4.0f);
    SphericalPoint<Float, 3> sphere(original);
    Point<Float, 3> converted = sphere.to_cartesian();
    
    EXPECT_TRUE(points_almost_equal(original, converted));
}

// --- 4D 球坐标测试 ---
TEST(SphericalPointTest, Construction4D) {
    // 测试4维球坐标构造
    Vector<Float, 3> angles(M_PI / 6, M_PI / 4, M_PI / 3);
    Float radius = 5.0f;
    SphericalPoint<Float, 4> sphere(angles, radius);
    
    EXPECT_FLOAT_EQ(sphere.radius(), 5.0f);
    EXPECT_FLOAT_EQ(sphere.angle(0), M_PI / 6);
    EXPECT_FLOAT_EQ(sphere.angle(1), M_PI / 4);
    EXPECT_FLOAT_EQ(sphere.angle(2), M_PI / 3);
}

TEST(SphericalPointTest, RoundTrip4D) {
    // 测试4维往返转换的精度
    Point<Float, 4> original(1.0f, 2.0f, 3.0f, 4.0f);
    SphericalPoint<Float, 4> sphere(original);
    Point<Float, 4> converted = sphere.to_cartesian();
    
    EXPECT_TRUE(points_almost_equal(original, converted));
}

// --- 边界情况测试 ---
TEST(SphericalPointTest, ZeroRadius) {
    // 测试零半径情况
    Point<Float, 3> origin = Point<Float, 3>::zeros();
    SphericalPoint<Float, 3> sphere(origin);
    
    EXPECT_FLOAT_EQ(sphere.radius(), 0.0f);
    
    // 转换回笛卡尔坐标应该仍然是原点
    Point<Float, 3> converted = sphere.to_cartesian();
    EXPECT_TRUE(points_almost_equal(origin, converted));
}

TEST(SphericalPointTest, AxisAlignedPoints) {
    // 测试轴对齐的点
    
    // X轴正方向
    Point<Float, 3> x_axis(5.0f, 0.0f, 0.0f);
    SphericalPoint<Float, 3> sphere_x(x_axis);
    EXPECT_TRUE(are_almost_equal(sphere_x.radius(), 5.0f));
    EXPECT_TRUE(are_almost_equal(sphere_x.angle(0), M_PI / 2));  // 极角π/2
    EXPECT_TRUE(are_almost_equal(sphere_x.angle(1), 0.0f));  // 方位角0
    
    // Z轴正方向
    Point<Float, 3> z_axis(0.0f, 0.0f, 3.0f);
    SphericalPoint<Float, 3> sphere_z(z_axis);
    EXPECT_TRUE(are_almost_equal(sphere_z.radius(), 3.0f));
    EXPECT_TRUE(are_almost_equal(sphere_z.angle(0), 0.0f));  // 极角0
}

TEST(SphericalPointTest, NegativeCoordinates) {
    // 测试负坐标的处理
    Point<Float, 2> negative(-3.0f, -4.0f);
    SphericalPoint<Float, 2> sphere(negative);
    
    EXPECT_TRUE(are_almost_equal(sphere.radius(), 5.0f));
    
    // 角度应该在第三象限
    Float angle = sphere.angle(0);
    EXPECT_TRUE(angle > M_PI && angle < 2 * M_PI);
    
    // 往返转换验证
    Point<Float, 2> converted = sphere.to_cartesian();
    EXPECT_TRUE(points_almost_equal(negative, converted));
}

// --- 访问器方法测试 ---
TEST(SphericalPointTest, AccessorMethods) {
    // 测试3D情况
    Vector<Float, 2> angles_3d(1.0f, 2.0f);  // 极角1.0，方位角2.0
    Float radius = 7.0f;
    SphericalPoint<Float, 3> sphere_3d(angles_3d, radius);
    
    // 测试radius()方法
    EXPECT_FLOAT_EQ(sphere_3d.radius(), 7.0f);
    
    // 测试angle(i)方法
    EXPECT_FLOAT_EQ(sphere_3d.angle(0), 1.0f);  // 极角
    EXPECT_FLOAT_EQ(sphere_3d.angle(1), 2.0f);  // 方位角
    
    // 测试angles()方法
    const auto& all_angles_3d = sphere_3d.angles();
    EXPECT_FLOAT_EQ(all_angles_3d[0], 1.0f);
    EXPECT_FLOAT_EQ(all_angles_3d[1], 2.0f);
    
    // 测试新的azimuth()方法
    EXPECT_FLOAT_EQ(sphere_3d.azimuth(), 2.0f);  // 方位角应该是最后一个角度
    
    // 测试2D情况
    Vector<Float, 1> angles_2d(M_PI / 3);  // 方位角60度
    SphericalPoint<Float, 2> sphere_2d(angles_2d, 5.0f);
    
    EXPECT_FLOAT_EQ(sphere_2d.radius(), 5.0f);
    EXPECT_FLOAT_EQ(sphere_2d.angle(0), M_PI / 3);
    EXPECT_FLOAT_EQ(sphere_2d.azimuth(), M_PI / 3);  // 2D情况下方位角就是唯一的角度
}

// --- 精度和数值稳定性测试 ---
TEST(SphericalPointTest, NumericalStability) {
    // 测试非常小的值
    Point<Float, 3> tiny(1e-6f, 1e-6f, 1e-6f);
    SphericalPoint<Float, 3> sphere_tiny(tiny);
    Point<Float, 3> converted_tiny = sphere_tiny.to_cartesian();
    EXPECT_TRUE(points_almost_equal(tiny, converted_tiny, 1e-5f));
    
    // 测试非常大的值
    Point<Float, 3> large(1e6f, 1e6f, 1e6f);
    SphericalPoint<Float, 3> sphere_large(large);
    Point<Float, 3> converted_large = sphere_large.to_cartesian();
    EXPECT_TRUE(points_almost_equal(large, converted_large, 1e-3f));
}

// --- 类型别名测试 ---
TEST(SphericalPointTest, TypeAliases) {
    // 验证类型别名的正确性
    static_assert(std::is_same_v<Sphere2, SphericalPoint<Float, 2>>);
    static_assert(std::is_same_v<Sphere3, SphericalPoint<Float, 3>>);
    
    // 测试类型别名的使用
    Sphere2 sphere2d(Vector<Float, 1>(M_PI / 4), 2.0f);
    Sphere3 sphere3d(Vector<Float, 2>(M_PI / 4, M_PI / 3), 3.0f);
    
    EXPECT_FLOAT_EQ(sphere2d.radius(), 2.0f);
    EXPECT_FLOAT_EQ(sphere3d.radius(), 3.0f);
}

// --- main.cpp 测试用例 ---
TEST(SphericalPointTest, MainCppExample) {
    // 测试 main.cpp 中第12行的代码（更新为新的角度约定）
    // math::Sphere3 sp(math::Vec2{math::deg2rad(45.0), math::deg2rad(45.0)}, 1.0);
    Vec2 angles{deg2rad(45.0), deg2rad(45.0)};  // 极角45°，方位角45°
    Sphere3 sp(angles, 1.0);
    
    // 验证构造参数
    EXPECT_FLOAT_EQ(sp.radius(), 1.0f);
    EXPECT_TRUE(are_almost_equal(sp.angle(0), deg2rad(45.0)));  // 极角
    EXPECT_TRUE(are_almost_equal(sp.angle(1), deg2rad(45.0)));  // 方位角
    EXPECT_TRUE(are_almost_equal(sp.azimuth(), deg2rad(45.0)));  // 测试azimuth()方法
    
    // 测试转换为笛卡尔坐标
    Point<Float, 3> cartesian = sp.to_cartesian();
    
    // 计算期望值（新的角度约定：极角在前，方位角在后）
    // x = r * sin(theta) * cos(phi) = 1 * sin(45°) * cos(45°) = sin(π/4) * cos(π/4) = 0.5
    // y = r * sin(theta) * sin(phi) = 1 * sin(45°) * sin(45°) = sin(π/4) * sin(π/4) = 0.5  
    // z = r * cos(theta) = 1 * cos(45°) = cos(π/4) = √2/2
    Float expected_x = sin(deg2rad(45.0)) * cos(deg2rad(45.0));
    Float expected_y = sin(deg2rad(45.0)) * sin(deg2rad(45.0));
    Float expected_z = cos(deg2rad(45.0));
    
    EXPECT_TRUE(are_almost_equal(cartesian.x(), expected_x));
    EXPECT_TRUE(are_almost_equal(cartesian.y(), expected_y));
    EXPECT_TRUE(are_almost_equal(cartesian.z(), expected_z));
    
    // 测试往返转换
    Sphere3 sphere_from_cart(cartesian);
    Point<Float, 3> converted_back = sphere_from_cart.to_cartesian();
    EXPECT_TRUE(points_almost_equal(cartesian, converted_back));
}

// --- 编译时测试 ---
TEST(SphericalPointTest, ConstexprSupport) {
    // 验证constexpr支持
    constexpr Vector<Float, 1> angles(1.0f);
    constexpr Float radius = 2.0f;
    constexpr SphericalPoint<Float, 2> sphere(angles, radius);
    
    static_assert(sphere.radius() == 2.0f);
    static_assert(sphere.angle(0) == 1.0f);
    
    SUCCEED() << "Constexpr construction and access work correctly.";
}

} // namespace pbpt::math::testing