#include <gtest/gtest.h>

#include <cmath>
#include <type_traits>

#include "pbpt/pbpt.h"


namespace pbpt::geometry::testing {

bool are_almost_equal(math::Float a, math::Float b, math::Float epsilon = math::epsilon_v<math::Float>) {
    return pbpt::math::abs(a - b) < epsilon;
}

template <typename PointType>
bool points_almost_equal(const PointType& p1, const PointType& p2, math::Float epsilon = math::epsilon_v<math::Float>) {
    for (int i = 0; i < p1.dims(); ++i) {
        if (!are_almost_equal(p1[i], p2[i], epsilon)) {
            return false;
        }
    }
    return true;
}

template <typename PointType>
bool points_almost_equal_relative(const PointType& p1, const PointType& p2, math::Float relative_epsilon = math::Float(1e-5)) {
    for (int i = 0; i < p1.dims(); ++i) {
        math::Float abs_error = pbpt::math::abs(p1[i] - p2[i]);
        math::Float abs_p1 = pbpt::math::abs(p1[i]);
        math::Float abs_p2 = pbpt::math::abs(p2[i]);
        math::Float max_abs = std::max(abs_p1, abs_p2);

        // 使用相对误差，当数值很小时使用绝对误差
        if (max_abs > math::Float(1e-10)) {
            math::Float relative_error = abs_error / max_abs;
            if (relative_error > relative_epsilon) {
                return false;
            }
        } else {
            // 对于很小的数值，使用绝对误差
            if (abs_error > relative_epsilon) {
                return false;
            }
        }
    }
    return true;
}

// --- 2D 球坐标测试 ---
TEST(SphericalPointTest, Construction2D) {
    // 测试从角度和半径构造
    math::Vector<math::Float, 1> angles(M_PI / 4);  // 45度
    math::Float radius = 5.0f;
    SphericalPoint<math::Float, 2> sphere(angles, radius);

    EXPECT_FLOAT_EQ(sphere.radius(), 5.0f);
    EXPECT_FLOAT_EQ(sphere.angle(0), M_PI / 4);
}

TEST(SphericalPointTest, CartesianToSpherical2D) {
    // 测试从笛卡尔坐标构造球坐标
    math::Point<math::Float, 2> cart(3.0f, 4.0f);
    SphericalPoint<math::Float, 2> sphere(cart);

    // 验证半径: sqrt(3^2 + 4^2) = 5
    EXPECT_FLOAT_EQ(sphere.radius(), 5.0f);

    // 验证角度: atan2(4, 3)
    math::Float expected_angle = atan2(4.0f, 3.0f);
    EXPECT_TRUE(are_almost_equal(sphere.angle(0), expected_angle));
}

TEST(SphericalPointTest, SphericalToCartesian2D) {
    // 测试球坐标转换为笛卡尔坐标
    math::Vector<math::Float, 1> angles(M_PI / 6);  // 30度
    math::Float radius = 2.0f;
    SphericalPoint<math::Float, 2> sphere(angles, radius);

    math::Point<math::Float, 2> cart = sphere.to_cartesian();

    // 验证转换结果
    math::Float expected_x = 2.0f * cos(M_PI / 6);  // 2 * sqrt(3)/2
    math::Float expected_y = 2.0f * sin(M_PI / 6);  // 2 * 1/2

    EXPECT_TRUE(are_almost_equal(cart.x(), expected_x));
    EXPECT_TRUE(are_almost_equal(cart.y(), expected_y));
}

TEST(SphericalPointTest, RoundTrip2D) {
    // 测试往返转换的精度
    math::Point<math::Float, 2> original(1.5f, 2.5f);
    SphericalPoint<math::Float, 2> sphere(original);
    math::Point<math::Float, 2> converted = sphere.to_cartesian();

    EXPECT_TRUE(points_almost_equal(original, converted));
}

// --- 3D 球坐标测试 ---
TEST(SphericalPointTest, Construction3D) {
    // 测试从角度和半径构造
    math::Vector<math::Float, 2> angles(M_PI / 3, M_PI / 4);  // 极角60度，方位角45度
    math::Float radius = 10.0f;
    SphericalPoint<math::Float, 3> sphere(angles, radius);

    EXPECT_FLOAT_EQ(sphere.radius(), 10.0f);
    EXPECT_FLOAT_EQ(sphere.angle(0), M_PI / 3);  // 极角
    EXPECT_FLOAT_EQ(sphere.angle(1), M_PI / 4);  // 方位角
}

TEST(SphericalPointTest, CartesianToSpherical3D) {
    // 测试从笛卡尔坐标构造球坐标
    math::Point<math::Float, 3> cart(1.0f, 1.0f, sqrt(2.0f));
    SphericalPoint<math::Float, 3> sphere(cart);

    // 验证半径: sqrt(1^2 + 1^2 + (sqrt(2))^2) = 2
    EXPECT_TRUE(are_almost_equal(sphere.radius(), 2.0f));

    // 验证极角: acos(sqrt(2)/2) = π/4
    EXPECT_TRUE(are_almost_equal(sphere.angle(0), M_PI / 4));

    // 验证方位角: atan2(1, 1) = π/4
    EXPECT_TRUE(are_almost_equal(sphere.angle(1), M_PI / 4));
}

TEST(SphericalPointTest, SphericalToCartesian3D) {
    // 测试球坐标转换为笛卡尔坐标
    math::Vector<math::Float, 2> angles(M_PI / 2, 0.0f);  // 极角90度，方位角0度
    math::Float radius = 3.0f;
    SphericalPoint<math::Float, 3> sphere(angles, radius);

    math::Point<math::Float, 3> cart = sphere.to_cartesian();

    // 在极角90度时，z = r*cos(π/2) = 0
    // x = r*sin(π/2)*cos(0) = 3
    // y = r*sin(π/2)*sin(0) = 0
    EXPECT_TRUE(are_almost_equal(cart.x(), 3.0f));
    EXPECT_TRUE(are_almost_equal(cart.y(), 0.0f));
    EXPECT_TRUE(are_almost_equal(cart.z(), 0.0f));
}

TEST(SphericalPointTest, RoundTrip3D) {
    // 测试往返转换的精度
    math::Point<math::Float, 3> original(2.0f, 3.0f, 4.0f);
    SphericalPoint<math::Float, 3> sphere(original);
    math::Point<math::Float, 3> converted = sphere.to_cartesian();

    EXPECT_TRUE(points_almost_equal(original, converted));
}

// --- 4D 球坐标测试 ---
TEST(SphericalPointTest, Construction4D) {
    // 测试4维球坐标构造
    math::Vector<math::Float, 3> angles(M_PI / 6, M_PI / 4, M_PI / 3);
    math::Float radius = 5.0f;
    SphericalPoint<math::Float, 4> sphere(angles, radius);

    EXPECT_FLOAT_EQ(sphere.radius(), 5.0f);
    EXPECT_FLOAT_EQ(sphere.angle(0), M_PI / 6);
    EXPECT_FLOAT_EQ(sphere.angle(1), M_PI / 4);
    EXPECT_FLOAT_EQ(sphere.angle(2), M_PI / 3);
}

TEST(SphericalPointTest, RoundTrip4D) {
    // 测试4维往返转换的精度
    math::Point<math::Float, 4> original(1.0f, 2.0f, 3.0f, 4.0f);
    SphericalPoint<math::Float, 4> sphere(original);
    math::Point<math::Float, 4> converted = sphere.to_cartesian();

    EXPECT_TRUE(points_almost_equal(original, converted));
}

// --- 边界情况测试 ---
TEST(SphericalPointTest, ZeroRadius) {
    // 测试零半径情况
    math::Point<math::Float, 3> origin = math::Point<math::Float, 3>::zeros();
    SphericalPoint<math::Float, 3> sphere(origin);

    EXPECT_FLOAT_EQ(sphere.radius(), 0.0f);

    // 转换回笛卡尔坐标应该仍然是原点
    math::Point<math::Float, 3> converted = sphere.to_cartesian();
    EXPECT_TRUE(points_almost_equal(origin, converted));
}

TEST(SphericalPointTest, AxisAlignedPoints) {
    // 测试轴对齐的点

    // X轴正方向
    math::Point<math::Float, 3> x_axis(5.0f, 0.0f, 0.0f);
    SphericalPoint<math::Float, 3> sphere_x(x_axis);
    EXPECT_TRUE(are_almost_equal(sphere_x.radius(), 5.0f));
    EXPECT_TRUE(are_almost_equal(sphere_x.angle(0), M_PI / 2));  // 极角π/2
    EXPECT_TRUE(are_almost_equal(sphere_x.angle(1), 0.0f));      // 方位角0

    // Z轴正方向
    math::Point<math::Float, 3> z_axis(0.0f, 0.0f, 3.0f);
    SphericalPoint<math::Float, 3> sphere_z(z_axis);
    EXPECT_TRUE(are_almost_equal(sphere_z.radius(), 3.0f));
    EXPECT_TRUE(are_almost_equal(sphere_z.angle(0), 0.0f));  // 极角0
}

TEST(SphericalPointTest, NegativeCoordinates) {
    // 测试负坐标的处理
    math::Point<math::Float, 2> negative(-3.0f, -4.0f);
    SphericalPoint<math::Float, 2> sphere(negative);

    EXPECT_TRUE(are_almost_equal(sphere.radius(), 5.0f));

    // 角度应该在第三象限
    math::Float angle = sphere.angle(0);
    EXPECT_TRUE(angle > M_PI && angle < 2 * M_PI);

    // 往返转换验证
    math::Point<math::Float, 2> converted = sphere.to_cartesian();
    EXPECT_TRUE(points_almost_equal(negative, converted));
}

// --- 访问器方法测试 ---
TEST(SphericalPointTest, AccessorMethods) {
    // 测试3D情况
    math::Vector<math::Float, 2> angles_3d(1.0f, 2.0f);  // 极角1.0，方位角2.0
    math::Float radius = 7.0f;
    SphericalPoint<math::Float, 3> sphere_3d(angles_3d, radius);

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
    math::Vector<math::Float, 1> angles_2d(M_PI / 3);  // 方位角60度
    SphericalPoint<math::Float, 2> sphere_2d(angles_2d, 5.0f);

    EXPECT_FLOAT_EQ(sphere_2d.radius(), 5.0f);
    EXPECT_FLOAT_EQ(sphere_2d.angle(0), M_PI / 3);
    EXPECT_FLOAT_EQ(sphere_2d.azimuth(),
                    M_PI / 3);  // 2D情况下方位角就是唯一的角度
}

// --- 精度和数值稳定性测试 ---
TEST(SphericalPointTest, NumericalStability) {
    // 测试非常小的值
    math::Point<math::Float, 3> tiny(1e-6f, 1e-6f, 1e-6f);
    SphericalPoint<math::Float, 3> sphere_tiny(tiny);
    math::Point<math::Float, 3> converted_tiny = sphere_tiny.to_cartesian();
    EXPECT_TRUE(points_almost_equal(tiny, converted_tiny, 1e-5f));

    // 测试非常大的值
    math::Point<math::Float, 3> large(1e6f, 1e6f, 1e6f);
    SphericalPoint<math::Float, 3> sphere_large(large);
    math::Point<math::Float, 3> converted_large = sphere_large.to_cartesian();

    // 计算并输出详细的误差信息
    for (int i = 0; i < 3; ++i) {
        math::Float abs_error = pbpt::math::abs(large[i] - converted_large[i]);
        math::Float rel_error = abs_error / pbpt::math::abs(large[i]);
        std::cout << "Component " << i << ": original=" << large[i] << ", converted=" << converted_large[i]
                  << ", abs_error=" << abs_error << ", rel_error=" << rel_error << std::endl;
    }

    // 也输出球坐标信息
    std::cout << "Sphere radius: " << sphere_large.radius() << std::endl;
    std::cout << "Sphere angles: [" << sphere_large.angle(0) << ", " << sphere_large.angle(1) << "]" << std::endl;

    // 使用相对误差检查，1e-5 的相对精度对于大数值是合理的
    EXPECT_TRUE(points_almost_equal_relative(large, converted_large, math::Float(1e-5)))
        << "High precision conversion failed for large values";
}

// --- 类型别名测试 ---
TEST(SphericalPointTest, TypeAliases) {
    // 验证类型别名的正确性
    static_assert(std::is_same_v<Sphere2, SphericalPoint<math::Float, 2>>);
    static_assert(std::is_same_v<Sphere3, SphericalPoint<math::Float, 3>>);

    // 测试类型别名的使用
    Sphere2 sphere2d(math::Vector<math::Float, 1>(M_PI / 4), 2.0f);
    Sphere3 sphere3d(math::Vector<math::Float, 2>(M_PI / 4, M_PI / 3), 3.0f);

    EXPECT_FLOAT_EQ(sphere2d.radius(), 2.0f);
    EXPECT_FLOAT_EQ(sphere3d.radius(), 3.0f);
}

// --- main.cpp 测试用例 ---
TEST(SphericalPointTest, MainCppExample) {
    // 测试 main.cpp 中第12行的代码（更新为新的角度约定）
    // math::Sphere3 sp(math::Vec2{math::deg2rad(45.0),
    // math::deg2rad(45.0)}, 1.0);
    math::Vec2 angles{math::deg2rad(45.0), math::deg2rad(45.0)};  // 极角45°，方位角45°
    Sphere3 sp(angles, 1.0);

    // 验证构造参数
    EXPECT_FLOAT_EQ(sp.radius(), 1.0f);
    EXPECT_TRUE(are_almost_equal(sp.angle(0), math::deg2rad(45.0)));   // 极角
    EXPECT_TRUE(are_almost_equal(sp.angle(1), math::deg2rad(45.0)));   // 方位角
    EXPECT_TRUE(are_almost_equal(sp.azimuth(), math::deg2rad(45.0)));  // 测试azimuth()方法

    // 测试转换为笛卡尔坐标
    math::Point<math::Float, 3> cartesian = sp.to_cartesian();

    // 计算期望值（新的角度约定：极角在前，方位角在后）
    // x = r * sin(theta) * cos(phi) = 1 * sin(45°) * cos(45°) = sin(π/4) *
    // cos(π/4) = 0.5 y = r * sin(theta) * sin(phi) = 1 * sin(45°) * sin(45°) =
    // sin(π/4) * sin(π/4) = 0.5 z = r * cos(theta) = 1 * cos(45°) = cos(π/4) =
    // √2/2
    math::Float expected_x = sin(math::deg2rad(45.0)) * cos(math::deg2rad(45.0));
    math::Float expected_y = sin(math::deg2rad(45.0)) * sin(math::deg2rad(45.0));
    math::Float expected_z = cos(math::deg2rad(45.0));

    EXPECT_TRUE(are_almost_equal(cartesian.x(), expected_x));
    EXPECT_TRUE(are_almost_equal(cartesian.y(), expected_y));
    EXPECT_TRUE(are_almost_equal(cartesian.z(), expected_z));

    // 测试往返转换
    Sphere3 sphere_from_cart(cartesian);
    math::Point<math::Float, 3> converted_back = sphere_from_cart.to_cartesian();
    EXPECT_TRUE(points_almost_equal(cartesian, converted_back));
}

// --- 编译时测试 ---
TEST(SphericalPointTest, ConstexprSupport) {
    // 验证constexpr支持
    constexpr math::Vector<math::Float, 1> angles(1.0f);
    constexpr math::Float radius = 2.0f;
    constexpr SphericalPoint<math::Float, 2> sphere(angles, radius);

    static_assert(sphere.radius() == 2.0f);
    static_assert(sphere.angle(0) == 1.0f);

    SUCCEED() << "Constexpr construction and access work correctly.";
}

// ==================== wrap_angle_2pi Function Tests ====================

TEST(SphericalPointTest, WrapAngle2PiBasicTests) {
    // Test wrap_angle_2pi function with basic cases only

    // Test positive angles within range
    EXPECT_FLOAT_EQ(wrap_angle_2pi(math::Float(0)), math::Float(0));
    EXPECT_FLOAT_EQ(wrap_angle_2pi(math::pi_v<math::Float>), math::pi_v<math::Float>);
    EXPECT_FLOAT_EQ(wrap_angle_2pi(math::Float(1.5) * math::pi_v<math::Float>), math::Float(1.5) * math::pi_v<math::Float>);

    // Test simple negative angles
    EXPECT_FLOAT_EQ(wrap_angle_2pi(-math::pi_v<math::Float>), math::pi_v<math::Float>);
    EXPECT_FLOAT_EQ(wrap_angle_2pi(-math::pi_v<math::Float> / 2), math::Float(1.5) * math::pi_v<math::Float>);

    // Test simple large angles (with tolerance)
    EXPECT_TRUE(are_almost_equal(wrap_angle_2pi(math::Float(3) * math::pi_v<math::Float>), math::pi_v<math::Float>, math::Float(1e-5)));
}

// ==================== Enhanced Spherical Point Tests ====================

TEST(SphericalPointTest, FromCartesianStaticMethod) {
    // Test static factory method
    math::Point<math::Float, 2> pt_2d(3.0f, 4.0f);
    math::Point<math::Float, 3> pt_3d(1.0f, 2.0f, 3.0f);

    auto sphere_2d = SphericalPoint<math::Float, 2>::from_cartesian(pt_2d);
    auto sphere_3d = SphericalPoint<math::Float, 3>::from_cartesian(pt_3d);

    EXPECT_FLOAT_EQ(sphere_2d.radius(), 5.0f);         // sqrt(3^2 + 4^2) = 5
    EXPECT_FLOAT_EQ(sphere_3d.radius(), sqrt(14.0f));  // sqrt(1^2 + 2^2 + 3^2) = sqrt(14)
}

TEST(SphericalPointTest, HighDimensionalSupport) {
    // Test 5D spherical coordinates
    math::Point<math::Float, 5> pt_5d(1.0f, 2.0f, 3.0f, 4.0f, 5.0f);
    SphericalPoint<math::Float, 5> sphere_5d(pt_5d);
    math::Point<math::Float, 5> converted_5d = sphere_5d.to_cartesian();

    EXPECT_TRUE(points_almost_equal(pt_5d, converted_5d, math::Float(1e-5)));

    // Test 6D spherical coordinates
    math::Point<math::Float, 6> pt_6d(1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f);
    SphericalPoint<math::Float, 6> sphere_6d(pt_6d);
    math::Point<math::Float, 6> converted_6d = sphere_6d.to_cartesian();

    EXPECT_TRUE(points_almost_equal(pt_6d, converted_6d, math::Float(1e-5)));
}

TEST(SphericalPointTest, SpecialAngles3D) {
    // Test specific known angles

    // South pole (0, 0, -1)
    math::Point<math::Float, 3> south_pole(0.0f, 0.0f, -1.0f);
    SphericalPoint<math::Float, 3> sphere_south(south_pole);
    EXPECT_FLOAT_EQ(sphere_south.radius(), 1.0f);
    EXPECT_TRUE(are_almost_equal(sphere_south.angle(0), math::pi_v<math::Float>));  // θ = π for south pole

    // Test 45-degree elevation
    math::Point<math::Float, 3> pt_45deg(1.0f, 0.0f, 1.0f);  // Should have θ = π/4
    SphericalPoint<math::Float, 3> sphere_45(pt_45deg);
    math::Float expected_radius = sqrt(2.0f);
    EXPECT_TRUE(are_almost_equal(sphere_45.radius(), expected_radius));
    EXPECT_TRUE(are_almost_equal(sphere_45.angle(0), math::pi_v<math::Float> / 4));
    EXPECT_TRUE(are_almost_equal(sphere_45.angle(1), 0.0f));
}

TEST(SphericalPointTest, VerySmallValues) {
    // Test numerical stability with very small values
    math::Float epsilon = math::Float(1e-10);
    math::Point<math::Float, 2> small_2d(epsilon, epsilon);
    math::Point<math::Float, 3> small_3d(epsilon, epsilon, epsilon);

    SphericalPoint<math::Float, 2> sphere_2d_small(small_2d);
    SphericalPoint<math::Float, 3> sphere_3d_small(small_3d);

    // Should not crash and should have small but non-zero radius
    EXPECT_GT(sphere_2d_small.radius(), 0);
    EXPECT_GT(sphere_3d_small.radius(), 0);

    // Round trip should work
    math::Point<math::Float, 2> converted_2d = sphere_2d_small.to_cartesian();
    math::Point<math::Float, 3> converted_3d = sphere_3d_small.to_cartesian();

    EXPECT_TRUE(points_almost_equal(small_2d, converted_2d, math::Float(1e-8)));
    EXPECT_TRUE(points_almost_equal(small_3d, converted_3d, math::Float(1e-8)));
}

TEST(SphericalPointTest, ExtensiveRoundTripTests) {
    // Test more comprehensive round-trip conversions

    // 2D test cases
    std::vector<math::Point<math::Float, 2>> test_points_2d = {
        math::Point<math::Float, 2>(1, 0),       math::Point<math::Float, 2>(0, 1),      math::Point<math::Float, 2>(-1, 0), math::Point<math::Float, 2>(0, -1),
        math::Point<math::Float, 2>(1, 1),       math::Point<math::Float, 2>(-1, -1),    math::Point<math::Float, 2>(2, 3),  math::Point<math::Float, 2>(-1.5f, 2.5f),
        math::Point<math::Float, 2>(100, 0.01f), math::Point<math::Float, 2>(0.01f, 100)};

    for (const auto& pt : test_points_2d) {
        SphericalPoint<math::Float, 2> sphere(pt);
        math::Point<math::Float, 2> converted_back = sphere.to_cartesian();
        EXPECT_TRUE(points_almost_equal(pt, converted_back, math::Float(1e-5)))
            << "Failed for 2D point: (" << pt.x() << ", " << pt.y() << ")";
    }

    // 3D test cases
    std::vector<math::Point<math::Float, 3>> test_points_3d = {math::Point<math::Float, 3>(1, 0, 0),
                                                   math::Point<math::Float, 3>(0, 1, 0),
                                                   math::Point<math::Float, 3>(0, 0, 1),
                                                   math::Point<math::Float, 3>(-1, 0, 0),
                                                   math::Point<math::Float, 3>(0, -1, 0),
                                                   math::Point<math::Float, 3>(0, 0, -1),
                                                   math::Point<math::Float, 3>(1, 1, 1),
                                                   math::Point<math::Float, 3>(-1, -1, -1),
                                                   math::Point<math::Float, 3>(2, 3, 4),
                                                   math::Point<math::Float, 3>(-1.5f, 2.5f, -3.2f),
                                                   math::Point<math::Float, 3>(std::sqrt(3) / 2, 0.5f, 0),  // 30-60-90 triangle
                                                   math::Point<math::Float, 3>(0.5f, std::sqrt(3) / 2, 1)};

    for (const auto& pt : test_points_3d) {
        SphericalPoint<math::Float, 3> sphere(pt);
        math::Point<math::Float, 3> converted_back = sphere.to_cartesian();
        EXPECT_TRUE(points_almost_equal(pt, converted_back, math::Float(1e-5)))
            << "Failed for 3D point: (" << pt.x() << ", " << pt.y() << ", " << pt.z() << ")";
    }
}

// ==================== Spherical Geometry Function Tests ====================

TEST(SphericalGeometryTest, SphericalTriangleAreaBasic) {
    // Simple triangle on unit sphere
    math::Vector<math::Float, 3> a(1, 0, 0);  // Point on x-axis
    math::Vector<math::Float, 3> b(0, 1, 0);  // Point on y-axis
    math::Vector<math::Float, 3> c(0, 0, 1);  // North pole

    // Normalize the vectors
    a = a.normalized();
    b = b.normalized();
    c = c.normalized();

    math::Float area = spherical_triangle_area(a, b, c);
    EXPECT_GT(area, 0);
    EXPECT_LT(area, math::Float(4) * math::pi_v<math::Float>);  // Area should be reasonable

    // This specific triangle should have area π/2 (1/8 of sphere surface)
    EXPECT_TRUE(are_almost_equal(area, math::pi_v<math::Float> / 2, math::Float(1e-5)));
}

TEST(SphericalGeometryTest, SphericalPolygonAreaBasic) {
    // Test with a simple triangle first
    std::vector<math::Vector<math::Float, 3>> triangle_vertices = {math::Vector<math::Float, 3>(1, 0, 0).normalized(),
                                                       math::Vector<math::Float, 3>(0, 1, 0).normalized(),
                                                       math::Vector<math::Float, 3>(0, 0, 1).normalized()};

    math::Float triangle_area = spherical_polygon_area(triangle_vertices);
    EXPECT_GT(std::abs(triangle_area), math::Float(0));
    EXPECT_LT(std::abs(triangle_area), math::Float(4) * math::pi_v<math::Float>);
}

TEST(SphericalGeometryTest, ErrorHandling) {
    // Test error handling for invalid inputs

    // Test with degenerate triangle (collinear points)
    math::Vector<math::Float, 3> a(1, 0, 0);
    math::Vector<math::Float, 3> b(2, 0, 0);  // Same direction as a
    math::Vector<math::Float, 3> c(3, 0, 0);  // Same direction as a and b

    math::Float degenerate_area = spherical_triangle_area(a, b, c);
    // Degenerate triangle should have zero area
    EXPECT_TRUE(are_almost_equal(degenerate_area, math::Float(0), math::Float(1e-5)));

    // Test polygon with minimum vertices
    std::vector<math::Vector<math::Float, 3>> min_vertices = {math::Vector<math::Float, 3>(1, 0, 0).normalized(),
                                                  math::Vector<math::Float, 3>(0, 1, 0).normalized(),
                                                  math::Vector<math::Float, 3>(0, 0, 1).normalized()};

    math::Float min_area = spherical_polygon_area(min_vertices);
    EXPECT_GT(std::abs(min_area), math::Float(0));
}

// ==================== Enhanced Spherical Geometry Tests ====================

TEST(SphericalGeometryTest, SphericalTriangleAreaKnownValues) {
    // Test with known mathematical results

    // 1. Right spherical triangle with known area
    math::Vector<math::Float, 3> north_pole(0, 0, 1);
    math::Vector<math::Float, 3> equator_0(1, 0, 0);   // 0° longitude on equator
    math::Vector<math::Float, 3> equator_90(0, 1, 0);  // 90° longitude on equator

    math::Float right_triangle_area = spherical_triangle_area(north_pole, equator_0, equator_90);
    EXPECT_TRUE(are_almost_equal(right_triangle_area, math::pi_v<math::Float> / 2, math::Float(1e-4)));

    // 2. Small spherical triangle (should approximate planar area)
    math::Vector<math::Float, 3> small_a(1, 0, 0);
    math::Vector<math::Float, 3> small_b(math::Float(0.999), math::Float(0.01), math::Float(0.01));   // Very close to small_a
    math::Vector<math::Float, 3> small_c(math::Float(0.999), math::Float(0.01), math::Float(-0.01));  // Very close to small_a

    small_a = small_a.normalized();
    small_b = small_b.normalized();
    small_c = small_c.normalized();

    math::Float small_area = spherical_triangle_area(small_a, small_b, small_c);
    EXPECT_GT(small_area, math::Float(0));
    EXPECT_LT(small_area, math::Float(0.001));  // Should be very small
}

TEST(SphericalGeometryTest, SphericalTriangleAreaSymmetry) {
    // Test symmetry properties
    math::Vector<math::Float, 3> a(1, 0, 0);
    math::Vector<math::Float, 3> b(0, 1, 0);
    math::Vector<math::Float, 3> c(0, 0, 1);

    math::Float area_abc = spherical_triangle_area(a, b, c);
    math::Float area_acb = spherical_triangle_area(a, c, b);
    math::Float area_bac = spherical_triangle_area(b, a, c);
    math::Float area_bca = spherical_triangle_area(b, c, a);
    math::Float area_cab = spherical_triangle_area(c, a, b);
    math::Float area_cba = spherical_triangle_area(c, b, a);

    // All permutations should give the same absolute area
    EXPECT_TRUE(are_almost_equal(std::abs(area_abc), std::abs(area_acb), math::Float(1e-6)));
    EXPECT_TRUE(are_almost_equal(std::abs(area_abc), std::abs(area_bac), math::Float(1e-6)));
    EXPECT_TRUE(are_almost_equal(std::abs(area_abc), std::abs(area_bca), math::Float(1e-6)));
    EXPECT_TRUE(are_almost_equal(std::abs(area_abc), std::abs(area_cab), math::Float(1e-6)));
    EXPECT_TRUE(are_almost_equal(std::abs(area_abc), std::abs(area_cba), math::Float(1e-6)));
}

TEST(SphericalGeometryTest, SphericalTriangleAreaDegenerate) {
    // Test degenerate cases

    // 1. Collinear points (zero area)
    math::Vector<math::Float, 3> collinear_a(1, 0, 0);
    math::Vector<math::Float, 3> collinear_b(math::Float(0.7071), math::Float(0.7071), 0);  // 45° from a
    math::Vector<math::Float, 3> collinear_c(0, 1, 0);                          // 90° from a, collinear with a and b

    // Make them actually collinear on the sphere (great circle)
    math::Vector<math::Float, 3> gc_a(1, 0, 0);
    math::Vector<math::Float, 3> gc_b(math::Float(0.7071), math::Float(0.7071), 0);
    math::Vector<math::Float, 3> gc_c(0, 1, 0);

    math::Float collinear_area = spherical_triangle_area(gc_a, gc_b, gc_c);
    EXPECT_TRUE(are_almost_equal(collinear_area, math::Float(0), math::Float(1e-5)));

    // 2. Identical points (zero area)
    math::Vector<math::Float, 3> same_point(1, 0, 0);
    math::Float identical_area = spherical_triangle_area(same_point, same_point, same_point);
    EXPECT_TRUE(are_almost_equal(identical_area, math::Float(0), math::Float(1e-10)));

    // 3. Two identical points, one different (zero area)
    math::Vector<math::Float, 3> diff_point(0, 1, 0);
    math::Float two_same_area = spherical_triangle_area(same_point, same_point, diff_point);
    EXPECT_TRUE(are_almost_equal(two_same_area, math::Float(0), math::Float(1e-10)));
}

TEST(SphericalGeometryTest, SphericalTriangleAreaNumericalStability) {
    // Test numerical stability with various vector magnitudes

    // 1. Very large vectors (should be normalized internally by dot products)
    math::Vector<math::Float, 3> large_a(math::Float(1e6), 0, 0);
    math::Vector<math::Float, 3> large_b(0, math::Float(1e6), 0);
    math::Vector<math::Float, 3> large_c(0, 0, math::Float(1e6));

    // Normalize them first
    large_a = large_a.normalized();
    large_b = large_b.normalized();
    large_c = large_c.normalized();

    math::Float large_area = spherical_triangle_area(large_a, large_b, large_c);
    EXPECT_GT(large_area, math::Float(0));
    EXPECT_LT(large_area, math::Float(4) * math::pi_v<math::Float>);

    // 2. Very small vectors
    math::Vector<math::Float, 3> small_a(math::Float(1e-6), 0, 0);
    math::Vector<math::Float, 3> small_b(0, math::Float(1e-6), 0);
    math::Vector<math::Float, 3> small_c(0, 0, math::Float(1e-6));

    small_a = small_a.normalized();
    small_b = small_b.normalized();
    small_c = small_c.normalized();

    math::Float small_area = spherical_triangle_area(small_a, small_b, small_c);
    EXPECT_GT(small_area, math::Float(0));
    EXPECT_LT(small_area, math::Float(4) * math::pi_v<math::Float>);

    // Results should be the same regardless of input vector magnitude
    EXPECT_TRUE(are_almost_equal(large_area, small_area, math::Float(1e-5)));
}

TEST(SphericalGeometryTest, SphericalPolygonAreaPentagon) {
    // Test with a regular pentagon on the sphere
    std::vector<math::Vector<math::Float, 3>> pentagon_vertices;
    const int n_sides = 5;

    for (int i = 0; i < n_sides; ++i) {
        math::Float angle = math::Float(2 * i) * math::pi_v<math::Float> / math::Float(n_sides);
        pentagon_vertices.push_back(math::Vector<math::Float, 3>(cos(angle), sin(angle), 0).normalized());
    }

    math::Float pentagon_area = spherical_polygon_area(pentagon_vertices);
    EXPECT_GT(std::abs(pentagon_area), math::Float(0));
    EXPECT_LT(std::abs(pentagon_area), math::Float(4) * math::pi_v<math::Float>);

    // Pentagon should have larger area than triangle but smaller than square
    std::vector<math::Vector<math::Float, 3>> triangle_vertices = {math::Vector<math::Float, 3>(1, 0, 0).normalized(),
                                                       math::Vector<math::Float, 3>(0, 1, 0).normalized(),
                                                       math::Vector<math::Float, 3>(0, 0, 1).normalized()};
    math::Float triangle_area = std::abs(spherical_polygon_area(triangle_vertices));

    EXPECT_GT(std::abs(pentagon_area), triangle_area);
}

TEST(SphericalGeometryTest, SphericalPolygonAreaConsistency) {
    // Test that polygon area is consistent with triangulation
    std::vector<math::Vector<math::Float, 3>> quad_vertices = {
        math::Vector<math::Float, 3>(1, 0, 0).normalized(), math::Vector<math::Float, 3>(math::Float(0.7071), math::Float(0.7071), 0).normalized(),
        math::Vector<math::Float, 3>(0, 1, 0).normalized(), math::Vector<math::Float, 3>(math::Float(-0.7071), math::Float(0.7071), 0).normalized()};

    math::Float polygon_area = spherical_polygon_area(quad_vertices);

    // Manual triangulation: split into two triangles
    math::Float triangle1_area = spherical_triangle_area(quad_vertices[0], quad_vertices[1], quad_vertices[2]);
    math::Float triangle2_area = spherical_triangle_area(quad_vertices[0], quad_vertices[2], quad_vertices[3]);

    math::Float manual_area = triangle1_area + triangle2_area;

    // They should be approximately equal
    EXPECT_TRUE(are_almost_equal(std::abs(polygon_area), std::abs(manual_area), math::Float(1e-4)));
}

TEST(SphericalGeometryTest, SphericalPolygonAreaVertexOrder) {
    // Test that vertex order affects sign but not magnitude
    std::vector<math::Vector<math::Float, 3>> vertices_ccw = {math::Vector<math::Float, 3>(1, 0, 0).normalized(),
                                                  math::Vector<math::Float, 3>(0, 1, 0).normalized(),
                                                  math::Vector<math::Float, 3>(0, 0, 1).normalized()};

    std::vector<math::Vector<math::Float, 3>> vertices_cw = {math::Vector<math::Float, 3>(1, 0, 0).normalized(),
                                                 math::Vector<math::Float, 3>(0, 0, 1).normalized(),
                                                 math::Vector<math::Float, 3>(0, 1, 0).normalized()};

    math::Float area_ccw = spherical_polygon_area(vertices_ccw);
    math::Float area_cw = spherical_polygon_area(vertices_cw);

    // Magnitudes should be equal
    EXPECT_TRUE(are_almost_equal(std::abs(area_ccw), std::abs(area_cw), math::Float(1e-6)));

    // Signs might be different (depending on implementation)
    // But at least one should be positive
    EXPECT_TRUE(area_ccw > math::Float(0) || area_cw > math::Float(0));
}

TEST(SphericalGeometryTest, SphericalTriangleAreaEdgeCases) {
    // 2. Very close points (should have small area)
    math::Vector<math::Float, 3> close_a(1, 0, 0);
    math::Vector<math::Float, 3> close_b(math::Float(0.999999), math::Float(0.000001), 0);
    math::Vector<math::Float, 3> close_c(math::Float(0.999999), 0, math::Float(0.000001));

    close_a = close_a.normalized();
    close_b = close_b.normalized();
    close_c = close_c.normalized();

    math::Float close_area = spherical_triangle_area(close_a, close_b, close_c);
    EXPECT_GT(close_area, math::Float(0));
    EXPECT_LT(close_area, math::Float(1e-10));  // Should be very small
}

// ==================== Performance and Precision Tests ====================

TEST(SphericalPointTest, HighPrecisionMath) {
    // Test with mathematical constants for high precision
    math::Float e_const = math::Float(2.718281828459045);
    math::Float pi_const = math::pi_v<math::Float>;
    math::Float sqrt2_const = math::Float(1.4142135623730951);

    math::Point<math::Float, 3> math_pt(e_const, pi_const, sqrt2_const);

    SphericalPoint<math::Float, 3> sphere(math_pt);
    math::Point<math::Float, 3> converted = sphere.to_cartesian();

    // Use more reasonable tolerance for floating point precision
    EXPECT_TRUE(are_almost_equal(converted.x(), math_pt.x(), math::Float(1e-5)));
    EXPECT_TRUE(are_almost_equal(converted.y(), math_pt.y(), math::Float(1e-5)));
    EXPECT_TRUE(are_almost_equal(converted.z(), math_pt.z(), math::Float(1e-5)));
}

// ==================== Spherical Coordinate Utility Function Tests ====================

TEST(SphericalUtilityTest, CosThetaFunction) {
    // Test cos_theta function for 3D vectors

    // Test with unit vector along z-axis (north pole)
    math::Vector<math::Float, 3> north_pole(0, 0, 1);
    EXPECT_FLOAT_EQ(cos_theta<math::Float>(north_pole), math::Float(1));

    // Test with unit vector along negative z-axis (south pole)
    math::Vector<math::Float, 3> south_pole(0, 0, -1);
    EXPECT_FLOAT_EQ(cos_theta<math::Float>(south_pole), math::Float(-1));

    // Test with unit vector in xy-plane (equator)
    math::Vector<math::Float, 3> equator(1, 0, 0);
    EXPECT_FLOAT_EQ(cos_theta<math::Float>(equator), math::Float(0));

    // Test with 45-degree elevation
    math::Vector<math::Float, 3> v_45deg = math::Vector<math::Float, 3>(1, 0, 1).normalized();
    EXPECT_TRUE(are_almost_equal(cos_theta<math::Float>(v_45deg), math::Float(1.0 / std::sqrt(2))));

    // Test with arbitrary vector
    math::Vector<math::Float, 3> arbitrary(3, 4, 5);  // z = 5
    math::Float expected_cos = math::Float(5);        // cos_theta just returns v.z()
    EXPECT_FLOAT_EQ(cos_theta<math::Float>(arbitrary), expected_cos);
}

TEST(SphericalUtilityTest, CosThetaSqFunction) {
    // Test cos_theta_sq function
    math::Vector<math::Float, 3> v(1, 2, 3);  // z = 3
    math::Float expected = math::Float(9);    // z^2 = 9
    EXPECT_FLOAT_EQ(cos2_theta<math::Float>(v), expected);

    // Test with normalized vector
    math::Vector<math::Float, 3> normalized = v.normalized();
    math::Float cos_val = cos_theta<math::Float>(normalized);
    EXPECT_TRUE(are_almost_equal(cos2_theta<math::Float>(normalized), cos_val * cos_val));
}

TEST(SphericalUtilityTest, SinThetaSqFunction) {
    // Test sin_theta_sq function
    math::Vector<math::Float, 3> north_pole(0, 0, 1);
    EXPECT_TRUE(are_almost_equal(sin2_theta<math::Float>(north_pole), math::Float(0), math::Float(1e-10)));

    math::Vector<math::Float, 3> equator(1, 0, 0);
    EXPECT_TRUE(are_almost_equal(sin2_theta<math::Float>(equator), math::Float(1), math::Float(1e-10)));

    // Test identity: sin²θ + cos²θ = 1
    math::Vector<math::Float, 3> arbitrary(2, 3, 4);
    arbitrary = arbitrary.normalized();
    math::Float sin_sq = sin2_theta<math::Float>(arbitrary);
    math::Float cos_sq = cos2_theta<math::Float>(arbitrary);
    EXPECT_TRUE(are_almost_equal(sin_sq + cos_sq, math::Float(1), math::Float(1e-6)));
}

TEST(SphericalUtilityTest, SinThetaFunction) {
    // Test sin_theta function
    math::Vector<math::Float, 3> north_pole(0, 0, 1);
    EXPECT_TRUE(are_almost_equal(sin_theta<math::Float>(north_pole), math::Float(0), math::Float(1e-10)));

    math::Vector<math::Float, 3> equator(1, 0, 0);
    EXPECT_TRUE(are_almost_equal(sin_theta<math::Float>(equator), math::Float(1), math::Float(1e-10)));

    math::Vector<math::Float, 3> v_45deg = math::Vector<math::Float, 3>(1, 0, 1).normalized();
    EXPECT_TRUE(are_almost_equal(sin_theta<math::Float>(v_45deg), math::Float(1.0 / std::sqrt(2))));

    // Test that sin_theta is always non-negative (implementation uses sqrt)
    math::Vector<math::Float, 3> various_vectors[] = {math::Vector<math::Float, 3>(1, 2, 3).normalized(),
                                          math::Vector<math::Float, 3>(-1, -2, -3).normalized(),
                                          math::Vector<math::Float, 3>(0, 1, -1).normalized()};

    for (const auto& v : various_vectors) {
        EXPECT_GE(sin_theta<math::Float>(v), math::Float(0));
    }
}

TEST(SphericalUtilityTest, TanThetaFunction) {
    // Test tan_theta function
    math::Vector<math::Float, 3> equator(1, 0, 0);
    // For equator: cos_theta = 0, sin_theta = 1, so tan should be infinity
    // But in practice, we need to handle division by zero carefully
    math::Float tan_val = tan_theta<math::Float>(equator);
    EXPECT_TRUE(std::isinf(tan_val) || std::abs(tan_val) > math::Float(1e6));  // Very large value

    math::Vector<math::Float, 3> v_45deg = math::Vector<math::Float, 3>(1, 0, 1).normalized();
    EXPECT_TRUE(are_almost_equal(tan_theta<math::Float>(v_45deg), math::Float(1)));

    math::Vector<math::Float, 3> north_pole(0, 0, 1);
    EXPECT_TRUE(are_almost_equal(tan_theta<math::Float>(north_pole), math::Float(0), math::Float(1e-10)));
}

TEST(SphericalUtilityTest, TanThetaSqFunction) {
    // Test tan_theta_sq function
    math::Vector<math::Float, 3> v_45deg = math::Vector<math::Float, 3>(1, 0, 1).normalized();
    EXPECT_TRUE(are_almost_equal(tan2_theta<math::Float>(v_45deg), math::Float(1)));

    math::Vector<math::Float, 3> north_pole(0, 0, 1);
    EXPECT_TRUE(are_almost_equal(tan2_theta<math::Float>(north_pole), math::Float(0), math::Float(1e-10)));

    // Test relationship with sin and cos (avoid division by zero)
    math::Vector<math::Float, 3> arbitrary(1, 2, 3);
    arbitrary = arbitrary.normalized();
    if (std::abs(cos_theta<math::Float>(arbitrary)) > math::Float(1e-6)) {
        math::Float tan_sq_direct = tan2_theta<math::Float>(arbitrary);
        math::Float tan_from_sin_cos = sin2_theta<math::Float>(arbitrary) / cos2_theta<math::Float>(arbitrary);
        EXPECT_TRUE(are_almost_equal(tan_sq_direct, tan_from_sin_cos, math::Float(1e-6)));
    }
}

TEST(SphericalUtilityTest, PhiFunction) {
    // Test phi function for azimuthal angle

    // Test cardinal directions
    math::Vector<math::Float, 3> pos_x(1, 0, 0);
    EXPECT_TRUE(are_almost_equal(phi(pos_x), math::Float(0), math::Float(1e-6)));

    math::Vector<math::Float, 3> pos_y(0, 1, 0);
    EXPECT_TRUE(are_almost_equal(phi(pos_y), math::pi_v<math::Float> / 2, math::Float(1e-6)));

    math::Vector<math::Float, 3> neg_x(-1, 0, 0);
    EXPECT_TRUE(are_almost_equal(phi(neg_x), math::pi_v<math::Float>, math::Float(1e-6)));

    math::Vector<math::Float, 3> neg_y(0, -1, 0);
    EXPECT_TRUE(are_almost_equal(phi(neg_y), math::Float(3) * math::pi_v<math::Float> / 2, math::Float(1e-6)));

    // Test 45-degree angles
    math::Vector<math::Float, 3> northeast(1, 1, 0);
    EXPECT_TRUE(are_almost_equal(phi(northeast), math::pi_v<math::Float> / 4, math::Float(1e-6)));

    math::Vector<math::Float, 3> southwest(-1, -1, 0);
    EXPECT_TRUE(are_almost_equal(phi(southwest), math::Float(5) * math::pi_v<math::Float> / 4, math::Float(1e-6)));

    // Test that phi is in range [0, 2π)
    std::vector<math::Vector<math::Float, 3>> test_vectors = {math::Vector<math::Float, 3>(1, 1, 1), math::Vector<math::Float, 3>(-1, 1, -1),
                                                  math::Vector<math::Float, 3>(2, -3, 4), math::Vector<math::Float, 3>(-5, -2, 1)};

    for (const auto& v : test_vectors) {
        math::Float phi_val = phi(v);
        EXPECT_GE(phi_val, math::Float(0));
        EXPECT_LT(phi_val, math::Float(2) * math::pi_v<math::Float>);
    }
}

TEST(SphericalUtilityTest, SinPhiFunction) {
    // Test sin_phi function

    // Test cardinal directions
    math::Vector<math::Float, 3> pos_x(1, 0, 1);  // Include z to avoid singularity
    pos_x = pos_x.normalized();
    EXPECT_TRUE(are_almost_equal(sin_phi(pos_x), math::Float(0), math::Float(1e-6)));

    math::Vector<math::Float, 3> pos_y(0, 1, 1);
    pos_y = pos_y.normalized();
    EXPECT_TRUE(are_almost_equal(sin_phi(pos_y), math::Float(1), math::Float(1e-6)));

    math::Vector<math::Float, 3> neg_y(0, -1, 1);
    neg_y = neg_y.normalized();
    EXPECT_TRUE(are_almost_equal(sin_phi(neg_y), math::Float(-1), math::Float(1e-6)));

    // Test 45-degree angle
    math::Vector<math::Float, 3> northeast(1, 1, 1);
    northeast = northeast.normalized();
    EXPECT_TRUE(are_almost_equal(sin_phi(northeast), math::Float(1) / std::sqrt(math::Float(2)), math::Float(1e-6)));

    // Test singularity at poles (sin_theta = 0)
    math::Vector<math::Float, 3> north_pole(0, 0, 1);
    EXPECT_TRUE(are_almost_equal(sin_phi(north_pole), math::Float(0), math::Float(1e-10)));

    math::Vector<math::Float, 3> south_pole(0, 0, -1);
    EXPECT_TRUE(are_almost_equal(sin_phi(south_pole), math::Float(0), math::Float(1e-10)));
}

TEST(SphericalUtilityTest, CosPhiFunction) {
    // Test cos_phi function

    // Test cardinal directions
    math::Vector<math::Float, 3> pos_x(1, 0, 1);
    pos_x = pos_x.normalized();
    EXPECT_TRUE(are_almost_equal(cos_phi(pos_x), math::Float(1), math::Float(1e-6)));

    math::Vector<math::Float, 3> pos_y(0, 1, 1);
    pos_y = pos_y.normalized();
    EXPECT_TRUE(are_almost_equal(cos_phi(pos_y), math::Float(0), math::Float(1e-6)));

    math::Vector<math::Float, 3> neg_x(-1, 0, 1);
    neg_x = neg_x.normalized();
    EXPECT_TRUE(are_almost_equal(cos_phi(neg_x), math::Float(-1), math::Float(1e-6)));

    // Test 45-degree angle
    math::Vector<math::Float, 3> northeast(1, 1, 1);
    northeast = northeast.normalized();
    EXPECT_TRUE(are_almost_equal(cos_phi(northeast), math::Float(1) / std::sqrt(math::Float(2)), math::Float(1e-6)));

    // Test singularity at poles
    math::Vector<math::Float, 3> north_pole(0, 0, 1);
    EXPECT_TRUE(are_almost_equal(cos_phi(north_pole), math::Float(1), math::Float(1e-10)));  // Default value

    // Test identity: sin²φ + cos²φ = 1 (for non-pole vectors)
    math::Vector<math::Float, 3> arbitrary(2, 3, 4);
    arbitrary = arbitrary.normalized();
    math::Float sin_phi_val = sin_phi(arbitrary);
    math::Float cos_phi_val = cos_phi(arbitrary);
    EXPECT_TRUE(are_almost_equal(sin_phi_val * sin_phi_val + cos_phi_val * cos_phi_val, math::Float(1), math::Float(1e-6)));
}

TEST(SphericalUtilityTest, CosDeltaPhiFunction) {
    // Test cos_delta_phi function for angle between projections

    // Test identical vectors
    math::Vector<math::Float, 3> v(1, 2, 3);
    EXPECT_TRUE(are_almost_equal(cos_delta_phi(v, v), math::Float(1), math::Float(1e-10)));

    // Test orthogonal vectors in xy-plane
    math::Vector<math::Float, 3> v1(1, 0, 5);  // Same z, different xy
    math::Vector<math::Float, 3> v2(0, 1, 5);
    EXPECT_TRUE(are_almost_equal(cos_delta_phi(v1, v2), math::Float(0), math::Float(1e-6)));

    // Test opposite vectors in xy-plane
    math::Vector<math::Float, 3> v3(1, 0, 3);
    math::Vector<math::Float, 3> v4(-1, 0, 3);
    EXPECT_TRUE(are_almost_equal(cos_delta_phi(v3, v4), math::Float(-1), math::Float(1e-6)));

    // Test 45-degree difference in xy-plane
    math::Vector<math::Float, 3> v5(1, 0, 2);
    math::Vector<math::Float, 3> v6(1, 1, 2);
    v6 = v6.normalized() * v5.length();                   // Maintain same z and radius in xy
    math::Float expected_cos = math::Float(1) / std::sqrt(math::Float(2));  // cos(45°)
    EXPECT_TRUE(are_almost_equal(cos_delta_phi(v5, v6), expected_cos, math::Float(1e-5)));

    // Test singularities (vectors with zero xy components)
    math::Vector<math::Float, 3> pole1(0, 0, 1);
    math::Vector<math::Float, 3> pole2(0, 0, -1);
    EXPECT_TRUE(are_almost_equal(cos_delta_phi(pole1, pole2), math::Float(1), math::Float(1e-10)));  // Default value

    math::Vector<math::Float, 3> mixed1(0, 0, 1);
    math::Vector<math::Float, 3> mixed2(1, 0, 2);
    EXPECT_TRUE(are_almost_equal(cos_delta_phi(mixed1, mixed2), math::Float(1), math::Float(1e-10)));
}

TEST(SphericalUtilityTest, TrigonometricIdentities) {
    // Test various trigonometric identities for spherical coordinates

    std::vector<math::Vector<math::Float, 3>> test_vectors = {
        math::Vector<math::Float, 3>(1, 0, 0).normalized(),  math::Vector<math::Float, 3>(0, 1, 0).normalized(),
        math::Vector<math::Float, 3>(0, 0, 1).normalized(),  math::Vector<math::Float, 3>(1, 1, 1).normalized(),
        math::Vector<math::Float, 3>(2, -3, 4).normalized(), math::Vector<math::Float, 3>(-1, 2, -3).normalized()};

    for (const auto& v : test_vectors) {
        // Test sin²θ + cos²θ = 1
        math::Float sin_sq = sin2_theta(v);
        math::Float cos_sq = cos2_theta(v);
        EXPECT_TRUE(are_almost_equal(sin_sq + cos_sq, math::Float(1), math::Float(1e-6)))
            << "sin²θ + cos²θ = 1 failed for vector (" << v.x() << ", " << v.y() << ", " << v.z() << ")";

        // Test sin²φ + cos²φ = 1 (except at poles)
        if (sin_theta(v) > math::Float(1e-6)) {  // Avoid poles
            math::Float sin_phi_val = sin_phi(v);
            math::Float cos_phi_val = cos_phi(v);
            EXPECT_TRUE(are_almost_equal(sin_phi_val * sin_phi_val + cos_phi_val * cos_phi_val, math::Float(1), math::Float(1e-6)))
                << "sin²φ + cos²φ = 1 failed for vector (" << v.x() << ", " << v.y() << ", " << v.z() << ")";
        }

        // Test tan²θ = sin²θ / cos²θ (except when cos_theta = 0)
        if (std::abs(cos_theta(v)) > math::Float(1e-6)) {
            math::Float tan_sq_direct = tan2_theta(v);
            math::Float tan_sq_from_trig = sin2_theta(v) / cos2_theta(v);
            EXPECT_TRUE(are_almost_equal(tan_sq_direct, tan_sq_from_trig, math::Float(1e-6)))
                << "tan²θ identity failed for vector (" << v.x() << ", " << v.y() << ", " << v.z() << ")";
        }
    }
}

// ==================== Equal Area Mapping Tests ====================

TEST(SphericalUtilityTest, EqualAreaSquareToSphereBasic) {
    // Test equal_area_square_to_sphere function

    // Test corners of unit square [0,1]²
    math::Vector<math::Float, 2> corner_00(0, 0);
    math::Vector<math::Float, 2> corner_01(0, 1);
    math::Vector<math::Float, 2> corner_10(1, 0);
    math::Vector<math::Float, 2> corner_11(1, 1);

    auto sphere_00 = equal_area_square_to_sphere(corner_00);
    auto sphere_01 = equal_area_square_to_sphere(corner_01);
    auto sphere_10 = equal_area_square_to_sphere(corner_10);
    auto sphere_11 = equal_area_square_to_sphere(corner_11);

    // All results should be unit vectors
    EXPECT_TRUE(are_almost_equal(sphere_00.length(), math::Float(1), math::Float(1e-6)));
    EXPECT_TRUE(are_almost_equal(sphere_01.length(), math::Float(1), math::Float(1e-6)));
    EXPECT_TRUE(are_almost_equal(sphere_10.length(), math::Float(1), math::Float(1e-6)));
    EXPECT_TRUE(are_almost_equal(sphere_11.length(), math::Float(1), math::Float(1e-6)));

    // Test center of square
    math::Vector<math::Float, 2> center(0.5f, 0.5f);
    auto sphere_center = equal_area_square_to_sphere(center);
    EXPECT_TRUE(are_almost_equal(sphere_center.length(), math::Float(1), math::Float(1e-6)));

    // Center should map to some reasonable point on sphere
    EXPECT_GE(sphere_center.z(), math::Float(-1));
    EXPECT_LE(sphere_center.z(), math::Float(1));
}

TEST(SphericalUtilityTest, EqualAreaSquareToSphereCoverage) {
    // Test that the mapping covers the sphere reasonably
    const int samples = 10;
    std::vector<math::Vector<math::Float, 3>> mapped_points;

    for (int i = 0; i <= samples; ++i) {
        for (int j = 0; j <= samples; ++j) {
            math::Float u = math::Float(i) / math::Float(samples);
            math::Float v = math::Float(j) / math::Float(samples);
            math::Vector<math::Float, 2> square_pt(u, v);
            auto sphere_pt = equal_area_square_to_sphere(square_pt);

            // Should be unit vector
            EXPECT_TRUE(are_almost_equal(sphere_pt.length(), math::Float(1), math::Float(1e-5)));
            mapped_points.push_back(sphere_pt);
        }
    }

    // Check that we get reasonable coverage of the sphere
    // Find min/max z values
    math::Float min_z = mapped_points[0].z();
    math::Float max_z = mapped_points[0].z();

    for (const auto& pt : mapped_points) {
        min_z = std::min(min_z, pt.z());
        max_z = std::max(max_z, pt.z());
    }

    // Should cover a reasonable range of z values
    EXPECT_LT(min_z, math::Float(0));            // Should reach southern hemisphere
    EXPECT_GT(max_z, math::Float(0));            // Should reach northern hemisphere
    EXPECT_GT(max_z - min_z, math::Float(1.5));  // Reasonable coverage range
}

TEST(SphericalUtilityTest, EqualAreaSquareToSphereSymmetry) {
    // Test symmetries of the equal area mapping

    // Test that opposite corners map to points with opposite signs in appropriate coordinates
    math::Vector<math::Float, 2> corner_00(0, 0);
    math::Vector<math::Float, 2> corner_11(1, 1);

    auto sphere_00 = equal_area_square_to_sphere(corner_00);
    auto sphere_11 = equal_area_square_to_sphere(corner_11);

    // The mapping should have some reasonable symmetry properties
    // (exact properties depend on the specific equal-area mapping implementation)
    EXPECT_TRUE(are_almost_equal(sphere_00.length(), sphere_11.length(), math::Float(1e-6)));

    // Test center symmetry
    math::Vector<math::Float, 2> center(0.5f, 0.5f);
    auto sphere_center = equal_area_square_to_sphere(center);

    // Center of square should map to a point with reasonable coordinates
    EXPECT_TRUE(std::abs(sphere_center.x()) <= math::Float(1));
    EXPECT_TRUE(std::abs(sphere_center.y()) <= math::Float(1));
    EXPECT_TRUE(std::abs(sphere_center.z()) <= math::Float(1));
}

// ==================== Enhanced wrap_angle_2pi Tests ====================

TEST(SphericalUtilityTest, WrapAngle2PiExtensive) {
    // Test wrap_angle_2pi with more comprehensive cases

    // Test multiple full rotations
    math::Float two_pi = math::Float(2) * math::pi_v<math::Float>;

    // Positive multiple rotations
    EXPECT_TRUE(are_almost_equal(wrap_angle_2pi(math::Float(4) * math::pi_v<math::Float>), math::Float(0), math::Float(1e-6)));
    EXPECT_TRUE(are_almost_equal(wrap_angle_2pi(math::Float(6) * math::pi_v<math::Float>), math::Float(0), math::Float(1e-6)));
    EXPECT_TRUE(are_almost_equal(wrap_angle_2pi(math::Float(5) * math::pi_v<math::Float>), math::pi_v<math::Float>, math::Float(1e-6)));

    // Negative multiple rotations
    EXPECT_TRUE(are_almost_equal(wrap_angle_2pi(math::Float(-4) * math::pi_v<math::Float>), math::Float(0), math::Float(1e-6)));
    EXPECT_TRUE(are_almost_equal(wrap_angle_2pi(math::Float(-3) * math::pi_v<math::Float>), math::pi_v<math::Float>, math::Float(1e-6)));

    // Test with small angles
    EXPECT_TRUE(are_almost_equal(wrap_angle_2pi(math::Float(0.1)), math::Float(0.1), math::Float(1e-10)));
    EXPECT_TRUE(are_almost_equal(wrap_angle_2pi(math::Float(-0.1)), two_pi - math::Float(0.1), math::Float(1e-6)));

    // Test boundary conditions
    EXPECT_TRUE(are_almost_equal(wrap_angle_2pi(two_pi), math::Float(0), math::Float(1e-6)));
    EXPECT_TRUE(are_almost_equal(wrap_angle_2pi(-two_pi), math::Float(0), math::Float(1e-6)));

    // Test that output is always in [0, 2π)
    std::vector<math::Float> test_angles = {math::Float(-10), math::Float(-5.5), math::Float(-1),  math::Float(0),   math::Float(1),
                                      math::Float(5.5), math::Float(10),   math::Float(100), math::Float(-100)};

    for (math::Float angle : test_angles) {
        math::Float wrapped = wrap_angle_2pi(angle);
        EXPECT_GE(wrapped, math::Float(0)) << "Wrapped angle " << wrapped << " from " << angle << " is negative";
        EXPECT_LT(wrapped, two_pi) << "Wrapped angle " << wrapped << " from " << angle << " is >= 2π";
    }
}

TEST(SphericalUtilityTest, WrapAngle2PiConsistency) {
    // Test that wrapping is consistent and idempotent

    std::vector<math::Float> test_angles = {math::Float(-7), math::Float(-3.5), math::Float(0), math::Float(3.5), math::Float(7), math::Float(15)};

    for (math::Float angle : test_angles) {
        math::Float wrapped_once = wrap_angle_2pi(angle);
        math::Float wrapped_twice = wrap_angle_2pi(wrapped_once);

        // Wrapping should be idempotent
        EXPECT_TRUE(are_almost_equal(wrapped_once, wrapped_twice, math::Float(1e-10)))
            << "Idempotent property failed for angle " << angle;

        // Wrapped angle should be equivalent (differ by multiple of 2π)
        math::Float diff = angle - wrapped_once;
        math::Float multiple_of_2pi = std::round(diff / (math::Float(2) * math::pi_v<math::Float>));
        math::Float expected_diff = multiple_of_2pi * math::Float(2) * math::pi_v<math::Float>;

        EXPECT_TRUE(are_almost_equal(diff, expected_diff, math::Float(1e-5)))
            << "Angle " << angle << " and wrapped " << wrapped_once << " don't differ by multiple of 2π";
    }
}

}  // namespace pbpt::geometry::testing
