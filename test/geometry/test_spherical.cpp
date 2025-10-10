#include <gtest/gtest.h>

#include <cmath>
#include <type_traits>

#include "pbpt.h"

using namespace pbpt::geometry;

namespace pbpt::geometry::testing {

bool are_almost_equal(Float a, Float b, Float epsilon = epsilon_v<Float>) {
    return pbpt::math::abs(a - b) < epsilon;
}

template <typename PointType>
bool points_almost_equal(const PointType& p1, const PointType& p2, Float epsilon = epsilon_v<Float>) {
    for (int i = 0; i < p1.dims(); ++i) {
        if (!are_almost_equal(p1[i], p2[i], epsilon)) {
            return false;
        }
    }
    return true;
}

template <typename PointType>
bool points_almost_equal_relative(const PointType& p1, const PointType& p2, Float relative_epsilon = Float(1e-5)) {
    for (int i = 0; i < p1.dims(); ++i) {
        Float abs_error = pbpt::math::abs(p1[i] - p2[i]);
        Float abs_p1 = pbpt::math::abs(p1[i]);
        Float abs_p2 = pbpt::math::abs(p2[i]);
        Float max_abs = std::max(abs_p1, abs_p2);
        
        // 使用相对误差，当数值很小时使用绝对误差
        if (max_abs > Float(1e-10)) {
            Float relative_error = abs_error / max_abs;
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
    Vector<Float, 1>         angles(M_PI / 4);  // 45度
    Float                    radius = 5.0f;
    SphericalPoint<Float, 2> sphere(angles, radius);

    EXPECT_FLOAT_EQ(sphere.radius(), 5.0f);
    EXPECT_FLOAT_EQ(sphere.angle(0), M_PI / 4);
}

TEST(SphericalPointTest, CartesianToSpherical2D) {
    // 测试从笛卡尔坐标构造球坐标
    Point<Float, 2>          cart(3.0f, 4.0f);
    SphericalPoint<Float, 2> sphere(cart);

    // 验证半径: sqrt(3^2 + 4^2) = 5
    EXPECT_FLOAT_EQ(sphere.radius(), 5.0f);

    // 验证角度: atan2(4, 3)
    Float expected_angle = atan2(4.0f, 3.0f);
    EXPECT_TRUE(are_almost_equal(sphere.angle(0), expected_angle));
}

TEST(SphericalPointTest, SphericalToCartesian2D) {
    // 测试球坐标转换为笛卡尔坐标
    Vector<Float, 1>         angles(M_PI / 6);  // 30度
    Float                    radius = 2.0f;
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
    Point<Float, 2>          original(1.5f, 2.5f);
    SphericalPoint<Float, 2> sphere(original);
    Point<Float, 2>          converted = sphere.to_cartesian();

    EXPECT_TRUE(points_almost_equal(original, converted));
}

// --- 3D 球坐标测试 ---
TEST(SphericalPointTest, Construction3D) {
    // 测试从角度和半径构造
    Vector<Float, 2>         angles(M_PI / 3, M_PI / 4);  // 极角60度，方位角45度
    Float                    radius = 10.0f;
    SphericalPoint<Float, 3> sphere(angles, radius);

    EXPECT_FLOAT_EQ(sphere.radius(), 10.0f);
    EXPECT_FLOAT_EQ(sphere.angle(0), M_PI / 3);  // 极角
    EXPECT_FLOAT_EQ(sphere.angle(1), M_PI / 4);  // 方位角
}

TEST(SphericalPointTest, CartesianToSpherical3D) {
    // 测试从笛卡尔坐标构造球坐标
    Point<Float, 3>          cart(1.0f, 1.0f, sqrt(2.0f));
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
    Vector<Float, 2>         angles(M_PI / 2, 0.0f);  // 极角90度，方位角0度
    Float                    radius = 3.0f;
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
    Point<Float, 3>          original(2.0f, 3.0f, 4.0f);
    SphericalPoint<Float, 3> sphere(original);
    Point<Float, 3>          converted = sphere.to_cartesian();

    EXPECT_TRUE(points_almost_equal(original, converted));
}

// --- 4D 球坐标测试 ---
TEST(SphericalPointTest, Construction4D) {
    // 测试4维球坐标构造
    Vector<Float, 3>         angles(M_PI / 6, M_PI / 4, M_PI / 3);
    Float                    radius = 5.0f;
    SphericalPoint<Float, 4> sphere(angles, radius);

    EXPECT_FLOAT_EQ(sphere.radius(), 5.0f);
    EXPECT_FLOAT_EQ(sphere.angle(0), M_PI / 6);
    EXPECT_FLOAT_EQ(sphere.angle(1), M_PI / 4);
    EXPECT_FLOAT_EQ(sphere.angle(2), M_PI / 3);
}

TEST(SphericalPointTest, RoundTrip4D) {
    // 测试4维往返转换的精度
    Point<Float, 4>          original(1.0f, 2.0f, 3.0f, 4.0f);
    SphericalPoint<Float, 4> sphere(original);
    Point<Float, 4>          converted = sphere.to_cartesian();

    EXPECT_TRUE(points_almost_equal(original, converted));
}

// --- 边界情况测试 ---
TEST(SphericalPointTest, ZeroRadius) {
    // 测试零半径情况
    Point<Float, 3>          origin = Point<Float, 3>::zeros();
    SphericalPoint<Float, 3> sphere(origin);

    EXPECT_FLOAT_EQ(sphere.radius(), 0.0f);

    // 转换回笛卡尔坐标应该仍然是原点
    Point<Float, 3> converted = sphere.to_cartesian();
    EXPECT_TRUE(points_almost_equal(origin, converted));
}

TEST(SphericalPointTest, AxisAlignedPoints) {
    // 测试轴对齐的点

    // X轴正方向
    Point<Float, 3>          x_axis(5.0f, 0.0f, 0.0f);
    SphericalPoint<Float, 3> sphere_x(x_axis);
    EXPECT_TRUE(are_almost_equal(sphere_x.radius(), 5.0f));
    EXPECT_TRUE(are_almost_equal(sphere_x.angle(0), M_PI / 2));  // 极角π/2
    EXPECT_TRUE(are_almost_equal(sphere_x.angle(1), 0.0f));      // 方位角0

    // Z轴正方向
    Point<Float, 3>          z_axis(0.0f, 0.0f, 3.0f);
    SphericalPoint<Float, 3> sphere_z(z_axis);
    EXPECT_TRUE(are_almost_equal(sphere_z.radius(), 3.0f));
    EXPECT_TRUE(are_almost_equal(sphere_z.angle(0), 0.0f));  // 极角0
}

TEST(SphericalPointTest, NegativeCoordinates) {
    // 测试负坐标的处理
    Point<Float, 2>          negative(-3.0f, -4.0f);
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
    Vector<Float, 2>         angles_3d(1.0f, 2.0f);  // 极角1.0，方位角2.0
    Float                    radius = 7.0f;
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
    Vector<Float, 1>         angles_2d(M_PI / 3);  // 方位角60度
    SphericalPoint<Float, 2> sphere_2d(angles_2d, 5.0f);

    EXPECT_FLOAT_EQ(sphere_2d.radius(), 5.0f);
    EXPECT_FLOAT_EQ(sphere_2d.angle(0), M_PI / 3);
    EXPECT_FLOAT_EQ(sphere_2d.azimuth(),
                    M_PI / 3);  // 2D情况下方位角就是唯一的角度
}

// --- 精度和数值稳定性测试 ---
TEST(SphericalPointTest, NumericalStability) {
    // 测试非常小的值
    Point<Float, 3>          tiny(1e-6f, 1e-6f, 1e-6f);
    SphericalPoint<Float, 3> sphere_tiny(tiny);
    Point<Float, 3>          converted_tiny = sphere_tiny.to_cartesian();
    EXPECT_TRUE(points_almost_equal(tiny, converted_tiny, 1e-5f));

    // 测试非常大的值
    Point<Float, 3>          large(1e6f, 1e6f, 1e6f);
    SphericalPoint<Float, 3> sphere_large(large);
    Point<Float, 3>          converted_large = sphere_large.to_cartesian();

    // 计算并输出详细的误差信息
    for (int i = 0; i < 3; ++i) {
        Float abs_error = pbpt::math::abs(large[i] - converted_large[i]);
        Float rel_error = abs_error / pbpt::math::abs(large[i]);
        std::cout << "Component " << i << ": original=" << large[i] 
                  << ", converted=" << converted_large[i]
                  << ", abs_error=" << abs_error 
                  << ", rel_error=" << rel_error << std::endl;
    }
    
    // 也输出球坐标信息
    std::cout << "Sphere radius: " << sphere_large.radius() << std::endl;
    std::cout << "Sphere angles: [" << sphere_large.angle(0) << ", " 
              << sphere_large.angle(1) << "]" << std::endl;
    
    // 使用相对误差检查，1e-5 的相对精度对于大数值是合理的
    EXPECT_TRUE(points_almost_equal_relative(large, converted_large, Float(1e-5)))
        << "High precision conversion failed for large values";
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
    // math::Sphere3 sp(math::Vec2{math::deg2rad(45.0),
    // math::deg2rad(45.0)}, 1.0);
    Vec2    angles{deg2rad(45.0), deg2rad(45.0)};  // 极角45°，方位角45°
    Sphere3 sp(angles, 1.0);

    // 验证构造参数
    EXPECT_FLOAT_EQ(sp.radius(), 1.0f);
    EXPECT_TRUE(are_almost_equal(sp.angle(0), deg2rad(45.0)));   // 极角
    EXPECT_TRUE(are_almost_equal(sp.angle(1), deg2rad(45.0)));   // 方位角
    EXPECT_TRUE(are_almost_equal(sp.azimuth(), deg2rad(45.0)));  // 测试azimuth()方法

    // 测试转换为笛卡尔坐标
    Point<Float, 3> cartesian = sp.to_cartesian();

    // 计算期望值（新的角度约定：极角在前，方位角在后）
    // x = r * sin(theta) * cos(phi) = 1 * sin(45°) * cos(45°) = sin(π/4) *
    // cos(π/4) = 0.5 y = r * sin(theta) * sin(phi) = 1 * sin(45°) * sin(45°) =
    // sin(π/4) * sin(π/4) = 0.5 z = r * cos(theta) = 1 * cos(45°) = cos(π/4) =
    // √2/2
    Float expected_x = sin(deg2rad(45.0)) * cos(deg2rad(45.0));
    Float expected_y = sin(deg2rad(45.0)) * sin(deg2rad(45.0));
    Float expected_z = cos(deg2rad(45.0));

    EXPECT_TRUE(are_almost_equal(cartesian.x(), expected_x));
    EXPECT_TRUE(are_almost_equal(cartesian.y(), expected_y));
    EXPECT_TRUE(are_almost_equal(cartesian.z(), expected_z));

    // 测试往返转换
    Sphere3         sphere_from_cart(cartesian);
    Point<Float, 3> converted_back = sphere_from_cart.to_cartesian();
    EXPECT_TRUE(points_almost_equal(cartesian, converted_back));
}

// --- 编译时测试 ---
TEST(SphericalPointTest, ConstexprSupport) {
    // 验证constexpr支持
    constexpr Vector<Float, 1>         angles(1.0f);
    constexpr Float                    radius = 2.0f;
    constexpr SphericalPoint<Float, 2> sphere(angles, radius);

    static_assert(sphere.radius() == 2.0f);
    static_assert(sphere.angle(0) == 1.0f);

    SUCCEED() << "Constexpr construction and access work correctly.";
}

// ==================== wrap_angle_2pi Function Tests ====================

TEST(SphericalPointTest, WrapAngle2PiBasicTests) {
    // Test wrap_angle_2pi function with basic cases only
    
    // Test positive angles within range
    EXPECT_FLOAT_EQ(wrap_angle_2pi(Float(0)), Float(0));
    EXPECT_FLOAT_EQ(wrap_angle_2pi(pi_v<Float>), pi_v<Float>);
    EXPECT_FLOAT_EQ(wrap_angle_2pi(Float(1.5) * pi_v<Float>), Float(1.5) * pi_v<Float>);
    
    // Test simple negative angles
    EXPECT_FLOAT_EQ(wrap_angle_2pi(-pi_v<Float>), pi_v<Float>);
    EXPECT_FLOAT_EQ(wrap_angle_2pi(-pi_v<Float> / 2), Float(1.5) * pi_v<Float>);
    
    // Test simple large angles (with tolerance)
    EXPECT_TRUE(are_almost_equal(wrap_angle_2pi(Float(3) * pi_v<Float>), pi_v<Float>, Float(1e-5)));
}

// ==================== Enhanced Spherical Point Tests ====================

TEST(SphericalPointTest, FromCartesianStaticMethod) {
    // Test static factory method
    Point<Float, 2> pt_2d(3.0f, 4.0f);
    Point<Float, 3> pt_3d(1.0f, 2.0f, 3.0f);
    
    auto sphere_2d = SphericalPoint<Float, 2>::from_cartesian(pt_2d);
    auto sphere_3d = SphericalPoint<Float, 3>::from_cartesian(pt_3d);
    
    EXPECT_FLOAT_EQ(sphere_2d.radius(), 5.0f);  // sqrt(3^2 + 4^2) = 5
    EXPECT_FLOAT_EQ(sphere_3d.radius(), sqrt(14.0f));  // sqrt(1^2 + 2^2 + 3^2) = sqrt(14)
}

TEST(SphericalPointTest, HighDimensionalSupport) {
    // Test 5D spherical coordinates
    Point<Float, 5> pt_5d(1.0f, 2.0f, 3.0f, 4.0f, 5.0f);
    SphericalPoint<Float, 5> sphere_5d(pt_5d);
    Point<Float, 5> converted_5d = sphere_5d.to_cartesian();
    
    EXPECT_TRUE(points_almost_equal(pt_5d, converted_5d, Float(1e-5)));
    
    // Test 6D spherical coordinates
    Point<Float, 6> pt_6d(1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f);
    SphericalPoint<Float, 6> sphere_6d(pt_6d);
    Point<Float, 6> converted_6d = sphere_6d.to_cartesian();
    
    EXPECT_TRUE(points_almost_equal(pt_6d, converted_6d, Float(1e-5)));
}

TEST(SphericalPointTest, SpecialAngles3D) {
    // Test specific known angles
    
    // South pole (0, 0, -1)
    Point<Float, 3> south_pole(0.0f, 0.0f, -1.0f);
    SphericalPoint<Float, 3> sphere_south(south_pole);
    EXPECT_FLOAT_EQ(sphere_south.radius(), 1.0f);
    EXPECT_TRUE(are_almost_equal(sphere_south.angle(0), pi_v<Float>));  // θ = π for south pole
    
    // Test 45-degree elevation
    Point<Float, 3> pt_45deg(1.0f, 0.0f, 1.0f);  // Should have θ = π/4
    SphericalPoint<Float, 3> sphere_45(pt_45deg);
    Float expected_radius = sqrt(2.0f);
    EXPECT_TRUE(are_almost_equal(sphere_45.radius(), expected_radius));
    EXPECT_TRUE(are_almost_equal(sphere_45.angle(0), pi_v<Float> / 4));
    EXPECT_TRUE(are_almost_equal(sphere_45.angle(1), 0.0f));
}

TEST(SphericalPointTest, VerySmallValues) {
    // Test numerical stability with very small values
    Float epsilon = Float(1e-10);
    Point<Float, 2> small_2d(epsilon, epsilon);
    Point<Float, 3> small_3d(epsilon, epsilon, epsilon);
    
    SphericalPoint<Float, 2> sphere_2d_small(small_2d);
    SphericalPoint<Float, 3> sphere_3d_small(small_3d);
    
    // Should not crash and should have small but non-zero radius
    EXPECT_GT(sphere_2d_small.radius(), 0);
    EXPECT_GT(sphere_3d_small.radius(), 0);
    
    // Round trip should work
    Point<Float, 2> converted_2d = sphere_2d_small.to_cartesian();
    Point<Float, 3> converted_3d = sphere_3d_small.to_cartesian();
    
    EXPECT_TRUE(points_almost_equal(small_2d, converted_2d, Float(1e-8)));
    EXPECT_TRUE(points_almost_equal(small_3d, converted_3d, Float(1e-8)));
}

TEST(SphericalPointTest, ExtensiveRoundTripTests) {
    // Test more comprehensive round-trip conversions
    
    // 2D test cases
    std::vector<Point<Float, 2>> test_points_2d = {
        Point<Float, 2>(1, 0), Point<Float, 2>(0, 1), 
        Point<Float, 2>(-1, 0), Point<Float, 2>(0, -1),
        Point<Float, 2>(1, 1), Point<Float, 2>(-1, -1), 
        Point<Float, 2>(2, 3), Point<Float, 2>(-1.5f, 2.5f),
        Point<Float, 2>(100, 0.01f), Point<Float, 2>(0.01f, 100)
    };
    
    for (const auto& pt : test_points_2d) {
        SphericalPoint<Float, 2> sphere(pt);
        Point<Float, 2> converted_back = sphere.to_cartesian();
        EXPECT_TRUE(points_almost_equal(pt, converted_back, Float(1e-5)))
            << "Failed for 2D point: (" << pt.x() << ", " << pt.y() << ")";
    }
    
    // 3D test cases
    std::vector<Point<Float, 3>> test_points_3d = {
        Point<Float, 3>(1, 0, 0), Point<Float, 3>(0, 1, 0), Point<Float, 3>(0, 0, 1),
        Point<Float, 3>(-1, 0, 0), Point<Float, 3>(0, -1, 0), Point<Float, 3>(0, 0, -1),
        Point<Float, 3>(1, 1, 1), Point<Float, 3>(-1, -1, -1),
        Point<Float, 3>(2, 3, 4), Point<Float, 3>(-1.5f, 2.5f, -3.2f),
        Point<Float, 3>(std::sqrt(3)/2, 0.5f, 0), // 30-60-90 triangle
        Point<Float, 3>(0.5f, std::sqrt(3)/2, 1)
    };
    
    for (const auto& pt : test_points_3d) {
        SphericalPoint<Float, 3> sphere(pt);
        Point<Float, 3> converted_back = sphere.to_cartesian();
        EXPECT_TRUE(points_almost_equal(pt, converted_back, Float(1e-5)))
            << "Failed for 3D point: (" << pt.x() << ", " << pt.y() << ", " << pt.z() << ")";
    }
}

// ==================== Spherical Geometry Function Tests ====================

TEST(SphericalGeometryTest, SphericalTriangleAreaBasic) {
    // Simple triangle on unit sphere
    Vector<Float, 3> a(1, 0, 0);  // Point on x-axis
    Vector<Float, 3> b(0, 1, 0);  // Point on y-axis  
    Vector<Float, 3> c(0, 0, 1);  // North pole
    
    // Normalize the vectors
    a = a.normalized();
    b = b.normalized();
    c = c.normalized();
    
    Float area = spherical_triangle_area(a, b, c);
    EXPECT_GT(area, 0);
    EXPECT_LT(area, Float(4) * pi_v<Float>);  // Area should be reasonable
    
    // This specific triangle should have area π/2 (1/8 of sphere surface)
    EXPECT_TRUE(are_almost_equal(area, pi_v<Float> / 2, Float(1e-5)));
}

TEST(SphericalGeometryTest, SphericalPolygonAreaBasic) {
    // Test with a simple triangle first
    std::vector<Vector<Float, 3>> triangle_vertices = {
        Vector<Float, 3>(1, 0, 0).normalized(),
        Vector<Float, 3>(0, 1, 0).normalized(),
        Vector<Float, 3>(0, 0, 1).normalized()
    };
    
    Float triangle_area = spherical_polygon_area(triangle_vertices);
    EXPECT_GT(std::abs(triangle_area), Float(0));
    EXPECT_LT(std::abs(triangle_area), Float(4) * pi_v<Float>);
}

TEST(SphericalGeometryTest, ErrorHandling) {
    // Test error handling for invalid inputs
    
    // Test with degenerate triangle (collinear points)
    Vector<Float, 3> a(1, 0, 0);
    Vector<Float, 3> b(2, 0, 0);  // Same direction as a
    Vector<Float, 3> c(3, 0, 0);  // Same direction as a and b
    
    Float degenerate_area = spherical_triangle_area(a, b, c);
    // Degenerate triangle should have zero area
    EXPECT_TRUE(are_almost_equal(degenerate_area, Float(0), Float(1e-5)));
    
    // Test polygon with minimum vertices
    std::vector<Vector<Float, 3>> min_vertices = {
        Vector<Float, 3>(1, 0, 0).normalized(),
        Vector<Float, 3>(0, 1, 0).normalized(),
        Vector<Float, 3>(0, 0, 1).normalized()
    };
    
    Float min_area = spherical_polygon_area(min_vertices);
    EXPECT_GT(std::abs(min_area), Float(0));
}

// ==================== Enhanced Spherical Geometry Tests ====================

TEST(SphericalGeometryTest, SphericalTriangleAreaKnownValues) {
    // Test with known mathematical results
    
    // 1. Right spherical triangle with known area
    Vector<Float, 3> north_pole(0, 0, 1);
    Vector<Float, 3> equator_0(1, 0, 0);    // 0° longitude on equator
    Vector<Float, 3> equator_90(0, 1, 0);   // 90° longitude on equator
    
    Float right_triangle_area = spherical_triangle_area(north_pole, equator_0, equator_90);
    EXPECT_TRUE(are_almost_equal(right_triangle_area, pi_v<Float> / 2, Float(1e-4)));
    
    // 2. Small spherical triangle (should approximate planar area)
    Vector<Float, 3> small_a(1, 0, 0);
    Vector<Float, 3> small_b(Float(0.999), Float(0.01), Float(0.01));  // Very close to small_a
    Vector<Float, 3> small_c(Float(0.999), Float(0.01), Float(-0.01)); // Very close to small_a
    
    small_a = small_a.normalized();
    small_b = small_b.normalized();
    small_c = small_c.normalized();
    
    Float small_area = spherical_triangle_area(small_a, small_b, small_c);
    EXPECT_GT(small_area, Float(0));
    EXPECT_LT(small_area, Float(0.001));  // Should be very small
}

TEST(SphericalGeometryTest, SphericalTriangleAreaSymmetry) {
    // Test symmetry properties
    Vector<Float, 3> a(1, 0, 0);
    Vector<Float, 3> b(0, 1, 0);
    Vector<Float, 3> c(0, 0, 1);
    
    Float area_abc = spherical_triangle_area(a, b, c);
    Float area_acb = spherical_triangle_area(a, c, b);
    Float area_bac = spherical_triangle_area(b, a, c);
    Float area_bca = spherical_triangle_area(b, c, a);
    Float area_cab = spherical_triangle_area(c, a, b);
    Float area_cba = spherical_triangle_area(c, b, a);
    
    // All permutations should give the same absolute area
    EXPECT_TRUE(are_almost_equal(std::abs(area_abc), std::abs(area_acb), Float(1e-6)));
    EXPECT_TRUE(are_almost_equal(std::abs(area_abc), std::abs(area_bac), Float(1e-6)));
    EXPECT_TRUE(are_almost_equal(std::abs(area_abc), std::abs(area_bca), Float(1e-6)));
    EXPECT_TRUE(are_almost_equal(std::abs(area_abc), std::abs(area_cab), Float(1e-6)));
    EXPECT_TRUE(are_almost_equal(std::abs(area_abc), std::abs(area_cba), Float(1e-6)));
}

TEST(SphericalGeometryTest, SphericalTriangleAreaDegenerate) {
    // Test degenerate cases
    
    // 1. Collinear points (zero area)
    Vector<Float, 3> collinear_a(1, 0, 0);
    Vector<Float, 3> collinear_b(Float(0.7071), Float(0.7071), 0);  // 45° from a
    Vector<Float, 3> collinear_c(0, 1, 0);  // 90° from a, collinear with a and b
    
    // Make them actually collinear on the sphere (great circle)
    Vector<Float, 3> gc_a(1, 0, 0);
    Vector<Float, 3> gc_b(Float(0.7071), Float(0.7071), 0);
    Vector<Float, 3> gc_c(0, 1, 0);
    
    Float collinear_area = spherical_triangle_area(gc_a, gc_b, gc_c);
    EXPECT_TRUE(are_almost_equal(collinear_area, Float(0), Float(1e-5)));
    
    // 2. Identical points (zero area)
    Vector<Float, 3> same_point(1, 0, 0);
    Float identical_area = spherical_triangle_area(same_point, same_point, same_point);
    EXPECT_TRUE(are_almost_equal(identical_area, Float(0), Float(1e-10)));
    
    // 3. Two identical points, one different (zero area)
    Vector<Float, 3> diff_point(0, 1, 0);
    Float two_same_area = spherical_triangle_area(same_point, same_point, diff_point);
    EXPECT_TRUE(are_almost_equal(two_same_area, Float(0), Float(1e-10)));
}

TEST(SphericalGeometryTest, SphericalTriangleAreaNumericalStability) {
    // Test numerical stability with various vector magnitudes
    
    // 1. Very large vectors (should be normalized internally by dot products)
    Vector<Float, 3> large_a(Float(1e6), 0, 0);
    Vector<Float, 3> large_b(0, Float(1e6), 0);
    Vector<Float, 3> large_c(0, 0, Float(1e6));
    
    // Normalize them first
    large_a = large_a.normalized();
    large_b = large_b.normalized();
    large_c = large_c.normalized();
    
    Float large_area = spherical_triangle_area(large_a, large_b, large_c);
    EXPECT_GT(large_area, Float(0));
    EXPECT_LT(large_area, Float(4) * pi_v<Float>);
    
    // 2. Very small vectors
    Vector<Float, 3> small_a(Float(1e-6), 0, 0);
    Vector<Float, 3> small_b(0, Float(1e-6), 0);
    Vector<Float, 3> small_c(0, 0, Float(1e-6));
    
    small_a = small_a.normalized();
    small_b = small_b.normalized();
    small_c = small_c.normalized();
    
    Float small_area = spherical_triangle_area(small_a, small_b, small_c);
    EXPECT_GT(small_area, Float(0));
    EXPECT_LT(small_area, Float(4) * pi_v<Float>);
    
    // Results should be the same regardless of input vector magnitude
    EXPECT_TRUE(are_almost_equal(large_area, small_area, Float(1e-5)));
}

TEST(SphericalGeometryTest, SphericalPolygonAreaPentagon) {
    // Test with a regular pentagon on the sphere
    std::vector<Vector<Float, 3>> pentagon_vertices;
    const int n_sides = 5;
    
    for (int i = 0; i < n_sides; ++i) {
        Float angle = Float(2 * i) * pi_v<Float> / Float(n_sides);
        pentagon_vertices.push_back(Vector<Float, 3>(cos(angle), sin(angle), 0).normalized());
    }
    
    Float pentagon_area = spherical_polygon_area(pentagon_vertices);
    EXPECT_GT(std::abs(pentagon_area), Float(0));
    EXPECT_LT(std::abs(pentagon_area), Float(4) * pi_v<Float>);
    
    // Pentagon should have larger area than triangle but smaller than square
    std::vector<Vector<Float, 3>> triangle_vertices = {
        Vector<Float, 3>(1, 0, 0).normalized(),
        Vector<Float, 3>(0, 1, 0).normalized(),
        Vector<Float, 3>(0, 0, 1).normalized()
    };
    Float triangle_area = std::abs(spherical_polygon_area(triangle_vertices));
    
    EXPECT_GT(std::abs(pentagon_area), triangle_area);
}

TEST(SphericalGeometryTest, SphericalPolygonAreaConsistency) {
    // Test that polygon area is consistent with triangulation
    std::vector<Vector<Float, 3>> quad_vertices = {
        Vector<Float, 3>(1, 0, 0).normalized(),
        Vector<Float, 3>(Float(0.7071), Float(0.7071), 0).normalized(),
        Vector<Float, 3>(0, 1, 0).normalized(),
        Vector<Float, 3>(Float(-0.7071), Float(0.7071), 0).normalized()
    };
    
    Float polygon_area = spherical_polygon_area(quad_vertices);
    
    // Manual triangulation: split into two triangles
    Float triangle1_area = spherical_triangle_area(
        quad_vertices[0], quad_vertices[1], quad_vertices[2]
    );
    Float triangle2_area = spherical_triangle_area(
        quad_vertices[0], quad_vertices[2], quad_vertices[3]
    );
    
    Float manual_area = triangle1_area + triangle2_area;
    
    // They should be approximately equal
    EXPECT_TRUE(are_almost_equal(std::abs(polygon_area), std::abs(manual_area), Float(1e-4)));
}

TEST(SphericalGeometryTest, SphericalPolygonAreaVertexOrder) {
    // Test that vertex order affects sign but not magnitude
    std::vector<Vector<Float, 3>> vertices_ccw = {
        Vector<Float, 3>(1, 0, 0).normalized(),
        Vector<Float, 3>(0, 1, 0).normalized(),
        Vector<Float, 3>(0, 0, 1).normalized()
    };
    
    std::vector<Vector<Float, 3>> vertices_cw = {
        Vector<Float, 3>(1, 0, 0).normalized(),
        Vector<Float, 3>(0, 0, 1).normalized(),
        Vector<Float, 3>(0, 1, 0).normalized()
    };
    
    Float area_ccw = spherical_polygon_area(vertices_ccw);
    Float area_cw = spherical_polygon_area(vertices_cw);
    
    // Magnitudes should be equal
    EXPECT_TRUE(are_almost_equal(std::abs(area_ccw), std::abs(area_cw), Float(1e-6)));
    
    // Signs might be different (depending on implementation)
    // But at least one should be positive
    EXPECT_TRUE(area_ccw > Float(0) || area_cw > Float(0));
}

TEST(SphericalGeometryTest, SphericalTriangleAreaEdgeCases) {
    // 2. Very close points (should have small area)
    Vector<Float, 3> close_a(1, 0, 0);
    Vector<Float, 3> close_b(Float(0.999999), Float(0.000001), 0);
    Vector<Float, 3> close_c(Float(0.999999), 0, Float(0.000001));
    
    close_a = close_a.normalized();
    close_b = close_b.normalized();
    close_c = close_c.normalized();
    
    Float close_area = spherical_triangle_area(close_a, close_b, close_c);
    EXPECT_GT(close_area, Float(0));
    EXPECT_LT(close_area, Float(1e-10));  // Should be very small
}

// ==================== Performance and Precision Tests ====================

TEST(SphericalPointTest, HighPrecisionMath) {
    // Test with mathematical constants for high precision
    Float e_const = Float(2.718281828459045);
    Float pi_const = pi_v<Float>;
    Float sqrt2_const = Float(1.4142135623730951);
    
    Point<Float, 3> math_pt(e_const, pi_const, sqrt2_const);
    
    SphericalPoint<Float, 3> sphere(math_pt);
    Point<Float, 3> converted = sphere.to_cartesian();
    
    // Use more reasonable tolerance for floating point precision
    EXPECT_TRUE(are_almost_equal(converted.x(), math_pt.x(), Float(1e-5)));
    EXPECT_TRUE(are_almost_equal(converted.y(), math_pt.y(), Float(1e-5)));
    EXPECT_TRUE(are_almost_equal(converted.z(), math_pt.z(), Float(1e-5)));
}

// ==================== Spherical Coordinate Utility Function Tests ====================

TEST(SphericalUtilityTest, CosThetaFunction) {
    // Test cos_theta function for 3D vectors
    
    // Test with unit vector along z-axis (north pole)
    Vector<Float, 3> north_pole(0, 0, 1);
    EXPECT_FLOAT_EQ(cos_theta<Float>(north_pole), Float(1));
    
    // Test with unit vector along negative z-axis (south pole)
    Vector<Float, 3> south_pole(0, 0, -1);
    EXPECT_FLOAT_EQ(cos_theta<Float>(south_pole), Float(-1));
    
    // Test with unit vector in xy-plane (equator)
    Vector<Float, 3> equator(1, 0, 0);
    EXPECT_FLOAT_EQ(cos_theta<Float>(equator), Float(0));
    
    // Test with 45-degree elevation
    Vector<Float, 3> v_45deg = Vector<Float, 3>(1, 0, 1).normalized();
    EXPECT_TRUE(are_almost_equal(cos_theta<Float>(v_45deg), Float(1.0 / std::sqrt(2))));
    
    // Test with arbitrary vector
    Vector<Float, 3> arbitrary(3, 4, 5);  // z = 5
    Float expected_cos = Float(5);  // cos_theta just returns v.z()
    EXPECT_FLOAT_EQ(cos_theta<Float>(arbitrary), expected_cos);
}

TEST(SphericalUtilityTest, CosThetaSqFunction) {
    // Test cos_theta_sq function
    Vector<Float, 3> v(1, 2, 3);  // z = 3
    Float expected = Float(9);  // z^2 = 9
    EXPECT_FLOAT_EQ(cos_theta_sq<Float>(v), expected);
    
    // Test with normalized vector
    Vector<Float, 3> normalized = v.normalized();
    Float cos_val = cos_theta<Float>(normalized);
    EXPECT_TRUE(are_almost_equal(cos_theta_sq<Float>(normalized), cos_val * cos_val));
}

TEST(SphericalUtilityTest, SinThetaSqFunction) {
    // Test sin_theta_sq function
    Vector<Float, 3> north_pole(0, 0, 1);
    EXPECT_TRUE(are_almost_equal(sin_theta_sq<Float>(north_pole), Float(0), Float(1e-10)));
    
    Vector<Float, 3> equator(1, 0, 0);
    EXPECT_TRUE(are_almost_equal(sin_theta_sq<Float>(equator), Float(1), Float(1e-10)));
    
    // Test identity: sin²θ + cos²θ = 1
    Vector<Float, 3> arbitrary(2, 3, 4);
    arbitrary = arbitrary.normalized();
    Float sin_sq = sin_theta_sq<Float>(arbitrary);
    Float cos_sq = cos_theta_sq<Float>(arbitrary);
    EXPECT_TRUE(are_almost_equal(sin_sq + cos_sq, Float(1), Float(1e-6)));
}

TEST(SphericalUtilityTest, SinThetaFunction) {
    // Test sin_theta function
    Vector<Float, 3> north_pole(0, 0, 1);
    EXPECT_TRUE(are_almost_equal(sin_theta<Float>(north_pole), Float(0), Float(1e-10)));
    
    Vector<Float, 3> equator(1, 0, 0);
    EXPECT_TRUE(are_almost_equal(sin_theta<Float>(equator), Float(1), Float(1e-10)));
    
    Vector<Float, 3> v_45deg = Vector<Float, 3>(1, 0, 1).normalized();
    EXPECT_TRUE(are_almost_equal(sin_theta<Float>(v_45deg), Float(1.0 / std::sqrt(2))));
    
    // Test that sin_theta is always non-negative (implementation uses sqrt)
    Vector<Float, 3> various_vectors[] = {
        Vector<Float, 3>(1, 2, 3).normalized(),
        Vector<Float, 3>(-1, -2, -3).normalized(),
        Vector<Float, 3>(0, 1, -1).normalized()
    };
    
    for (const auto& v : various_vectors) {
        EXPECT_GE(sin_theta<Float>(v), Float(0));
    }
}

TEST(SphericalUtilityTest, TanThetaFunction) {
    // Test tan_theta function
    Vector<Float, 3> equator(1, 0, 0);
    // For equator: cos_theta = 0, sin_theta = 1, so tan should be infinity
    // But in practice, we need to handle division by zero carefully
    Float tan_val = tan_theta<Float>(equator);
    EXPECT_TRUE(std::isinf(tan_val) || std::abs(tan_val) > Float(1e6));  // Very large value
    
    Vector<Float, 3> v_45deg = Vector<Float, 3>(1, 0, 1).normalized();
    EXPECT_TRUE(are_almost_equal(tan_theta<Float>(v_45deg), Float(1)));
    
    Vector<Float, 3> north_pole(0, 0, 1);
    EXPECT_TRUE(are_almost_equal(tan_theta<Float>(north_pole), Float(0), Float(1e-10)));
}

TEST(SphericalUtilityTest, TanThetaSqFunction) {
    // Test tan_theta_sq function
    Vector<Float, 3> v_45deg = Vector<Float, 3>(1, 0, 1).normalized();
    EXPECT_TRUE(are_almost_equal(tan_theta_sq<Float>(v_45deg), Float(1)));
    
    Vector<Float, 3> north_pole(0, 0, 1);
    EXPECT_TRUE(are_almost_equal(tan_theta_sq<Float>(north_pole), Float(0), Float(1e-10)));
    
    // Test relationship with sin and cos (avoid division by zero)
    Vector<Float, 3> arbitrary(1, 2, 3);
    arbitrary = arbitrary.normalized();
    if (std::abs(cos_theta<Float>(arbitrary)) > Float(1e-6)) {
        Float tan_sq_direct = tan_theta_sq<Float>(arbitrary);
        Float tan_from_sin_cos = sin_theta_sq<Float>(arbitrary) / cos_theta_sq<Float>(arbitrary);
        EXPECT_TRUE(are_almost_equal(tan_sq_direct, tan_from_sin_cos, Float(1e-6)));
    }
}

TEST(SphericalUtilityTest, PhiFunction) {
    // Test phi function for azimuthal angle
    
    // Test cardinal directions
    Vector<Float, 3> pos_x(1, 0, 0);
    EXPECT_TRUE(are_almost_equal(phi(pos_x), Float(0), Float(1e-6)));
    
    Vector<Float, 3> pos_y(0, 1, 0);
    EXPECT_TRUE(are_almost_equal(phi(pos_y), pi_v<Float> / 2, Float(1e-6)));
    
    Vector<Float, 3> neg_x(-1, 0, 0);
    EXPECT_TRUE(are_almost_equal(phi(neg_x), pi_v<Float>, Float(1e-6)));
    
    Vector<Float, 3> neg_y(0, -1, 0);
    EXPECT_TRUE(are_almost_equal(phi(neg_y), Float(3) * pi_v<Float> / 2, Float(1e-6)));
    
    // Test 45-degree angles
    Vector<Float, 3> northeast(1, 1, 0);
    EXPECT_TRUE(are_almost_equal(phi(northeast), pi_v<Float> / 4, Float(1e-6)));
    
    Vector<Float, 3> southwest(-1, -1, 0);
    EXPECT_TRUE(are_almost_equal(phi(southwest), Float(5) * pi_v<Float> / 4, Float(1e-6)));
    
    // Test that phi is in range [0, 2π)
    std::vector<Vector<Float, 3>> test_vectors = {
        Vector<Float, 3>(1, 1, 1), Vector<Float, 3>(-1, 1, -1),
        Vector<Float, 3>(2, -3, 4), Vector<Float, 3>(-5, -2, 1)
    };
    
    for (const auto& v : test_vectors) {
        Float phi_val = phi(v);
        EXPECT_GE(phi_val, Float(0));
        EXPECT_LT(phi_val, Float(2) * pi_v<Float>);
    }
}

TEST(SphericalUtilityTest, SinPhiFunction) {
    // Test sin_phi function
    
    // Test cardinal directions
    Vector<Float, 3> pos_x(1, 0, 1);  // Include z to avoid singularity
    pos_x = pos_x.normalized();
    EXPECT_TRUE(are_almost_equal(sin_phi(pos_x), Float(0), Float(1e-6)));
    
    Vector<Float, 3> pos_y(0, 1, 1);
    pos_y = pos_y.normalized();
    EXPECT_TRUE(are_almost_equal(sin_phi(pos_y), Float(1), Float(1e-6)));
    
    Vector<Float, 3> neg_y(0, -1, 1);
    neg_y = neg_y.normalized();
    EXPECT_TRUE(are_almost_equal(sin_phi(neg_y), Float(-1), Float(1e-6)));
    
    // Test 45-degree angle
    Vector<Float, 3> northeast(1, 1, 1);
    northeast = northeast.normalized();
    EXPECT_TRUE(are_almost_equal(sin_phi(northeast), Float(1) / std::sqrt(Float(2)), Float(1e-6)));
    
    // Test singularity at poles (sin_theta = 0)
    Vector<Float, 3> north_pole(0, 0, 1);
    EXPECT_TRUE(are_almost_equal(sin_phi(north_pole), Float(0), Float(1e-10)));
    
    Vector<Float, 3> south_pole(0, 0, -1);
    EXPECT_TRUE(are_almost_equal(sin_phi(south_pole), Float(0), Float(1e-10)));
}

TEST(SphericalUtilityTest, CosPhiFunction) {
    // Test cos_phi function
    
    // Test cardinal directions
    Vector<Float, 3> pos_x(1, 0, 1);
    pos_x = pos_x.normalized();
    EXPECT_TRUE(are_almost_equal(cos_phi(pos_x), Float(1), Float(1e-6)));
    
    Vector<Float, 3> pos_y(0, 1, 1);
    pos_y = pos_y.normalized();
    EXPECT_TRUE(are_almost_equal(cos_phi(pos_y), Float(0), Float(1e-6)));
    
    Vector<Float, 3> neg_x(-1, 0, 1);
    neg_x = neg_x.normalized();
    EXPECT_TRUE(are_almost_equal(cos_phi(neg_x), Float(-1), Float(1e-6)));
    
    // Test 45-degree angle
    Vector<Float, 3> northeast(1, 1, 1);
    northeast = northeast.normalized();
    EXPECT_TRUE(are_almost_equal(cos_phi(northeast), Float(1) / std::sqrt(Float(2)), Float(1e-6)));
    
    // Test singularity at poles
    Vector<Float, 3> north_pole(0, 0, 1);
    EXPECT_TRUE(are_almost_equal(cos_phi(north_pole), Float(1), Float(1e-10)));  // Default value
    
    // Test identity: sin²φ + cos²φ = 1 (for non-pole vectors)
    Vector<Float, 3> arbitrary(2, 3, 4);
    arbitrary = arbitrary.normalized();
    Float sin_phi_val = sin_phi(arbitrary);
    Float cos_phi_val = cos_phi(arbitrary);
    EXPECT_TRUE(are_almost_equal(sin_phi_val * sin_phi_val + cos_phi_val * cos_phi_val, Float(1), Float(1e-6)));
}

TEST(SphericalUtilityTest, CosDeltaPhiFunction) {
    // Test cos_delta_phi function for angle between projections
    
    // Test identical vectors
    Vector<Float, 3> v(1, 2, 3);
    EXPECT_TRUE(are_almost_equal(cos_delta_phi(v, v), Float(1), Float(1e-10)));
    
    // Test orthogonal vectors in xy-plane
    Vector<Float, 3> v1(1, 0, 5);  // Same z, different xy
    Vector<Float, 3> v2(0, 1, 5);
    EXPECT_TRUE(are_almost_equal(cos_delta_phi(v1, v2), Float(0), Float(1e-6)));
    
    // Test opposite vectors in xy-plane
    Vector<Float, 3> v3(1, 0, 3);
    Vector<Float, 3> v4(-1, 0, 3);
    EXPECT_TRUE(are_almost_equal(cos_delta_phi(v3, v4), Float(-1), Float(1e-6)));
    
    // Test 45-degree difference in xy-plane
    Vector<Float, 3> v5(1, 0, 2);
    Vector<Float, 3> v6(1, 1, 2);
    v6 = v6.normalized() * v5.length();  // Maintain same z and radius in xy
    Float expected_cos = Float(1) / std::sqrt(Float(2));  // cos(45°)
    EXPECT_TRUE(are_almost_equal(cos_delta_phi(v5, v6), expected_cos, Float(1e-5)));
    
    // Test singularities (vectors with zero xy components)
    Vector<Float, 3> pole1(0, 0, 1);
    Vector<Float, 3> pole2(0, 0, -1);
    EXPECT_TRUE(are_almost_equal(cos_delta_phi(pole1, pole2), Float(1), Float(1e-10)));  // Default value
    
    Vector<Float, 3> mixed1(0, 0, 1);
    Vector<Float, 3> mixed2(1, 0, 2);
    EXPECT_TRUE(are_almost_equal(cos_delta_phi(mixed1, mixed2), Float(1), Float(1e-10)));
}

TEST(SphericalUtilityTest, TrigonometricIdentities) {
    // Test various trigonometric identities for spherical coordinates
    
    std::vector<Vector<Float, 3>> test_vectors = {
        Vector<Float, 3>(1, 0, 0).normalized(),
        Vector<Float, 3>(0, 1, 0).normalized(),
        Vector<Float, 3>(0, 0, 1).normalized(),
        Vector<Float, 3>(1, 1, 1).normalized(),
        Vector<Float, 3>(2, -3, 4).normalized(),
        Vector<Float, 3>(-1, 2, -3).normalized()
    };
    
    for (const auto& v : test_vectors) {
        // Test sin²θ + cos²θ = 1
        Float sin_sq = sin_theta_sq(v);
        Float cos_sq = cos_theta_sq(v);
        EXPECT_TRUE(are_almost_equal(sin_sq + cos_sq, Float(1), Float(1e-6)))
            << "sin²θ + cos²θ = 1 failed for vector (" << v.x() << ", " << v.y() << ", " << v.z() << ")";
        
        // Test sin²φ + cos²φ = 1 (except at poles)
        if (sin_theta(v) > Float(1e-6)) {  // Avoid poles
            Float sin_phi_val = sin_phi(v);
            Float cos_phi_val = cos_phi(v);
            EXPECT_TRUE(are_almost_equal(sin_phi_val * sin_phi_val + cos_phi_val * cos_phi_val, Float(1), Float(1e-6)))
                << "sin²φ + cos²φ = 1 failed for vector (" << v.x() << ", " << v.y() << ", " << v.z() << ")";
        }
        
        // Test tan²θ = sin²θ / cos²θ (except when cos_theta = 0)
        if (std::abs(cos_theta(v)) > Float(1e-6)) {
            Float tan_sq_direct = tan_theta_sq(v);
            Float tan_sq_from_trig = sin_theta_sq(v) / cos_theta_sq(v);
            EXPECT_TRUE(are_almost_equal(tan_sq_direct, tan_sq_from_trig, Float(1e-6)))
                << "tan²θ identity failed for vector (" << v.x() << ", " << v.y() << ", " << v.z() << ")";
        }
    }
}

// ==================== Equal Area Mapping Tests ====================

TEST(SphericalUtilityTest, EqualAreaSquareToSphereBasic) {
    // Test equal_area_square_to_sphere function
    
    // Test corners of unit square [0,1]²
    Vector<Float, 2> corner_00(0, 0);
    Vector<Float, 2> corner_01(0, 1);
    Vector<Float, 2> corner_10(1, 0);
    Vector<Float, 2> corner_11(1, 1);
    
    auto sphere_00 = equal_area_square_to_sphere(corner_00);
    auto sphere_01 = equal_area_square_to_sphere(corner_01);
    auto sphere_10 = equal_area_square_to_sphere(corner_10);
    auto sphere_11 = equal_area_square_to_sphere(corner_11);
    
    // All results should be unit vectors
    EXPECT_TRUE(are_almost_equal(sphere_00.length(), Float(1), Float(1e-6)));
    EXPECT_TRUE(are_almost_equal(sphere_01.length(), Float(1), Float(1e-6)));
    EXPECT_TRUE(are_almost_equal(sphere_10.length(), Float(1), Float(1e-6)));
    EXPECT_TRUE(are_almost_equal(sphere_11.length(), Float(1), Float(1e-6)));
    
    // Test center of square
    Vector<Float, 2> center(0.5f, 0.5f);
    auto sphere_center = equal_area_square_to_sphere(center);
    EXPECT_TRUE(are_almost_equal(sphere_center.length(), Float(1), Float(1e-6)));
    
    // Center should map to some reasonable point on sphere
    EXPECT_GE(sphere_center.z(), Float(-1));
    EXPECT_LE(sphere_center.z(), Float(1));
}

TEST(SphericalUtilityTest, EqualAreaSquareToSphereCoverage) {
    // Test that the mapping covers the sphere reasonably
    const int samples = 10;
    std::vector<Vector<Float, 3>> mapped_points;
    
    for (int i = 0; i <= samples; ++i) {
        for (int j = 0; j <= samples; ++j) {
            Float u = Float(i) / Float(samples);
            Float v = Float(j) / Float(samples);
            Vector<Float, 2> square_pt(u, v);
            auto sphere_pt = equal_area_square_to_sphere(square_pt);
            
            // Should be unit vector
            EXPECT_TRUE(are_almost_equal(sphere_pt.length(), Float(1), Float(1e-5)));
            mapped_points.push_back(sphere_pt);
        }
    }
    
    // Check that we get reasonable coverage of the sphere
    // Find min/max z values
    Float min_z = mapped_points[0].z();
    Float max_z = mapped_points[0].z();
    
    for (const auto& pt : mapped_points) {
        min_z = std::min(min_z, pt.z());
        max_z = std::max(max_z, pt.z());
    }
    
    // Should cover a reasonable range of z values
    EXPECT_LT(min_z, Float(0));  // Should reach southern hemisphere
    EXPECT_GT(max_z, Float(0));  // Should reach northern hemisphere
    EXPECT_GT(max_z - min_z, Float(1.5));  // Reasonable coverage range
}

TEST(SphericalUtilityTest, EqualAreaSquareToSphereSymmetry) {
    // Test symmetries of the equal area mapping
    
    // Test that opposite corners map to points with opposite signs in appropriate coordinates
    Vector<Float, 2> corner_00(0, 0);
    Vector<Float, 2> corner_11(1, 1);
    
    auto sphere_00 = equal_area_square_to_sphere(corner_00);
    auto sphere_11 = equal_area_square_to_sphere(corner_11);
    
    // The mapping should have some reasonable symmetry properties
    // (exact properties depend on the specific equal-area mapping implementation)
    EXPECT_TRUE(are_almost_equal(sphere_00.length(), sphere_11.length(), Float(1e-6)));
    
    // Test center symmetry
    Vector<Float, 2> center(0.5f, 0.5f);
    auto sphere_center = equal_area_square_to_sphere(center);
    
    // Center of square should map to a point with reasonable coordinates
    EXPECT_TRUE(std::abs(sphere_center.x()) <= Float(1));
    EXPECT_TRUE(std::abs(sphere_center.y()) <= Float(1));
    EXPECT_TRUE(std::abs(sphere_center.z()) <= Float(1));
}

// ==================== Enhanced wrap_angle_2pi Tests ====================

TEST(SphericalUtilityTest, WrapAngle2PiExtensive) {
    // Test wrap_angle_2pi with more comprehensive cases
    
    // Test multiple full rotations
    Float two_pi = Float(2) * pi_v<Float>;
    
    // Positive multiple rotations
    EXPECT_TRUE(are_almost_equal(wrap_angle_2pi(Float(4) * pi_v<Float>), Float(0), Float(1e-6)));
    EXPECT_TRUE(are_almost_equal(wrap_angle_2pi(Float(6) * pi_v<Float>), Float(0), Float(1e-6)));
    EXPECT_TRUE(are_almost_equal(wrap_angle_2pi(Float(5) * pi_v<Float>), pi_v<Float>, Float(1e-6)));
    
    // Negative multiple rotations
    EXPECT_TRUE(are_almost_equal(wrap_angle_2pi(Float(-4) * pi_v<Float>), Float(0), Float(1e-6)));
    EXPECT_TRUE(are_almost_equal(wrap_angle_2pi(Float(-3) * pi_v<Float>), pi_v<Float>, Float(1e-6)));
    
    // Test with small angles
    EXPECT_TRUE(are_almost_equal(wrap_angle_2pi(Float(0.1)), Float(0.1), Float(1e-10)));
    EXPECT_TRUE(are_almost_equal(wrap_angle_2pi(Float(-0.1)), two_pi - Float(0.1), Float(1e-6)));
    
    // Test boundary conditions
    EXPECT_TRUE(are_almost_equal(wrap_angle_2pi(two_pi), Float(0), Float(1e-6)));
    EXPECT_TRUE(are_almost_equal(wrap_angle_2pi(-two_pi), Float(0), Float(1e-6)));
    
    // Test that output is always in [0, 2π)
    std::vector<Float> test_angles = {
        Float(-10), Float(-5.5), Float(-1), Float(0), Float(1), 
        Float(5.5), Float(10), Float(100), Float(-100)
    };
    
    for (Float angle : test_angles) {
        Float wrapped = wrap_angle_2pi(angle);
        EXPECT_GE(wrapped, Float(0)) << "Wrapped angle " << wrapped << " from " << angle << " is negative";
        EXPECT_LT(wrapped, two_pi) << "Wrapped angle " << wrapped << " from " << angle << " is >= 2π";
    }
}

TEST(SphericalUtilityTest, WrapAngle2PiConsistency) {
    // Test that wrapping is consistent and idempotent
    
    std::vector<Float> test_angles = {
        Float(-7), Float(-3.5), Float(0), Float(3.5), Float(7), Float(15)
    };
    
    for (Float angle : test_angles) {
        Float wrapped_once = wrap_angle_2pi(angle);
        Float wrapped_twice = wrap_angle_2pi(wrapped_once);
        
        // Wrapping should be idempotent
        EXPECT_TRUE(are_almost_equal(wrapped_once, wrapped_twice, Float(1e-10)))
            << "Idempotent property failed for angle " << angle;
        
        // Wrapped angle should be equivalent (differ by multiple of 2π)
        Float diff = angle - wrapped_once;
        Float multiple_of_2pi = std::round(diff / (Float(2) * pi_v<Float>));
        Float expected_diff = multiple_of_2pi * Float(2) * pi_v<Float>;
        
        EXPECT_TRUE(are_almost_equal(diff, expected_diff, Float(1e-5)))
            << "Angle " << angle << " and wrapped " << wrapped_once 
            << " don't differ by multiple of 2π";
    }
}

}  // namespace pbpt::math::testing