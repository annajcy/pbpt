#include <gtest/gtest.h>

#include <sstream>
#include <stdexcept>
#include <type_traits>
#include <optional>

#include "math/geometry/bounds.hpp"
#include "math/geometry/ray.hpp"

namespace pbpt::math::testing {

// 用于编译时浮点数比较的辅助函数
constexpr bool are_almost_equal(Float a, Float b, Float epsilon = 1e-6) {
    return pbpt::math::abs(a - b) < epsilon;
}

// 用于运行时浮点数比较的辅助函数
bool almost_equal(Float a, Float b, Float epsilon = 1e-6f) {
    return pbpt::math::abs(a - b) < epsilon;
}

// --- 编译时测试 ---
TEST(BoundingBoxConstexprTest, Construction) {
    constexpr Pt3                   p1(1.0, 2.0, 3.0);
    constexpr Bounds<Float, 3> box1(p1);
    static_assert(box1.min() == p1 && box1.max() == p1);

    constexpr Bounds<Float, 3> box2;
    static_assert(box2.min().x() == std::numeric_limits<Float>::max());
    static_assert(box2.max().x() == std::numeric_limits<Float>::lowest());
}

TEST(BoundingBoxConstexprTest, UnionOperations) {
    constexpr Pt3                   p1(1.0, 2.0, 3.0);
    constexpr Pt3                   p2(4.0, 5.0, 6.0);
    constexpr Bounds<Float, 3> box1(p1);

    constexpr auto box2 = box1.united(p2);
    static_assert(box2.min() == p1);
    static_assert(box2.max() == p2);
}

TEST(BoundingBoxConstexprTest, GeometricProperties) {
    constexpr Pt3                   p1(1.0, 2.0, 3.0);
    constexpr Pt3                   p2(4.0, 6.0, 8.0);
    constexpr Bounds<Float, 3> box1(p1);
    constexpr auto                  box2 = box1.united(p2);

    constexpr auto center = box2.center();
    static_assert(are_almost_equal(center.x(), 2.5));
    static_assert(are_almost_equal(center.y(), 4.0));
    static_assert(are_almost_equal(center.z(), 5.5));

    constexpr auto diag = box2.diagonal();
    static_assert(are_almost_equal(diag.x(), 3.0));
    static_assert(are_almost_equal(diag.y(), 4.0));
    static_assert(are_almost_equal(diag.z(), 5.0));
}

// --- 运行时测试 ---
TEST(BoundingBoxTest, ContainsPoint) {
    Pt3                   p1(1.0, 2.0, 3.0);
    Pt3                   p2(4.0, 5.0, 6.0);
    Bounds<Float, 3> box(p1);
    box.unite(p2);

    EXPECT_TRUE(box.contains(Pt3(2.0, 3.0, 4.0)));
    EXPECT_TRUE(box.contains(p1));
    EXPECT_TRUE(box.contains(p2));
    EXPECT_FALSE(box.contains(Pt3(0.0, 2.0, 3.0)));
    EXPECT_FALSE(box.contains(Pt3(5.0, 5.0, 6.0)));
}

TEST(BoundingBoxTest, Overlapping) {
    Bounds<Float, 3> box1;
    box1.unite(Pt3(1.0, 1.0, 1.0));
    box1.unite(Pt3(3.0, 3.0, 3.0));

    Bounds<Float, 3> box2;
    box2.unite(Pt3(2.0, 2.0, 2.0));
    box2.unite(Pt3(4.0, 4.0, 4.0));

    EXPECT_TRUE(box1.is_overlapped(box2));

    Bounds<Float, 3> box3;
    box3.unite(Pt3(5.0, 5.0, 5.0));
    box3.unite(Pt3(6.0, 6.0, 6.0));

    EXPECT_FALSE(box1.is_overlapped(box3));
}

TEST(BoundingBoxTest, SurfaceAreaAndVolume) {
    Bounds<Float, 3> box;
    box.unite(Pt3(0.0, 0.0, 0.0));
    box.unite(Pt3(2.0, 3.0, 4.0));

    EXPECT_FLOAT_EQ(box.volume(), 24.0);
    EXPECT_FLOAT_EQ(box.surface_area(), 52.0);
}

TEST(BoundingBoxTest, MaxExtent) {
    Bounds<Float, 3> box;
    box.unite(Pt3(0.0, 0.0, 0.0));
    box.unite(Pt3(2.0, 3.0, 4.0));

    EXPECT_EQ(box.max_extent(), 2);  // z轴最长
}

TEST(BoundingBoxTest, Offset) {
    Bounds<Float, 3> box;
    box.unite(Pt3(0.0, 0.0, 0.0));
    box.unite(Pt3(2.0, 4.0, 6.0));

    Pt3  p(1.0, 2.0, 3.0);
    Vec3 offset = box.offset(p);

    EXPECT_FLOAT_EQ(offset.x(), 0.5);
    EXPECT_FLOAT_EQ(offset.y(), 0.5);
    EXPECT_FLOAT_EQ(offset.z(), 0.5);
}

// --- 构造函数测试 ---
TEST(BoundingBoxTest, DefaultConstructor) {
    Bounds<Float, 3> box;
    
    // 默认构造应该创建一个"反向"边界框，min为最大值，max为最小值
    EXPECT_EQ(box.min().x(), std::numeric_limits<Float>::max());
    EXPECT_EQ(box.min().y(), std::numeric_limits<Float>::max());
    EXPECT_EQ(box.min().z(), std::numeric_limits<Float>::max());
    EXPECT_EQ(box.max().x(), std::numeric_limits<Float>::lowest());
    EXPECT_EQ(box.max().y(), std::numeric_limits<Float>::lowest());
    EXPECT_EQ(box.max().z(), std::numeric_limits<Float>::lowest());
}

TEST(BoundingBoxTest, SinglePointConstructor) {
    Pt3 p(1.0, 2.0, 3.0);
    Bounds<Float, 3> box(p);
    
    EXPECT_EQ(box.min(), p);
    EXPECT_EQ(box.max(), p);
}

TEST(BoundingBoxTest, MultiplePointsConstructor) {
    Pt3 p1(1.0, 2.0, 3.0);
    Pt3 p2(4.0, 5.0, 6.0);
    Pt3 p3(-1.0, 7.0, 2.0);
    Bounds<Float, 3> box(p1, p2, p3);
    
    EXPECT_EQ(box.min(), Pt3(-1.0, 2.0, 2.0));
    EXPECT_EQ(box.max(), Pt3(4.0, 7.0, 6.0));
}

// --- 边角点测试 ---
TEST(BoundingBoxTest, Corner) {
    Bounds<Float, 3> box;
    box.unite(Pt3(1.0, 2.0, 3.0));
    box.unite(Pt3(4.0, 5.0, 6.0));
    
    // 测试8个角点
    EXPECT_EQ(box.corner(0), Pt3(1.0, 2.0, 3.0));  // 000
    EXPECT_EQ(box.corner(1), Pt3(4.0, 2.0, 3.0));  // 001
    EXPECT_EQ(box.corner(2), Pt3(1.0, 5.0, 3.0));  // 010
    EXPECT_EQ(box.corner(3), Pt3(4.0, 5.0, 3.0));  // 011
    EXPECT_EQ(box.corner(4), Pt3(1.0, 2.0, 6.0));  // 100
    EXPECT_EQ(box.corner(5), Pt3(4.0, 2.0, 6.0));  // 101
    EXPECT_EQ(box.corner(6), Pt3(1.0, 5.0, 6.0));  // 110
    EXPECT_EQ(box.corner(7), Pt3(4.0, 5.0, 6.0));  // 111
}

TEST(BoundingBoxTest, Corner2D) {
    Bounds<Float, 2> box;
    box.unite(Pt2(1.0, 2.0));
    box.unite(Pt2(3.0, 4.0));
    
    // 测试4个角点
    EXPECT_EQ(box.corner(0), Pt2(1.0, 2.0));  // 00
    EXPECT_EQ(box.corner(1), Pt2(3.0, 2.0));  // 01
    EXPECT_EQ(box.corner(2), Pt2(1.0, 4.0));  // 10
    EXPECT_EQ(box.corner(3), Pt2(3.0, 4.0));  // 11
}

// --- 合并操作测试 ---
TEST(BoundingBoxTest, UnitePoint) {
    Bounds<Float, 3> box;
    
    // 首次合并点
    box.unite(Pt3(1.0, 2.0, 3.0));
    EXPECT_EQ(box.min(), Pt3(1.0, 2.0, 3.0));
    EXPECT_EQ(box.max(), Pt3(1.0, 2.0, 3.0));
    
    // 合并更大的点
    box.unite(Pt3(4.0, 5.0, 6.0));
    EXPECT_EQ(box.min(), Pt3(1.0, 2.0, 3.0));
    EXPECT_EQ(box.max(), Pt3(4.0, 5.0, 6.0));
    
    // 合并更小的点
    box.unite(Pt3(-1.0, 0.0, 1.0));
    EXPECT_EQ(box.min(), Pt3(-1.0, 0.0, 1.0));
    EXPECT_EQ(box.max(), Pt3(4.0, 5.0, 6.0));
}

TEST(BoundingBoxTest, UniteBounds) {
    Bounds<Float, 3> box1;
    box1.unite(Pt3(1.0, 2.0, 3.0));
    box1.unite(Pt3(2.0, 3.0, 4.0));
    
    Bounds<Float, 3> box2;
    box2.unite(Pt3(0.0, 1.0, 5.0));
    box2.unite(Pt3(3.0, 4.0, 6.0));
    
    box1.unite(box2);
    EXPECT_EQ(box1.min(), Pt3(0.0, 1.0, 3.0));
    EXPECT_EQ(box1.max(), Pt3(3.0, 4.0, 6.0));
}

TEST(BoundingBoxTest, UnitedPoint) {
    Bounds<Float, 3> box;
    box.unite(Pt3(1.0, 2.0, 3.0));
    box.unite(Pt3(4.0, 5.0, 6.0));
    
    Bounds<Float, 3> new_box = box.united(Pt3(7.0, 8.0, 9.0));
    
    // 原box不变
    EXPECT_EQ(box.min(), Pt3(1.0, 2.0, 3.0));
    EXPECT_EQ(box.max(), Pt3(4.0, 5.0, 6.0));
    
    // 新box包含新点
    EXPECT_EQ(new_box.min(), Pt3(1.0, 2.0, 3.0));
    EXPECT_EQ(new_box.max(), Pt3(7.0, 8.0, 9.0));
}

TEST(BoundingBoxTest, UnitedBounds) {
    Bounds<Float, 3> box1;
    box1.unite(Pt3(1.0, 2.0, 3.0));
    box1.unite(Pt3(2.0, 3.0, 4.0));
    
    Bounds<Float, 3> box2;
    box2.unite(Pt3(0.0, 1.0, 5.0));
    box2.unite(Pt3(3.0, 4.0, 6.0));
    
    Bounds<Float, 3> result = box1.united(box2);
    
    // 原boxes不变
    EXPECT_EQ(box1.min(), Pt3(1.0, 2.0, 3.0));
    EXPECT_EQ(box1.max(), Pt3(2.0, 3.0, 4.0));
    EXPECT_EQ(box2.min(), Pt3(0.0, 1.0, 5.0));
    EXPECT_EQ(box2.max(), Pt3(3.0, 4.0, 6.0));
    
    // 结果包含两个box
    EXPECT_EQ(result.min(), Pt3(0.0, 1.0, 3.0));
    EXPECT_EQ(result.max(), Pt3(3.0, 4.0, 6.0));
}

// --- 重叠测试 ---
TEST(BoundingBoxTest, OverlappingEdgeCases) {
    Bounds<Float, 3> box1;
    box1.unite(Pt3(0.0, 0.0, 0.0));
    box1.unite(Pt3(1.0, 1.0, 1.0));
    
    Bounds<Float, 3> box2;
    box2.unite(Pt3(1.0, 1.0, 1.0));
    box2.unite(Pt3(2.0, 2.0, 2.0));
    
    // 边界接触应该算重叠
    EXPECT_TRUE(box1.is_overlapped(box2));
    
    Bounds<Float, 3> box3;
    box3.unite(Pt3(1.001, 1.001, 1.001));
    box3.unite(Pt3(2.0, 2.0, 2.0));
    
    // 稍微分离就不重叠
    EXPECT_FALSE(box1.is_overlapped(box3));
}

TEST(BoundingBoxTest, OverlappedBox) {
    Bounds<Float, 3> box1;
    box1.unite(Pt3(0.0, 0.0, 0.0));
    box1.unite(Pt3(3.0, 3.0, 3.0));
    
    Bounds<Float, 3> box2;
    box2.unite(Pt3(2.0, 2.0, 2.0));
    box2.unite(Pt3(5.0, 5.0, 5.0));
    
    Bounds<Float, 3> overlap = box1.overlapped_box(box2);
    EXPECT_EQ(overlap.min(), Pt3(2.0, 2.0, 2.0));
    EXPECT_EQ(overlap.max(), Pt3(3.0, 3.0, 3.0));
}

TEST(BoundingBoxTest, OverlappedBoxNoOverlap) {
    Bounds<Float, 3> box1;
    box1.unite(Pt3(0.0, 0.0, 0.0));
    box1.unite(Pt3(1.0, 1.0, 1.0));
    
    Bounds<Float, 3> box2;
    box2.unite(Pt3(2.0, 2.0, 2.0));
    box2.unite(Pt3(3.0, 3.0, 3.0));
    
    Bounds<Float, 3> overlap = box1.overlapped_box(box2);
    // 当没有重叠时，结果应该是无效的（min > max）
    EXPECT_GT(overlap.min().x(), overlap.max().x());
    EXPECT_GT(overlap.min().y(), overlap.max().y());
    EXPECT_GT(overlap.min().z(), overlap.max().z());
}

// --- 包含关系测试 ---
TEST(BoundingBoxTest, ContainsBoundaryPoints) {
    Bounds<Float, 3> box;
    box.unite(Pt3(0.0, 0.0, 0.0));
    box.unite(Pt3(2.0, 2.0, 2.0));
    
    // 边界点应该包含
    EXPECT_TRUE(box.contains(Pt3(0.0, 0.0, 0.0)));
    EXPECT_TRUE(box.contains(Pt3(2.0, 2.0, 2.0)));
    EXPECT_TRUE(box.contains(Pt3(1.0, 0.0, 2.0)));
    EXPECT_TRUE(box.contains(Pt3(0.0, 2.0, 1.0)));
    
    // 超出边界的点不包含
    EXPECT_FALSE(box.contains(Pt3(-0.001, 0.0, 0.0)));
    EXPECT_FALSE(box.contains(Pt3(2.001, 2.0, 2.0)));
    EXPECT_FALSE(box.contains(Pt3(1.0, -0.001, 2.0)));
}

// --- 几何属性测试 ---
TEST(BoundingBoxTest, CenterAndDiagonal) {
    Bounds<Float, 3> box;
    box.unite(Pt3(-1.0, -2.0, -3.0));
    box.unite(Pt3(3.0, 4.0, 5.0));
    
    Pt3 center = box.center();
    EXPECT_FLOAT_EQ(center.x(), 1.0);
    EXPECT_FLOAT_EQ(center.y(), 1.0);
    EXPECT_FLOAT_EQ(center.z(), 1.0);
    
    Vec3 diag = box.diagonal();
    EXPECT_FLOAT_EQ(diag.x(), 4.0);
    EXPECT_FLOAT_EQ(diag.y(), 6.0);
    EXPECT_FLOAT_EQ(diag.z(), 8.0);
}

TEST(BoundingBoxTest, MaxExtentDifferentAxes) {
    // X轴最大
    Bounds<Float, 3> box1;
    box1.unite(Pt3(0.0, 0.0, 0.0));
    box1.unite(Pt3(5.0, 2.0, 3.0));
    EXPECT_EQ(box1.max_extent(), 0);
    
    // Y轴最大
    Bounds<Float, 3> box2;
    box2.unite(Pt3(0.0, 0.0, 0.0));
    box2.unite(Pt3(2.0, 6.0, 3.0));
    EXPECT_EQ(box2.max_extent(), 1);
    
    // Z轴最大
    Bounds<Float, 3> box3;
    box3.unite(Pt3(0.0, 0.0, 0.0));
    box3.unite(Pt3(2.0, 3.0, 7.0));
    EXPECT_EQ(box3.max_extent(), 2);
}

TEST(BoundingBoxTest, VolumeAndSurfaceArea2D) {
    Bounds<Float, 2> box;
    box.unite(Pt2(0.0, 0.0));
    box.unite(Pt2(3.0, 4.0));
    
    EXPECT_FLOAT_EQ(box.volume(), 12.0);  // 面积
}

TEST(BoundingBoxTest, OffsetEdgeCases) {
    Bounds<Float, 3> box;
    box.unite(Pt3(2.0, 4.0, 6.0));
    box.unite(Pt3(8.0, 10.0, 12.0));
    
    // 左下角点
    Vec3 offset1 = box.offset(Pt3(2.0, 4.0, 6.0));
    EXPECT_FLOAT_EQ(offset1.x(), 0.0);
    EXPECT_FLOAT_EQ(offset1.y(), 0.0);
    EXPECT_FLOAT_EQ(offset1.z(), 0.0);
    
    // 右上角点
    Vec3 offset2 = box.offset(Pt3(8.0, 10.0, 12.0));
    EXPECT_FLOAT_EQ(offset2.x(), 1.0);
    EXPECT_FLOAT_EQ(offset2.y(), 1.0);
    EXPECT_FLOAT_EQ(offset2.z(), 1.0);
    
    // 中心点
    Vec3 offset3 = box.offset(Pt3(5.0, 7.0, 9.0));
    EXPECT_FLOAT_EQ(offset3.x(), 0.5);
    EXPECT_FLOAT_EQ(offset3.y(), 0.5);
    EXPECT_FLOAT_EQ(offset3.z(), 0.5);
}

// --- 特殊情况测试 ---
TEST(BoundingBoxTest, EmptyBox) {
    Bounds<Float, 3> box;
    
    // 空盒子的体积应该是负数或零
    EXPECT_LE(box.volume(), 0.0);
}

TEST(BoundingBoxTest, PointBox) {
    Bounds<Float, 3> box;
    box.unite(Pt3(1.0, 2.0, 3.0));
    
    EXPECT_FLOAT_EQ(box.volume(), 0.0);
    EXPECT_FLOAT_EQ(box.surface_area(), 0.0);
    EXPECT_EQ(box.center(), Pt3(1.0, 2.0, 3.0));
    EXPECT_EQ(box.diagonal(), Vec3(0.0, 0.0, 0.0));
}

// --- 输出流测试 ---
TEST(BoundingBoxTest, OutputStreamOperator) {
    Bounds<Float, 3> box;
    box.unite(Pt3(1.0, 2.0, 3.0));
    box.unite(Pt3(4.0, 5.0, 6.0));
    
    std::stringstream ss;
    ss << box;
    
    std::string result = ss.str();
    EXPECT_TRUE(result.find("BoundingBox") != std::string::npos);
    EXPECT_TRUE(result.find("1") != std::string::npos);
    EXPECT_TRUE(result.find("6") != std::string::npos);
}

// --- 不同维度测试 ---
TEST(BoundingBoxTest, OneDimensional) {
    Bounds<Float, 1> box;
    box.unite(Point<Float, 1>(3.0));
    box.unite(Point<Float, 1>(7.0));
    
    EXPECT_EQ(box.min()[0], 3.0);
    EXPECT_EQ(box.max()[0], 7.0);
    EXPECT_FLOAT_EQ(box.volume(), 4.0);
    EXPECT_EQ(box.max_extent(), 0);
}

TEST(BoundingBoxTest, FourDimensional) {
    Bounds<Float, 4> box;
    Point<Float, 4> p1(1.0, 2.0, 3.0, 4.0);
    Point<Float, 4> p2(2.0, 3.0, 4.0, 5.0);
    box.unite(p1);
    box.unite(p2);
    
    EXPECT_EQ(box.min(), p1);
    EXPECT_EQ(box.max(), p2);
    EXPECT_FLOAT_EQ(box.volume(), 1.0);  // 4D体积
}

// --- 类型别名测试 ---
TEST(BoundingBoxTest, TypeAliases) {
    Bounds3 box3;
    box3.unite(Pt3(0.0, 0.0, 0.0));
    box3.unite(Pt3(1.0, 1.0, 1.0));
    EXPECT_FLOAT_EQ(box3.volume(), 1.0);
    
    Bounds2 box2;
    box2.unite(Pt2(0.0, 0.0));
    box2.unite(Pt2(2.0, 3.0));
    EXPECT_FLOAT_EQ(box2.volume(), 6.0);
}

// --- 边界值和精度测试 ---
TEST(BoundingBoxTest, NegativeCoordinates) {
    Bounds<Float, 3> box;
    box.unite(Pt3(-10.0, -20.0, -30.0));
    box.unite(Pt3(-5.0, -15.0, -25.0));
    
    EXPECT_EQ(box.min(), Pt3(-10.0, -20.0, -30.0));
    EXPECT_EQ(box.max(), Pt3(-5.0, -15.0, -25.0));
    EXPECT_FLOAT_EQ(box.volume(), 5.0 * 5.0 * 5.0);
}

TEST(BoundingBoxTest, VerySmallValues) {
    Float epsilon = std::numeric_limits<Float>::epsilon();
    Bounds<Float, 3> box;
    box.unite(Pt3(0.0, 0.0, 0.0));
    box.unite(Pt3(epsilon, epsilon, epsilon));
    
    EXPECT_TRUE(box.contains(Pt3(epsilon/2, epsilon/2, epsilon/2)));
    EXPECT_GT(box.volume(), 0.0);
}

TEST(BoundingBoxTest, VeryLargeValues) {
    Float large = std::numeric_limits<Float>::max() / 1000;  // 避免溢出
    Bounds<Float, 3> box;
    box.unite(Pt3(0.0, 0.0, 0.0));
    box.unite(Pt3(large, large, large));
    
    EXPECT_TRUE(box.contains(Pt3(large/2, large/2, large/2)));
    EXPECT_GT(box.volume(), 0.0);
}

// --- 链式操作测试 ---
TEST(BoundingBoxTest, ChainedUniteOperations) {
    Bounds<Float, 3> box;
    Pt3 p1(1.0, 1.0, 1.0);
    Pt3 p2(2.0, 2.0, 2.0);
    Pt3 p3(3.0, 3.0, 3.0);
    
    // 测试链式调用
    box.unite(p1).unite(p2).unite(p3);
    
    EXPECT_EQ(box.min(), p1);
    EXPECT_EQ(box.max(), p3);
}

TEST(BoundingBoxTest, ChainedUnitedOperations) {
    Bounds<Float, 3> box;
    box.unite(Pt3(1.0, 1.0, 1.0));
    
    Bounds<Float, 3> result = box.united(Pt3(0.0, 0.0, 0.0))
                                 .united(Pt3(2.0, 2.0, 2.0))
                                 .united(Pt3(3.0, 3.0, 3.0));
    
    EXPECT_EQ(result.min(), Pt3(0.0, 0.0, 0.0));
    EXPECT_EQ(result.max(), Pt3(3.0, 3.0, 3.0));
}

// --- 自相交测试 ---
TEST(BoundingBoxTest, SelfOverlap) {
    Bounds<Float, 3> box;
    box.unite(Pt3(1.0, 1.0, 1.0));
    box.unite(Pt3(3.0, 3.0, 3.0));
    
    EXPECT_TRUE(box.is_overlapped(box));
    
    Bounds<Float, 3> overlap = box.overlapped_box(box);
    EXPECT_EQ(overlap.min(), box.min());
    EXPECT_EQ(overlap.max(), box.max());
}

// --- 不规则形状测试 ---
TEST(BoundingBoxTest, IrregularShapes) {
    Bounds<Float, 3> box;
    
    // 添加一系列不规则分布的点
    box.unite(Pt3(1.0, 5.0, 2.0));
    box.unite(Pt3(7.0, 1.0, 8.0));
    box.unite(Pt3(3.0, 9.0, 4.0));
    box.unite(Pt3(6.0, 2.0, 1.0));
    box.unite(Pt3(2.0, 8.0, 7.0));
    
    EXPECT_EQ(box.min(), Pt3(1.0, 1.0, 1.0));
    EXPECT_EQ(box.max(), Pt3(7.0, 9.0, 8.0));
    EXPECT_FLOAT_EQ(box.volume(), 6.0 * 8.0 * 7.0);
}

// --- 退化情况测试 ---
TEST(BoundingBoxTest, DegenerateBoxes) {
    // 在一个轴上退化为线段
    Bounds<Float, 3> line_box;
    line_box.unite(Pt3(1.0, 2.0, 3.0));
    line_box.unite(Pt3(1.0, 2.0, 6.0));
    
    EXPECT_FLOAT_EQ(line_box.volume(), 0.0);
    EXPECT_EQ(line_box.max_extent(), 2);  // Z轴是唯一的非零维度
    
    // 在两个轴上退化为平面
    Bounds<Float, 3> plane_box;
    plane_box.unite(Pt3(1.0, 2.0, 3.0));
    plane_box.unite(Pt3(4.0, 5.0, 3.0));
    
    EXPECT_FLOAT_EQ(plane_box.volume(), 0.0);
    // 表面积应该是两个面的面积之和：2 * (3.0 * 3.0 + 0 + 0) = 18
    EXPECT_FLOAT_EQ(plane_box.surface_area(), 18.0);
}

// --- 多维corner测试 ---
TEST(BoundingBoxTest, CornerHighDimensions) {
    Bounds<Float, 4> box4d;
    Point<Float, 4> p1(1.0, 2.0, 3.0, 4.0);
    Point<Float, 4> p2(5.0, 6.0, 7.0, 8.0);
    box4d.unite(p1);
    box4d.unite(p2);
    
    // 测试一些4D角点
    Point<Float, 4> corner0(1.0, 2.0, 3.0, 4.0);  // 0000
    Point<Float, 4> corner15(5.0, 6.0, 7.0, 8.0); // 1111
    Point<Float, 4> corner5(5.0, 2.0, 7.0, 4.0);  // 0101
    Point<Float, 4> corner10(1.0, 6.0, 3.0, 8.0); // 1010
    
    EXPECT_EQ(box4d.corner(0), corner0);
    EXPECT_EQ(box4d.corner(15), corner15);
    EXPECT_EQ(box4d.corner(5), corner5);
    EXPECT_EQ(box4d.corner(10), corner10);
}

// --- 性能边界测试 ---
TEST(BoundingBoxTest, ManyPointsUnion) {
    Bounds<Float, 3> box;
    
    // 添加大量点
    for (int i = 0; i < 100; ++i) {
        Float x = static_cast<Float>(i % 10);
        Float y = static_cast<Float>((i / 10) % 10);
        Float z = static_cast<Float>(i) / 100.0f;
        box.unite(Pt3(x, y, z));
    }
    
    EXPECT_EQ(box.min(), Pt3(0.0, 0.0, 0.0));
    EXPECT_EQ(box.max(), Pt3(9.0, 9.0, static_cast<Float>(99) / 100.0f));
}

// --- 复合操作测试 ---
TEST(BoundingBoxTest, ComplexOverlapScenarios) {
    // 测试多个盒子的复杂重叠情况
    Bounds<Float, 3> box1, box2, box3;
    
    box1.unite(Pt3(0.0, 0.0, 0.0));
    box1.unite(Pt3(3.0, 3.0, 3.0));
    
    box2.unite(Pt3(2.0, 2.0, 2.0));
    box2.unite(Pt3(5.0, 5.0, 5.0));
    
    box3.unite(Pt3(4.0, 4.0, 4.0));
    box3.unite(Pt3(6.0, 6.0, 6.0));
    
    // box1和box2重叠
    EXPECT_TRUE(box1.is_overlapped(box2));
    
    // box2和box3重叠  
    EXPECT_TRUE(box2.is_overlapped(box3));
    
    // box1和box3不重叠
    EXPECT_FALSE(box1.is_overlapped(box3));
    
    // 测试重叠区域
    Bounds<Float, 3> overlap12 = box1.overlapped_box(box2);
    EXPECT_EQ(overlap12.min(), Pt3(2.0, 2.0, 2.0));
    EXPECT_EQ(overlap12.max(), Pt3(3.0, 3.0, 3.0));
}

// --- 精确度相关测试 ---
TEST(BoundingBoxTest, FloatingPointPrecision) {
    Bounds<Float, 3> box;
    Float val = 1.0f / 3.0f;  // 可能有精度误差的值
    
    box.unite(Pt3(0.0, 0.0, 0.0));
    box.unite(Pt3(val, val, val));
    
    // 测试包含性在精度范围内是否正确
    EXPECT_TRUE(box.contains(Pt3(val/2, val/2, val/2)));
    EXPECT_TRUE(box.contains(Pt3(0.0, 0.0, 0.0)));
    EXPECT_TRUE(box.contains(Pt3(val, val, val)));
}

// --- 光线-边界盒相交测试 ---
TEST(BoundingBoxTest, RayBoundsIntersection_BasicHit) {
    // 创建一个单位立方体
    Bounds<Float, 3> box;
    box.unite(Pt3(0.0, 0.0, 0.0));
    box.unite(Pt3(1.0, 1.0, 1.0));
    
    // 从外部射向盒子的光线
    Ray<Float, 3> ray(Pt3(-1.0, 0.5, 0.5), Vec3(1.0, 0.0, 0.0));
    
    auto result = intersect_ray_bounds(ray, box);
    ASSERT_TRUE(result.has_value());
    
    // 应该在 t=1.0 处进入，t=2.0 处离开
    EXPECT_FLOAT_EQ(result->first, 1.0);
    EXPECT_FLOAT_EQ(result->second, 2.0);
}

TEST(BoundingBoxTest, RayBoundsIntersection_NoHit) {
    // 创建一个单位立方体
    Bounds<Float, 3> box;
    box.unite(Pt3(0.0, 0.0, 0.0));
    box.unite(Pt3(1.0, 1.0, 1.0));
    
    // 错过盒子的光线
    Ray<Float, 3> ray(Pt3(-1.0, 2.0, 0.5), Vec3(1.0, 0.0, 0.0));
    
    auto result = intersect_ray_bounds(ray, box);
    EXPECT_FALSE(result.has_value());
}

TEST(BoundingBoxTest, RayBoundsIntersection_OriginInside) {
    // 创建一个单位立方体
    Bounds<Float, 3> box;
    box.unite(Pt3(0.0, 0.0, 0.0));
    box.unite(Pt3(1.0, 1.0, 1.0));
    
    // 从盒子内部发出的光线
    Ray<Float, 3> ray(Pt3(0.5, 0.5, 0.5), Vec3(1.0, 0.0, 0.0));
    
    auto result = intersect_ray_bounds(ray, box);
    ASSERT_TRUE(result.has_value());
    
    // 当起点在内部时，t_min是负值（表示进入点在光线的"过去"），t_max是正值（出射点）
    EXPECT_LT(result->first, 0.0);  // 进入点在过去
    EXPECT_FLOAT_EQ(result->second, 0.5);  // 在 t=0.5 处离开
}

// --- 更多射线起点在包围盒内的测试用例 ---
TEST(BoundingBoxTest, RayBoundsIntersection_OriginInsideNearBoundary) {
    // 创建一个单位立方体
    Bounds<Float, 3> box;
    box.unite(Pt3(0.0, 0.0, 0.0));
    box.unite(Pt3(1.0, 1.0, 1.0));
    
    // 起点接近边界的光线
    Ray<Float, 3> ray(Pt3(0.99, 0.5, 0.5), Vec3(1.0, 0.0, 0.0));
    
    auto result = intersect_ray_bounds(ray, box);
    ASSERT_TRUE(result.has_value());
    
    // t_min是负值，t_max很小但为正值
    EXPECT_LT(result->first, 0.0);
    EXPECT_NEAR(result->second, 0.01f, 1e-6f);  // 使用NEAR来处理浮点数精度
}

TEST(BoundingBoxTest, RayBoundsIntersection_OriginInsideCorner) {
    // 创建一个单位立方体
    Bounds<Float, 3> box;
    box.unite(Pt3(0.0, 0.0, 0.0));
    box.unite(Pt3(1.0, 1.0, 1.0));
    
    // 从角落发出的光线
    Ray<Float, 3> ray(Pt3(0.1, 0.1, 0.1), Vec3(1.0, 1.0, 1.0).normalized());
    
    auto result = intersect_ray_bounds(ray, box);
    ASSERT_TRUE(result.has_value());
    
    // t_min是负值，t_max是正值
    EXPECT_LT(result->first, 0.0);
    EXPECT_GT(result->second, 0.0);
    
    // 验证出射点在盒子边界上
    Pt3 exit_point = ray.at(result->second);
    EXPECT_TRUE(almost_equal(exit_point.x(), 1.0f) || 
                almost_equal(exit_point.y(), 1.0f) || 
                almost_equal(exit_point.z(), 1.0f));
}

TEST(BoundingBoxTest, RayBoundsIntersection_OriginInsideNegativeDirection) {
    // 创建一个单位立方体
    Bounds<Float, 3> box;
    box.unite(Pt3(0.0, 0.0, 0.0));
    box.unite(Pt3(1.0, 1.0, 1.0));
    
    // 从内部向负方向发出的光线
    Ray<Float, 3> ray(Pt3(0.7, 0.5, 0.5), Vec3(-1.0, 0.0, 0.0));
    
    auto result = intersect_ray_bounds(ray, box);
    ASSERT_TRUE(result.has_value());
    
    // t_min是负值，t_max是正值
    EXPECT_LT(result->first, 0.0);
    EXPECT_FLOAT_EQ(result->second, 0.7f);  // 在 t=0.7 处离开（向左到达x=0）
}

TEST(BoundingBoxTest, RayBoundsIntersection_OriginInsideAllAxes) {
    // 创建一个单位立方体
    Bounds<Float, 3> box;
    box.unite(Pt3(0.0, 0.0, 0.0));
    box.unite(Pt3(1.0, 1.0, 1.0));
    
    Pt3 inside_point(0.3, 0.4, 0.6);
    
    // 测试沿每个轴方向的光线
    struct TestCase {
        Vec3 direction;
        Float expected_t_max;
        std::string name;
    };
    
    std::vector<TestCase> test_cases = {
        {Vec3(1.0, 0.0, 0.0), 0.7f, "+X"},
        {Vec3(-1.0, 0.0, 0.0), 0.3f, "-X"},
        {Vec3(0.0, 1.0, 0.0), 0.6f, "+Y"},
        {Vec3(0.0, -1.0, 0.0), 0.4f, "-Y"},
        {Vec3(0.0, 0.0, 1.0), 0.4f, "+Z"},
        {Vec3(0.0, 0.0, -1.0), 0.6f, "-Z"}
    };
    
    for (const auto& test_case : test_cases) {
        Ray<Float, 3> ray(inside_point, test_case.direction);
        auto result = intersect_ray_bounds(ray, box);
        
        ASSERT_TRUE(result.has_value()) << "Failed for direction: " << test_case.name;
        EXPECT_LT(result->first, 0.0) << "t_min should be negative for direction: " << test_case.name;
        EXPECT_FLOAT_EQ(result->second, test_case.expected_t_max) << "Incorrect t_max for direction: " << test_case.name;
    }
}

TEST(BoundingBoxTest, RayBoundsIntersection_OriginOnBoundary) {
    // 创建一个单位立方体
    Bounds<Float, 3> box;
    box.unite(Pt3(0.0, 0.0, 0.0));
    box.unite(Pt3(1.0, 1.0, 1.0));
    
    // 起点在边界上，向内的光线
    Ray<Float, 3> ray_inward(Pt3(0.0, 0.5, 0.5), Vec3(1.0, 0.0, 0.0));
    auto result_inward = intersect_ray_bounds(ray_inward, box);
    ASSERT_TRUE(result_inward.has_value());
    EXPECT_LE(result_inward->first, 0.0);  // 可能是0或负值，取决于边界处理
    EXPECT_FLOAT_EQ(result_inward->second, 1.0);  // 出射点
    
    // 起点在边界上，向外的光线
    Ray<Float, 3> ray_outward(Pt3(0.0, 0.5, 0.5), Vec3(-1.0, 0.0, 0.0));
    auto result_outward = intersect_ray_bounds(ray_outward, box);
    // 由于边界处理，可能仍然有相交（取决于浮点数精度）
    if (result_outward.has_value()) {
        EXPECT_LE(result_outward->second, 0.0);  // 如果有相交，出射点应该在当前或过去
    }
    
    // 起点在边界上，平行于边界的光线
    Ray<Float, 3> ray_parallel(Pt3(0.0, 0.5, 0.5), Vec3(0.0, 1.0, 0.0));
    auto result_parallel = intersect_ray_bounds(ray_parallel, box);
    ASSERT_TRUE(result_parallel.has_value());
    EXPECT_LT(result_parallel->first, 0.0);
    EXPECT_FLOAT_EQ(result_parallel->second, 0.5);
}

TEST(BoundingBoxTest, RayBoundsIntersection_OriginInsideDiagonal) {
    // 创建一个单位立方体
    Bounds<Float, 3> box;
    box.unite(Pt3(0.0, 0.0, 0.0));
    box.unite(Pt3(1.0, 1.0, 1.0));
    
    // 从内部发出对角线光线
    Ray<Float, 3> ray(Pt3(0.25, 0.25, 0.25), Vec3(1.0, 1.0, 1.0).normalized());
    
    auto result = intersect_ray_bounds(ray, box);
    ASSERT_TRUE(result.has_value());
    EXPECT_LT(result->first, 0.0);  // 进入点在过去
    EXPECT_GT(result->second, 0.0);  // 出射点在未来
    
    // 验证出射点确实在边界上
    Pt3 exit_point = ray.at(result->second);
    bool on_boundary = almost_equal(exit_point.x(), 1.0f) || 
                       almost_equal(exit_point.y(), 1.0f) || 
                       almost_equal(exit_point.z(), 1.0f);
    EXPECT_TRUE(on_boundary);
}

TEST(BoundingBoxTest, RayBoundsIntersection_OriginInside2D) {
    // 2D 情况：矩形内部的光线
    Bounds<Float, 2> box2d;
    box2d.unite(Pt2(0.0, 0.0));
    box2d.unite(Pt2(2.0, 3.0));
    
    // 从矩形内部发出的光线
    Ray<Float, 2> ray2d(Pt2(0.5, 1.0), Vec2(1.0, 0.0));
    
    auto result2d = intersect_ray_bounds(ray2d, box2d);
    ASSERT_TRUE(result2d.has_value());
    EXPECT_LT(result2d->first, 0.0);  // 进入点在过去
    EXPECT_FLOAT_EQ(result2d->second, 1.5); // 从 0.5 到 2.0 的距离是 1.5
}

TEST(BoundingBoxTest, RayBoundsIntersection_OriginInsideZeroDirection) {
    // 创建一个单位立方体
    Bounds<Float, 3> box;
    box.unite(Pt3(0.0, 0.0, 0.0));
    box.unite(Pt3(1.0, 1.0, 1.0));
    
    // 从内部发出但某个方向分量为零的光线
    Ray<Float, 3> ray(Pt3(0.3, 0.5, 0.7), Vec3(0.0, 1.0, 0.0));
    
    auto result = intersect_ray_bounds(ray, box);
    ASSERT_TRUE(result.has_value());
    EXPECT_LT(result->first, 0.0);  // 进入点在过去
    EXPECT_FLOAT_EQ(result->second, 0.5); // 从 0.5 到 1.0 的距离是 0.5
}

TEST(BoundingBoxTest, RayBoundsIntersection_OriginInsideOffsetBox) {
    // 偏移的盒子，起点在内部
    Bounds<Float, 3> box;
    box.unite(Pt3(10.0, 20.0, 30.0));
    box.unite(Pt3(13.0, 24.0, 35.0));
    
    Ray<Float, 3> ray(Pt3(11.5, 22.0, 32.5), Vec3(1.0, 0.0, 0.0));
    
    auto result = intersect_ray_bounds(ray, box);
    ASSERT_TRUE(result.has_value());
    EXPECT_LT(result->first, 0.0);  // 进入点在过去
    EXPECT_FLOAT_EQ(result->second, 1.5); // 从 11.5 到 13.0 的距离是 1.5
    
    // 验证相交点
    Pt3 exit_point = ray.at(result->second);
    EXPECT_FLOAT_EQ(exit_point.x(), 13.0);
    EXPECT_FLOAT_EQ(exit_point.y(), 22.0);
    EXPECT_FLOAT_EQ(exit_point.z(), 32.5);
}

// --- 额外测试：验证 t=0 时刻的点确实是起点 ---
TEST(BoundingBoxTest, RayBoundsIntersection_OriginInsideVerifyRayPoints) {
    // 创建一个单位立方体
    Bounds<Float, 3> box;
    box.unite(Pt3(0.0, 0.0, 0.0));
    box.unite(Pt3(1.0, 1.0, 1.0));
    
    Pt3 origin(0.3, 0.4, 0.6);
    Ray<Float, 3> ray(origin, Vec3(1.0, 0.0, 0.0));
    
    auto result = intersect_ray_bounds(ray, box);
    ASSERT_TRUE(result.has_value());
    
    // 验证 t=0 时的点确实是起点
    Pt3 point_at_zero = ray.at(0.0);
    EXPECT_EQ(point_at_zero, origin);
    
    // 验证进入点在盒子边界上（t_min < 0）
    Pt3 entry_point = ray.at(result->first);
    EXPECT_TRUE(box.contains(entry_point) || almost_equal(entry_point.x(), 0.0f));
    
    // 验证出射点在盒子边界上
    Pt3 exit_point = ray.at(result->second);
    EXPECT_TRUE(almost_equal(exit_point.x(), 1.0f));
    EXPECT_FLOAT_EQ(exit_point.y(), 0.4);
    EXPECT_FLOAT_EQ(exit_point.z(), 0.6);
}

TEST(BoundingBoxTest, RayBoundsIntersection_ParallelToFace) {
    // 创建一个单位立方体
    Bounds<Float, 3> box;
    box.unite(Pt3(0.0, 0.0, 0.0));
    box.unite(Pt3(1.0, 1.0, 1.0));
    
    // 平行于盒子表面但不相交的光线
    Ray<Float, 3> ray(Pt3(-1.0, 2.0, 0.5), Vec3(1.0, 0.0, 0.0));
    
    auto result = intersect_ray_bounds(ray, box);
    EXPECT_FALSE(result.has_value());
    
    // 平行于盒子表面且在表面上的光线
    Ray<Float, 3> ray2(Pt3(-1.0, 0.0, 0.5), Vec3(1.0, 0.0, 0.0));
    
    auto result2 = intersect_ray_bounds(ray2, box);
    ASSERT_TRUE(result2.has_value());
    EXPECT_FLOAT_EQ(result2->first, 1.0);
    EXPECT_FLOAT_EQ(result2->second, 2.0);
}

TEST(BoundingBoxTest, RayBoundsIntersection_ZeroDirection) {
    // 创建一个单位立方体
    Bounds<Float, 3> box;
    box.unite(Pt3(0.0, 0.0, 0.0));
    box.unite(Pt3(1.0, 1.0, 1.0));
    
    // 在某个轴上方向为零的光线（在盒子内部）
    Ray<Float, 3> ray(Pt3(0.5, 0.5, 0.5), Vec3(0.0, 1.0, 0.0));
    
    auto result = intersect_ray_bounds(ray, box);
    ASSERT_TRUE(result.has_value());
    
    // 在某个轴上方向为零的光线（在盒子外部）
    Ray<Float, 3> ray2(Pt3(2.0, 0.5, 0.5), Vec3(0.0, 1.0, 0.0));
    
    auto result2 = intersect_ray_bounds(ray2, box);
    EXPECT_FALSE(result2.has_value());
}

TEST(BoundingBoxTest, RayBoundsIntersection_DiagonalRay) {
    // 创建一个单位立方体
    Bounds<Float, 3> box;
    box.unite(Pt3(0.0, 0.0, 0.0));
    box.unite(Pt3(1.0, 1.0, 1.0));
    
    // 对角线光线
    Ray<Float, 3> ray(Pt3(-1.0, -1.0, -1.0), Vec3(1.0, 1.0, 1.0).normalized());
    
    auto result = intersect_ray_bounds(ray, box);
    ASSERT_TRUE(result.has_value());
    
    // 验证相交点在盒子边界上
    Pt3 entry_point = ray.at(result->first);
    Pt3 exit_point = ray.at(result->second);
    
    EXPECT_TRUE(box.contains(entry_point));
    EXPECT_TRUE(box.contains(exit_point));
}

TEST(BoundingBoxTest, RayBoundsIntersection_NegativeDirection) {
    // 创建一个单位立方体
    Bounds<Float, 3> box;
    box.unite(Pt3(0.0, 0.0, 0.0));
    box.unite(Pt3(1.0, 1.0, 1.0));
    
    // 负方向的光线
    Ray<Float, 3> ray(Pt3(2.0, 0.5, 0.5), Vec3(-1.0, 0.0, 0.0));
    
    auto result = intersect_ray_bounds(ray, box);
    ASSERT_TRUE(result.has_value());
    
    // 应该在 t=1.0 处进入，t=2.0 处离开
    EXPECT_FLOAT_EQ(result->first, 1.0);
    EXPECT_FLOAT_EQ(result->second, 2.0);
}

TEST(BoundingBoxTest, RayBoundsIntersection_2D) {
    // 2D 测试
    Bounds<Float, 2> box2d;
    box2d.unite(Pt2(0.0, 0.0));
    box2d.unite(Pt2(1.0, 1.0));
    
    Ray<Float, 2> ray2d(Pt2(-0.5, 0.5), Vec2(1.0, 0.0));
    
    auto result2d = intersect_ray_bounds(ray2d, box2d);
    ASSERT_TRUE(result2d.has_value());
    
    EXPECT_FLOAT_EQ(result2d->first, 0.5);
    EXPECT_FLOAT_EQ(result2d->second, 1.5);
}

TEST(BoundingBoxTest, RayBoundsIntersection_EdgeCases) {
    // 创建一个单位立方体
    Bounds<Float, 3> box;
    box.unite(Pt3(0.0, 0.0, 0.0));
    box.unite(Pt3(1.0, 1.0, 1.0));
    
    // 光线刚好擦过盒子的边
    Ray<Float, 3> ray_edge(Pt3(-1.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0));
    auto result_edge = intersect_ray_bounds(ray_edge, box);
    ASSERT_TRUE(result_edge.has_value());
    
    // 光线刚好擦过盒子的角
    Ray<Float, 3> ray_corner(Pt3(-1.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0));
    auto result_corner = intersect_ray_bounds(ray_corner, box);
    ASSERT_TRUE(result_corner.has_value());
}

TEST(BoundingBoxTest, RayBoundsIntersection_VerySmallBox) {
    // 非常小的盒子
    Float epsilon = 1e-6f;
    Bounds<Float, 3> small_box;
    small_box.unite(Pt3(0.0, 0.0, 0.0));
    small_box.unite(Pt3(epsilon, epsilon, epsilon));
    
    Ray<Float, 3> ray(Pt3(-epsilon, epsilon/2, epsilon/2), Vec3(1.0, 0.0, 0.0));
    
    auto result = intersect_ray_bounds(ray, small_box);
    ASSERT_TRUE(result.has_value());
    
    EXPECT_GT(result->first, 0.0);
    EXPECT_GT(result->second, result->first);
}

TEST(BoundingBoxTest, RayBoundsIntersection_OffsetBox) {
    // 偏移的盒子
    Bounds<Float, 3> box;
    box.unite(Pt3(10.0, 20.0, 30.0));
    box.unite(Pt3(11.0, 21.0, 31.0));
    
    Ray<Float, 3> ray(Pt3(9.0, 20.5, 30.5), Vec3(1.0, 0.0, 0.0));
    
    auto result = intersect_ray_bounds(ray, box);
    ASSERT_TRUE(result.has_value());
    
    // 验证相交点正确
    Pt3 entry = ray.at(result->first);
    Pt3 exit = ray.at(result->second);
    
    EXPECT_TRUE(box.contains(entry));
    EXPECT_TRUE(box.contains(exit));
}

}  // namespace pbpt::math::testing