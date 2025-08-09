#include <gtest/gtest.h>

#include <sstream>
#include <stdexcept>
#include <type_traits>

#include "math/geometry/bounding_box.hpp"

namespace pbpt::math::testing {

// 用于编译时浮点数比较的辅助函数
constexpr bool are_almost_equal(Float a, Float b, Float epsilon = 1e-6) {
    return pbpt::math::abs(a - b) < epsilon;
}

// --- 编译时测试 ---
TEST(BoundingBoxConstexprTest, Construction) {
    constexpr Pt3                   p1(1.0, 2.0, 3.0);
    constexpr BoundingBox<Float, 3> box1(p1);
    static_assert(box1.min() == p1 && box1.max() == p1);

    constexpr BoundingBox<Float, 3> box2;
    static_assert(box2.min().x() == std::numeric_limits<Float>::max());
    static_assert(box2.max().x() == std::numeric_limits<Float>::lowest());
}

TEST(BoundingBoxConstexprTest, UnionOperations) {
    constexpr Pt3                   p1(1.0, 2.0, 3.0);
    constexpr Pt3                   p2(4.0, 5.0, 6.0);
    constexpr BoundingBox<Float, 3> box1(p1);

    constexpr auto box2 = box1.united(p2);
    static_assert(box2.min() == p1);
    static_assert(box2.max() == p2);
}

TEST(BoundingBoxConstexprTest, GeometricProperties) {
    constexpr Pt3                   p1(1.0, 2.0, 3.0);
    constexpr Pt3                   p2(4.0, 6.0, 8.0);
    constexpr BoundingBox<Float, 3> box1(p1);
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
    BoundingBox<Float, 3> box(p1);
    box.unite(p2);

    EXPECT_TRUE(box.contains(Pt3(2.0, 3.0, 4.0)));
    EXPECT_TRUE(box.contains(p1));
    EXPECT_TRUE(box.contains(p2));
    EXPECT_FALSE(box.contains(Pt3(0.0, 2.0, 3.0)));
    EXPECT_FALSE(box.contains(Pt3(5.0, 5.0, 6.0)));
}

TEST(BoundingBoxTest, Overlapping) {
    BoundingBox<Float, 3> box1;
    box1.unite(Pt3(1.0, 1.0, 1.0));
    box1.unite(Pt3(3.0, 3.0, 3.0));

    BoundingBox<Float, 3> box2;
    box2.unite(Pt3(2.0, 2.0, 2.0));
    box2.unite(Pt3(4.0, 4.0, 4.0));

    EXPECT_TRUE(box1.is_overlapped(box2));

    BoundingBox<Float, 3> box3;
    box3.unite(Pt3(5.0, 5.0, 5.0));
    box3.unite(Pt3(6.0, 6.0, 6.0));

    EXPECT_FALSE(box1.is_overlapped(box3));
}

TEST(BoundingBoxTest, SurfaceAreaAndVolume) {
    BoundingBox<Float, 3> box;
    box.unite(Pt3(0.0, 0.0, 0.0));
    box.unite(Pt3(2.0, 3.0, 4.0));

    EXPECT_FLOAT_EQ(box.volume(), 24.0);
    EXPECT_FLOAT_EQ(box.surface_area(), 52.0);
}

TEST(BoundingBoxTest, MaxExtent) {
    BoundingBox<Float, 3> box;
    box.unite(Pt3(0.0, 0.0, 0.0));
    box.unite(Pt3(2.0, 3.0, 4.0));

    EXPECT_EQ(box.max_extent(), 2);  // z轴最长
}

TEST(BoundingBoxTest, Offset) {
    BoundingBox<Float, 3> box;
    box.unite(Pt3(0.0, 0.0, 0.0));
    box.unite(Pt3(2.0, 4.0, 6.0));

    Pt3  p(1.0, 2.0, 3.0);
    Vec3 offset = box.offset(p);

    EXPECT_FLOAT_EQ(offset.x(), 0.5);
    EXPECT_FLOAT_EQ(offset.y(), 0.5);
    EXPECT_FLOAT_EQ(offset.z(), 0.5);
}

}  // namespace pbpt::math::testing