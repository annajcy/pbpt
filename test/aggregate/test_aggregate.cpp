#include <gtest/gtest.h>

#include "aggregate/plugin/aggregate/linear_aggregate.hpp"
#include "geometry/ray.hpp"
#include "geometry/transform.hpp"
#include "shape/primitive.hpp"
#include "shape/plugin/shape/sphere.hpp"
namespace pbpt::aggregate::testing {

using pbpt::geometry::Ray;
using pbpt::geometry::Transform;
using pbpt::math::Point;
using pbpt::math::Vector;
using pbpt::shape::Primitive;
using pbpt::shape::Sphere;

using T = float;

namespace {
Sphere<T> make_sphere(const Vector<T, 3>& translate) {
    return Sphere<T>(Transform<T>::translate(translate), false, T(1));
}
}  // namespace

TEST(PrimitiveTest, CarriesMaterialIdAndIntersection) {
    auto sphere = make_sphere(Vector<T, 3>(0, 0, 0));
    Primitive<T> prim(std::move(sphere), 42);

    Ray<T, 3> ray(Point<T, 3>(0, 0, -3), Vector<T, 3>(0, 0, 1));
    auto hit = prim.intersect(ray);

    ASSERT_TRUE(hit.has_value());
    EXPECT_EQ(hit->material_id, 42);
    EXPECT_NEAR(hit->intersection.t, 2.0f, 1e-4f);
}

TEST(AggregateTest, ReturnsClosestHitWithMaterial) {
    Primitive<T> near_prim(make_sphere(Vector<T, 3>(0, 0, 0)), 1);
    Primitive<T> far_prim(make_sphere(Vector<T, 3>(0, 0, 5)), 2);

    LinearAggregate<T> agg({near_prim, far_prim});
    Ray<T, 3> ray(Point<T, 3>(0, 0, -3), Vector<T, 3>(0, 0, 1));

    auto hit = agg.intersect(ray);
    ASSERT_TRUE(hit.has_value());
    EXPECT_EQ(hit->material_id, 1);
    EXPECT_NEAR(hit->intersection.t, 2.0f, 1e-4f);
}

TEST(AggregateTest, IsIntersectedReportsNearestDistance) {
    Primitive<T> prim(make_sphere(Vector<T, 3>(0, 0, 2)), 7);
    LinearAggregate<T> agg({prim});

    Ray<T, 3> ray(Point<T, 3>(0, 0, -3), Vector<T, 3>(0, 0, 1));
    auto t = agg.is_intersected(ray);

    ASSERT_TRUE(t.has_value());
    EXPECT_NEAR(t.value(), 4.0f, 1e-4f);
}

}  // namespace pbpt::aggregate::testing
