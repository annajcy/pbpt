#include <array>
#include <gtest/gtest.h>

#include "pbpt/aggregate/plugin/aggregate/embree_aggregate.hpp"
#include "pbpt/aggregate/plugin/aggregate/linear_aggregate.hpp"
#include "pbpt/geometry/ray.hpp"
#include "pbpt/geometry/transform.hpp"
#include "pbpt/shape/primitive.hpp"
#include "pbpt/shape/plugin/shape/sphere.hpp"

namespace pbpt::aggregate::testing {

using pbpt::geometry::Ray;
using pbpt::geometry::RayDifferential;
using pbpt::geometry::Transform;
using pbpt::math::Point;
using pbpt::math::Vector;
using pbpt::aggregate::EmbreeAggregate;
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
    auto hit = prim.intersect_ray(ray);

    ASSERT_TRUE(hit.has_value());
    EXPECT_EQ(hit->material_id, 42);
    EXPECT_NEAR(hit->intersection.t, 2.0f, 1e-4f);
}

TEST(AggregateTest, ReturnsClosestHitWithMaterial) {
    Primitive<T> near_prim(make_sphere(Vector<T, 3>(0, 0, 0)), 1);
    Primitive<T> far_prim(make_sphere(Vector<T, 3>(0, 0, 5)), 2);

    LinearAggregate<T> agg({near_prim, far_prim});
    Ray<T, 3> ray(Point<T, 3>(0, 0, -3), Vector<T, 3>(0, 0, 1));

    auto hit = agg.intersect_ray(ray);
    ASSERT_TRUE(hit.has_value());
    EXPECT_EQ(hit->material_id, 1);
    EXPECT_NEAR(hit->intersection.t, 2.0f, 1e-4f);
}

TEST(AggregateTest, IsIntersectedReportsNearestDistance) {
    Primitive<T> prim(make_sphere(Vector<T, 3>(0, 0, 2)), 7);
    LinearAggregate<T> agg({prim});

    Ray<T, 3> ray(Point<T, 3>(0, 0, -3), Vector<T, 3>(0, 0, 1));
    auto t = agg.is_intersected_ray(ray);

    ASSERT_TRUE(t.has_value());
    EXPECT_NEAR(t.value(), 4.0f, 1e-4f);
}

TEST(EmbreeAggregateTest, ReturnsClosestHitWithMaterial) {
    Primitive<T> near_prim(make_sphere(Vector<T, 3>(0, 0, 0)), 11);
    Primitive<T> far_prim(make_sphere(Vector<T, 3>(0, 0, 5)), 22);

    EmbreeAggregate<T> agg(std::vector<Primitive<T>>{near_prim, far_prim});
    Ray<T, 3> ray(Point<T, 3>(0, 0, -3), Vector<T, 3>(0, 0, 1));

    auto hit = agg.intersect_ray(ray);
    ASSERT_TRUE(hit.has_value());
    EXPECT_EQ(hit->material_id, 11);
    EXPECT_NEAR(hit->intersection.t, 2.0f, 1e-4f);
}

TEST(EmbreeAggregateTest, ReturnsNulloptWhenNoHit) {
    Primitive<T> prim(make_sphere(Vector<T, 3>(0, 0, 0)), 3);
    EmbreeAggregate<T> agg(std::vector<Primitive<T>>{prim});

    Ray<T, 3> ray(Point<T, 3>(0, 3, -3), Vector<T, 3>(0, 0, 1));
    auto hit = agg.intersect_ray(ray);

    EXPECT_FALSE(hit.has_value());
}

TEST(EmbreeAggregateTest, RespectsTMinAndTMax) {
    Primitive<T> prim(make_sphere(Vector<T, 3>(0, 0, 0)), 5);
    EmbreeAggregate<T> agg(std::vector<Primitive<T>>{prim});

    Ray<T, 3> ray_skip_first(Point<T, 3>(0, 0, -3), Vector<T, 3>(0, 0, 1), 10.0f, 2.5f);
    auto hit_skip = agg.intersect_ray(ray_skip_first);
    ASSERT_TRUE(hit_skip.has_value());
    EXPECT_NEAR(hit_skip->intersection.t, 4.0f, 1e-4f);

    Ray<T, 3> ray_clip(Point<T, 3>(0, 0, -3), Vector<T, 3>(0, 0, 1), 1.5f, 0.0f);
    auto hit_clip = agg.intersect_ray(ray_clip);
    EXPECT_FALSE(hit_clip.has_value());
}

TEST(EmbreeAggregateTest, IsIntersectedReportsOcclusion) {
    Primitive<T> prim(make_sphere(Vector<T, 3>(0, 0, 2)), 7);
    EmbreeAggregate<T> agg(std::vector<Primitive<T>>{prim});

    Ray<T, 3> ray(Point<T, 3>(0, 0, -3), Vector<T, 3>(0, 0, 1));
    auto t = agg.is_intersected_ray(ray);

    ASSERT_TRUE(t.has_value());
    EXPECT_NEAR(t.value(), 0.0f, 1e-6f);
}

TEST(EmbreeAggregateTest, RayDifferentialPopulatesSurfaceDifferentials) {
    Primitive<T> prim(make_sphere(Vector<T, 3>(0, 0, 0)), 9);
    EmbreeAggregate<T> agg(std::vector<Primitive<T>>{prim});

    Ray<T, 3> main_ray(Point<T, 3>(0, 0, -3), Vector<T, 3>(0, 0, 1));
    std::array<Ray<T, 3>, 2> differential_rays = {
        Ray<T, 3>(Point<T, 3>(0.01f, 0.0f, -3.0f), Vector<T, 3>(0, 0, 1)),
        Ray<T, 3>(Point<T, 3>(0.0f, 0.01f, -3.0f), Vector<T, 3>(0, 0, 1))
    };
    RayDifferential<T, 3> ray_diff(main_ray, differential_rays);

    auto hit = agg.intersect_ray_differential(ray_diff);
    ASSERT_TRUE(hit.has_value());
    ASSERT_TRUE(hit->intersection.differentials.has_value());
    EXPECT_GT(hit->intersection.differentials->dpdx.length(), 0.0f);
    EXPECT_GT(hit->intersection.differentials->dpdy.length(), 0.0f);
}

TEST(EmbreeAggregateTest, EmptyAggregateReturnsNullopt) {
    EmbreeAggregate<T> default_agg;
    Ray<T, 3> ray(Point<T, 3>(0, 0, -3), Vector<T, 3>(0, 0, 1));

    EXPECT_FALSE(default_agg.intersect_ray(ray).has_value());
    EXPECT_FALSE(default_agg.is_intersected_ray(ray).has_value());

    EmbreeAggregate<T> empty_agg(std::vector<Primitive<T>>{});
    EXPECT_FALSE(empty_agg.intersect_ray(ray).has_value());
    EXPECT_FALSE(empty_agg.is_intersected_ray(ray).has_value());
}

}  // namespace pbpt::aggregate::testing
