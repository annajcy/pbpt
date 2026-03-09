#include <array>
#include <cmath>
#include <vector>
#include <gtest/gtest.h>

#include "pbpt/pbpt.h"


namespace pbpt::geometry::testing {

class InteractionTest : public ::testing::Test {
protected:
    void SetUp() override {
        p_lower = math::Pt3(math::Float(0.9), math::Float(1.9), math::Float(2.9));
        p_upper = math::Pt3(math::Float(1.1), math::Float(2.1), math::Float(3.1));

        wo = math::Vec3(0, 0, 1);
        n = math::Normal3(0, 0, 1);
        uv = math::Pt2(math::Float(0.5), math::Float(0.5));
        dpdu = math::Vec3(1, 0, 0);
        dpdv = math::Vec3(0, 1, 0);
    }

    math::Pt3 midpoint() const { return p_lower.mid(p_upper); }

    SurfaceInteraction<math::Float> make_surface_interaction() const {
        return SurfaceInteraction<math::Float>(p_lower, p_upper, wo, n, uv, dpdu, dpdv);
    }

    SurfaceInteraction<math::Float> make_surface_interaction_with_normal(const math::Normal3& n_in) const {
        return SurfaceInteraction<math::Float>(p_lower, p_upper, wo, n_in, uv, dpdu, dpdv);
    }

    math::Pt3 p_lower;
    math::Pt3 p_upper;
    math::Vec3 wo;
    math::Normal3 n;
    math::Pt2 uv;
    math::Vec3 dpdu, dpdv;
};

TEST_F(InteractionTest, OffsetRayOrigin) {
    math::Vec3 wi(0, 0, 1);
    math::Point<math::Float, 3> offset_origin = offset_ray_origin(p_lower, p_upper, wi, n);

    math::Point<math::Float, 3> original_point = midpoint();
    EXPECT_NE(offset_origin, original_point);

    math::Vector<math::Float, 3> offset_vec = offset_origin - original_point;
    math::Float dot_product = offset_vec.dot(n.to_vector());
    EXPECT_GT(dot_product, math::Float(0));
}

TEST_F(InteractionTest, OffsetRayOriginOppositeDirection) {
    math::Vec3 wi(0, 0, -1);

    math::Point<math::Float, 3> offset_origin = offset_ray_origin(p_lower, p_upper, wi, n);
    math::Point<math::Float, 3> original_point = midpoint();

    math::Vector<math::Float, 3> offset_vec = offset_origin - original_point;
    math::Float dot_product = offset_vec.dot(n.to_vector());
    EXPECT_LT(dot_product, math::Float(0));
}

TEST_F(InteractionTest, OffsetRayOriginWithDifferentNormals) {
    math::Vec3 wi(1, 0, 0);
    math::Normal3 side_normal(1, 0, 0);

    math::Point<math::Float, 3> offset_origin = offset_ray_origin(p_lower, p_upper, wi, side_normal);
    math::Point<math::Float, 3> original_point = midpoint();
    EXPECT_NE(offset_origin, original_point);
}

TEST_F(InteractionTest, SurfaceInteractionConstruction) {
    SurfaceInteraction<math::Float> si = make_surface_interaction();

    EXPECT_EQ(si.p_lower(), p_lower);
    EXPECT_EQ(si.p_upper(), p_upper);
    EXPECT_EQ(si.point(), midpoint());

    EXPECT_EQ(si.wo(), wo);
    EXPECT_EQ(si.n(), n);
    EXPECT_EQ(si.uv(), uv);
    EXPECT_EQ(si.dpdu(), dpdu);
    EXPECT_EQ(si.dpdv(), dpdv);
}

TEST_F(InteractionTest, SpawnRay) {
    SurfaceInteraction<math::Float> si = make_surface_interaction();

    math::Vec3 wi(0, 0, 1);
    Ray<math::Float, 3> ray = si.spawn_ray(wi);

    EXPECT_EQ(ray.direction(), wi);
    EXPECT_NE(ray.origin(), midpoint());
    EXPECT_GT(ray.t_max(), math::Float(0));
}

TEST_F(InteractionTest, SpawnRayToPoint) {
    SurfaceInteraction<math::Float> si = make_surface_interaction();

    math::Point<math::Float, 3> target(math::Float(5), math::Float(5), math::Float(5));
    Ray<math::Float, 3> ray = si.spawn_ray_to(target);

    math::Point<math::Float, 3> from_point = midpoint();
    math::Vector<math::Float, 3> expected_dir = (target - from_point).normalized();
    math::Vector<math::Float, 3> actual_dir = ray.direction();

    math::Float dot_product = actual_dir.dot(expected_dir);
    EXPECT_GT(dot_product, math::Float(0.9));
    EXPECT_GT(ray.t_max(), math::Float(0));
}

TEST_F(InteractionTest, DifferentNormalDirections) {
    std::vector<math::Normal3> normals = {math::Normal3(1, 0, 0),  math::Normal3(-1, 0, 0), math::Normal3(0, 1, 0),
                                    math::Normal3(0, -1, 0), math::Normal3(0, 0, 1),  math::Normal3(0, 0, -1)};

    math::Vec3 wi(0, 0, 1);

    for (const auto& normal : normals) {
        SurfaceInteraction<math::Float> si = make_surface_interaction_with_normal(normal);
        Ray<math::Float, 3> ray = si.spawn_ray(wi);

        EXPECT_EQ(ray.direction(), wi);
        EXPECT_GT(ray.t_max(), math::Float(0));

        math::Point<math::Float, 3> original_point = midpoint();
        math::Vector<math::Float, 3> offset_vec = ray.origin() - original_point;
        EXPECT_GT(offset_vec.length(), math::Float(0));
    }
}

TEST_F(InteractionTest, NumericalStability) {
    math::Pt3 small_lower(math::Float(1.0) - math::Float(1e-4), math::Float(2.0) - math::Float(1e-4), math::Float(3.0) - math::Float(1e-4));
    math::Pt3 small_upper(math::Float(1.0) + math::Float(1e-4), math::Float(2.0) + math::Float(1e-4), math::Float(3.0) + math::Float(1e-4));

    SurfaceInteraction<math::Float> si(small_lower, small_upper, wo, n, uv, dpdu, dpdv);

    math::Vec3 wi(0, 0, 1);
    Ray<math::Float, 3> ray = si.spawn_ray(wi);

    EXPECT_EQ(ray.direction(), wi);
    EXPECT_GT(ray.t_max(), math::Float(0));

    math::Point<math::Float, 3> original_point = small_lower.mid(small_upper);
    math::Vector<math::Float, 3> offset_vec = ray.origin() - original_point;
    EXPECT_GT(offset_vec.length(), math::Float(0));
    EXPECT_GT(offset_vec.length(), math::Float(1e-6));
}

TEST_F(InteractionTest, OffsetRayOriginBoundaryConditions) {
    math::Vec3 wi_perpendicular(1, 0, 0);
    math::Normal3 n_z(0, 0, 1);

    math::Point<math::Float, 3> offset_origin = offset_ray_origin(p_lower, p_upper, wi_perpendicular, n_z);
    math::Point<math::Float, 3> original_point = midpoint();

    math::Vector<math::Float, 3> offset_vec = offset_origin - original_point;
    EXPECT_GT(offset_vec.length(), math::Float(0));

    math::Float z_component = abs(offset_vec.z());
    math::Float xy_magnitude = sqrt(offset_vec.x() * offset_vec.x() + offset_vec.y() * offset_vec.y());
    EXPECT_GT(z_component, xy_magnitude * math::Float(0.1));
}

TEST_F(InteractionTest, OffsetScalesWithIntervalWidth) {
    math::Pt3 narrow_lower(math::Float(1.0) - math::Float(0.01), math::Float(2.0) - math::Float(0.01), math::Float(3.0) - math::Float(0.01));
    math::Pt3 narrow_upper(math::Float(1.0) + math::Float(0.01), math::Float(2.0) + math::Float(0.01), math::Float(3.0) + math::Float(0.01));

    math::Pt3 wide_lower(math::Float(1.0) - math::Float(0.1), math::Float(2.0) - math::Float(0.1), math::Float(3.0) - math::Float(0.1));
    math::Pt3 wide_upper(math::Float(1.0) + math::Float(0.1), math::Float(2.0) + math::Float(0.1), math::Float(3.0) + math::Float(0.1));

    math::Vec3 wi(0, 0, 1);
    math::Normal3 n_test(0, 0, 1);

    math::Point<math::Float, 3> narrow_offset = offset_ray_origin(narrow_lower, narrow_upper, wi, n_test);
    math::Point<math::Float, 3> wide_offset = offset_ray_origin(wide_lower, wide_upper, wi, n_test);

    math::Point<math::Float, 3> narrow_center = narrow_lower.mid(narrow_upper);
    math::Point<math::Float, 3> wide_center = wide_lower.mid(wide_upper);

    math::Float narrow_offset_dist = (narrow_offset - narrow_center).length();
    math::Float wide_offset_dist = (wide_offset - wide_center).length();

    EXPECT_GT(wide_offset_dist, narrow_offset_dist);
}

TEST_F(InteractionTest, SpawnRayToAccuracy) {
    SurfaceInteraction<math::Float> si1 = make_surface_interaction();

    math::Point<math::Float, 3> target(math::Float(10), math::Float(20), math::Float(30));
    Ray<math::Float, 3> ray = si1.spawn_ray_to(target);

    math::Point<math::Float, 3> ray_end = ray.at(ray.t_max());
    math::Vector<math::Float, 3> error_vec = ray_end - target;
    EXPECT_LT(error_vec.length(), math::Float(1.0));
}

TEST_F(InteractionTest, SpawnRayToDistantPoint) {
    SurfaceInteraction<math::Float> si = make_surface_interaction();

    math::Point<math::Float, 3> distant_point(math::Float(1000), math::Float(1000), math::Float(1000));
    Ray<math::Float, 3> ray = si.spawn_ray_to(distant_point);

    EXPECT_GT(ray.t_max(), math::Float(0));
    EXPECT_GT(ray.direction().length(), math::Float(0));

    math::Point<math::Float, 3> from_point = midpoint();
    math::Vector<math::Float, 3> expected_dir = (distant_point - from_point).normalized();
    math::Vector<math::Float, 3> actual_dir = ray.direction();
    math::Float dot_product = actual_dir.dot(expected_dir);
    EXPECT_GT(dot_product, math::Float(0.5));
}

TEST_F(InteractionTest, ComputeDifferentialsFromRayDifferential) {
    SurfaceInteraction<math::Float> si(math::Pt3(math::Float(0), math::Float(0), math::Float(0)), math::Vec3(0, 0, 1), math::Normal3(0, 0, 1),
                                 math::Pt2(math::Float(0), math::Float(0)), math::Vec3(1, 0, 0), math::Vec3(0, 1, 0), math::Vec3(0, 0, 0));

    Ray<math::Float, 3> main_ray(math::Pt3(0, 0, 1), math::Vec3(0, 0, -1));
    std::array<Ray<math::Float, 3>, 2> diff_rays{Ray<math::Float, 3>(math::Pt3(1, 0, 1), math::Vec3(0, 0, -1)),
                                           Ray<math::Float, 3>(math::Pt3(0, 1, 1), math::Vec3(0, 0, -1))};

    RayDifferential<math::Float, 3> ray_diff(main_ray, diff_rays);
    auto diffs = si.compute_differentials(ray_diff);

    ASSERT_TRUE(diffs.has_value());
    EXPECT_NEAR(diffs->dpdx.x(), math::Float(1), math::Float(1e-6));
    EXPECT_NEAR(diffs->dpdx.y(), math::Float(0), math::Float(1e-6));
    EXPECT_NEAR(diffs->dpdy.x(), math::Float(0), math::Float(1e-6));
    EXPECT_NEAR(diffs->dpdy.y(), math::Float(1), math::Float(1e-6));
    EXPECT_NEAR(diffs->dudx, math::Float(1), math::Float(1e-6));
    EXPECT_NEAR(diffs->dvdx, math::Float(0), math::Float(1e-6));
    EXPECT_NEAR(diffs->dudy, math::Float(0), math::Float(1e-6));
    EXPECT_NEAR(diffs->dvdy, math::Float(1), math::Float(1e-6));
}

TEST_F(InteractionTest, ComputeDifferentialsDegenerateParallelRay) {
    SurfaceInteraction<math::Float> si(math::Pt3(math::Float(0), math::Float(0), math::Float(0)), math::Vec3(0, 0, 1), math::Normal3(0, 0, 1),
                                 math::Pt2(math::Float(0), math::Float(0)), math::Vec3(1, 0, 0), math::Vec3(0, 1, 0), math::Vec3(0, 0, 0));

    Ray<math::Float, 3> main_ray(math::Pt3(0, 0, 1), math::Vec3(1, 0, 0));
    std::array<Ray<math::Float, 3>, 2> diff_rays{main_ray, main_ray};
    RayDifferential<math::Float, 3> ray_diff(main_ray, diff_rays);

    auto diffs = si.compute_differentials(ray_diff);
    EXPECT_FALSE(diffs.has_value());
}

}  // namespace pbpt::geometry::testing
