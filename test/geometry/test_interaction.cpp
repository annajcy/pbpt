#include <cmath>
#include <vector>
#include <gtest/gtest.h>

#include "pbpt.h"

namespace pbpt::geometry::testing {

class InteractionTest : public ::testing::Test {
protected:
    void SetUp() override {
        p_lower = Pt3(Float(0.9), Float(1.9), Float(2.9));
        p_upper = Pt3(Float(1.1), Float(2.1), Float(3.1));

        wo = Vec3(0, 0, 1);
        n = Normal3(0, 0, 1);
        uv = Pt2(Float(0.5), Float(0.5));
        dpdu = Vec3(1, 0, 0);
        dpdv = Vec3(0, 1, 0);
    }

    Pt3 midpoint() const {
        return p_lower.mid(p_upper);
    }

    SurfaceInteraction<Float> make_surface_interaction() const {
        return SurfaceInteraction<Float>(
            p_lower, p_upper, wo, n, uv, dpdu, dpdv
        );
    }

    SurfaceInteraction<Float> make_surface_interaction_with_normal(const Normal3& n_in) const {
        return SurfaceInteraction<Float>(
            p_lower, p_upper, wo, n_in, uv, dpdu, dpdv
        );
    }

    Pt3 p_lower;
    Pt3 p_upper;
    Vec3 wo;
    Normal3 n;
    Pt2 uv;
    Vec3 dpdu, dpdv;
};

TEST_F(InteractionTest, OffsetRayOrigin) {
    Vec3 wi(0, 0, 1);
    Point<Float, 3> offset_origin = offset_ray_origin(p_lower, p_upper, wi, n);

    Point<Float, 3> original_point = midpoint();
    EXPECT_NE(offset_origin, original_point);

    Vector<Float, 3> offset_vec = offset_origin - original_point;
    Float dot_product = offset_vec.dot(n.to_vector());
    EXPECT_GT(dot_product, Float(0));
}

TEST_F(InteractionTest, OffsetRayOriginOppositeDirection) {
    Vec3 wi(0, 0, -1);

    Point<Float, 3> offset_origin = offset_ray_origin(p_lower, p_upper, wi, n);
    Point<Float, 3> original_point = midpoint();

    Vector<Float, 3> offset_vec = offset_origin - original_point;
    Float dot_product = offset_vec.dot(n.to_vector());
    EXPECT_LT(dot_product, Float(0));
}

TEST_F(InteractionTest, OffsetRayOriginWithDifferentNormals) {
    Vec3 wi(1, 0, 0);
    Normal3 side_normal(1, 0, 0);

    Point<Float, 3> offset_origin = offset_ray_origin(p_lower, p_upper, wi, side_normal);
    Point<Float, 3> original_point = midpoint();
    EXPECT_NE(offset_origin, original_point);
}

TEST_F(InteractionTest, SurfaceInteractionConstruction) {
    SurfaceInteraction<Float> si = make_surface_interaction();

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
    SurfaceInteraction<Float> si = make_surface_interaction();

    Vec3 wi(0, 0, 1);
    Ray<Float, 3> ray = si.spawn_ray(wi);

    EXPECT_EQ(ray.direction(), wi);
    EXPECT_NE(ray.origin(), midpoint());
    EXPECT_GT(ray.t_max(), Float(0));
}

TEST_F(InteractionTest, SpawnRayToPoint) {
    SurfaceInteraction<Float> si = make_surface_interaction();

    Point<Float, 3> target(Float(5), Float(5), Float(5));
    Ray<Float, 3> ray = si.spawn_ray_to(target);

    Point<Float, 3> from_point = midpoint();
    Vector<Float, 3> expected_dir = (target - from_point).normalized();
    Vector<Float, 3> actual_dir = ray.direction();

    Float dot_product = actual_dir.dot(expected_dir);
    EXPECT_GT(dot_product, Float(0.9));
    EXPECT_GT(ray.t_max(), Float(0));
}

TEST_F(InteractionTest, DifferentNormalDirections) {
    std::vector<Normal3> normals = {
        Normal3(1, 0, 0),
        Normal3(-1, 0, 0),
        Normal3(0, 1, 0),
        Normal3(0, -1, 0),
        Normal3(0, 0, 1),
        Normal3(0, 0, -1)
    };

    Vec3 wi(0, 0, 1);

    for (const auto& normal : normals) {
        SurfaceInteraction<Float> si = make_surface_interaction_with_normal(normal);
        Ray<Float, 3> ray = si.spawn_ray(wi);

        EXPECT_EQ(ray.direction(), wi);
        EXPECT_GT(ray.t_max(), Float(0));

        Point<Float, 3> original_point = midpoint();
        Vector<Float, 3> offset_vec = ray.origin() - original_point;
        EXPECT_GT(offset_vec.length(), Float(0));
    }
}

TEST_F(InteractionTest, NumericalStability) {
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

    SurfaceInteraction<Float> si(
        small_lower, small_upper, wo, n, uv, dpdu, dpdv
    );

    Vec3 wi(0, 0, 1);
    Ray<Float, 3> ray = si.spawn_ray(wi);

    EXPECT_EQ(ray.direction(), wi);
    EXPECT_GT(ray.t_max(), Float(0));

    Point<Float, 3> original_point = small_lower.mid(small_upper);
    Vector<Float, 3> offset_vec = ray.origin() - original_point;
    EXPECT_GT(offset_vec.length(), Float(0));
    EXPECT_GT(offset_vec.length(), Float(1e-6));
}

TEST_F(InteractionTest, OffsetRayOriginBoundaryConditions) {
    Vec3 wi_perpendicular(1, 0, 0);
    Normal3 n_z(0, 0, 1);

    Point<Float, 3> offset_origin = offset_ray_origin(p_lower, p_upper, wi_perpendicular, n_z);
    Point<Float, 3> original_point = midpoint();

    Vector<Float, 3> offset_vec = offset_origin - original_point;
    EXPECT_GT(offset_vec.length(), Float(0));

    Float z_component = abs(offset_vec.z());
    Float xy_magnitude = sqrt(offset_vec.x() * offset_vec.x() + offset_vec.y() * offset_vec.y());
    EXPECT_GT(z_component, xy_magnitude * Float(0.1));
}

TEST_F(InteractionTest, OffsetScalesWithIntervalWidth) {
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

    EXPECT_GT(wide_offset_dist, narrow_offset_dist);
}

TEST_F(InteractionTest, SpawnRayToAccuracy) {
    SurfaceInteraction<Float> si1 = make_surface_interaction();

    Point<Float, 3> target(Float(10), Float(20), Float(30));
    Ray<Float, 3> ray = si1.spawn_ray_to(target);

    Point<Float, 3> ray_end = ray.at(ray.t_max());
    Vector<Float, 3> error_vec = ray_end - target;
    EXPECT_LT(error_vec.length(), Float(1.0));
}

TEST_F(InteractionTest, SpawnRayToDistantPoint) {
    SurfaceInteraction<Float> si = make_surface_interaction();

    Point<Float, 3> distant_point(Float(1000), Float(1000), Float(1000));
    Ray<Float, 3> ray = si.spawn_ray_to(distant_point);

    EXPECT_GT(ray.t_max(), Float(0));
    EXPECT_GT(ray.direction().length(), Float(0));

    Point<Float, 3> from_point = midpoint();
    Vector<Float, 3> expected_dir = (distant_point - from_point).normalized();
    Vector<Float, 3> actual_dir = ray.direction();
    Float dot_product = actual_dir.dot(expected_dir);
    EXPECT_GT(dot_product, Float(0.5));
}

}  // namespace pbpt::geometry::testing
