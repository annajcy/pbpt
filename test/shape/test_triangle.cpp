#include <array>
#include <gtest/gtest.h>
#include <vector>

#include "geometry/ray.hpp"
#include "geometry/transform.hpp"
#include "math/point.hpp"
#include "math/vector.hpp"
#include "shape/triangle.hpp"

namespace pbpt::shape::testing {

using T = double;
using pbpt::geometry::Ray;
using pbpt::geometry::RayDifferential;
using pbpt::geometry::Transform;
using pbpt::math::Point;
using pbpt::math::Vector;
using TriangleMeshD = TriangleMesh<T>;
using TriangleD = Triangle<T>;
using TriangleShapeInterface = Shape<TriangleD, T>;

namespace {
TriangleMeshD make_unit_mesh() {
    std::vector<int> indices{0, 1, 2};
    std::vector<Point<T, 3>> positions{
        Point<T, 3>(0, 0, 0),
        Point<T, 3>(1, 0, 0),
        Point<T, 3>(0, 1, 0)
    };
    std::vector<Point<T, 2>> uvs{
        Point<T, 2>(0, 0),
        Point<T, 2>(1, 0),
        Point<T, 2>(0, 1)
    };
    return TriangleMeshD(Transform<T>::identity(), indices, positions, {}, uvs, false);
}
}  // namespace

TEST(TriangleTest, BoundingBoxMatchesVertices) {
    TriangleMeshD mesh = make_unit_mesh();
    TriangleD tri(mesh, 0);

    auto bbox = tri.bounding_box();
    EXPECT_DOUBLE_EQ(bbox.min().x(), 0.0);
    EXPECT_DOUBLE_EQ(bbox.min().y(), 0.0);
    EXPECT_DOUBLE_EQ(bbox.min().z(), 0.0);
    EXPECT_DOUBLE_EQ(bbox.max().x(), 1.0);
    EXPECT_DOUBLE_EQ(bbox.max().y(), 1.0);
    EXPECT_DOUBLE_EQ(bbox.max().z(), 0.0);
}

TEST(TriangleTest, AreaIsHalfCrossProduct) {
    TriangleMeshD mesh = make_unit_mesh();
    TriangleD tri(mesh, 0);
    const TriangleShapeInterface& shape_iface = tri;

    EXPECT_NEAR(shape_iface.area(), 0.5, 1e-12);
}

TEST(TriangleTest, RayHitReturnsIntersectionAndInterpolatedUV) {
    TriangleMeshD mesh = make_unit_mesh();
    TriangleD tri(mesh, 0);

    Ray<T, 3> ray(Point<T, 3>(0.25, 0.25, 1.0), Vector<T, 3>(0, 0, -1));
    auto hit = tri.intersect(ray);

    ASSERT_TRUE(hit.has_value());
    EXPECT_NEAR(hit->t, 1.0, 1e-9);
    EXPECT_NEAR(hit->interaction.uv().x(), 0.25, 1e-9);
    EXPECT_NEAR(hit->interaction.uv().y(), 0.25, 1e-9);
    EXPECT_NEAR(hit->interaction.n().z(), 1.0, 1e-9);
}

TEST(TriangleTest, RayMissOutsideTriangle) {
    TriangleMeshD mesh = make_unit_mesh();
    TriangleD tri(mesh, 0);

    Ray<T, 3> ray(Point<T, 3>(1.5, 1.5, 1.0), Vector<T, 3>(0, 0, -1));
    auto hit = tri.intersect(ray);

    EXPECT_FALSE(hit.has_value());
}

TEST(TriangleTest, RayDifferentialComputesSurfaceDifferentials) {
    TriangleMeshD mesh = make_unit_mesh();
    TriangleD tri(mesh, 0);

    Ray<T, 3> main_ray(Point<T, 3>(0.25, 0.25, 1.0), Vector<T, 3>(0, 0, -1));
    std::array<Ray<T, 3>, 2> diff_rays{
        Ray<T, 3>(Point<T, 3>(0.26, 0.25, 1.0), Vector<T, 3>(0, 0, -1)),
        Ray<T, 3>(Point<T, 3>(0.25, 0.26, 1.0), Vector<T, 3>(0, 0, -1))
    };
    RayDifferential<T, 3> ray_diff(main_ray, diff_rays);

    auto hit = tri.intersect(ray_diff);
    ASSERT_TRUE(hit.has_value());
    ASSERT_TRUE(hit->differentials.has_value());
    const auto& surface_diffs = hit->differentials.value();

    const T offset = T(0.01);
    EXPECT_NEAR(surface_diffs.dpdx.x(), offset, 1e-9);
    EXPECT_NEAR(surface_diffs.dpdx.y(), 0.0, 1e-9);
    EXPECT_NEAR(surface_diffs.dpdx.z(), 0.0, 1e-9);
    EXPECT_NEAR(surface_diffs.dpdy.x(), 0.0, 1e-9);
    EXPECT_NEAR(surface_diffs.dpdy.y(), offset, 1e-9);
    EXPECT_NEAR(surface_diffs.dpdy.z(), 0.0, 1e-9);
    EXPECT_NEAR(surface_diffs.dudx, offset, 1e-9);
    EXPECT_NEAR(surface_diffs.dvdx, 0.0, 1e-9);
    EXPECT_NEAR(surface_diffs.dudy, 0.0, 1e-9);
    EXPECT_NEAR(surface_diffs.dvdy, offset, 1e-9);
}

TEST(TriangleTest, RayDifferentialDegenerateWhenDiffEqualsMain) {
    TriangleMeshD mesh = make_unit_mesh();
    TriangleD tri(mesh, 0);

    Ray<T, 3> main_ray(Point<T, 3>(0.25, 0.25, 1.0), Vector<T, 3>(0, 0, -1));
    std::array<Ray<T, 3>, 2> diff_rays{main_ray, main_ray};
    RayDifferential<T, 3> ray_diff(main_ray, diff_rays);

    auto hit = tri.intersect(ray_diff);
    ASSERT_TRUE(hit.has_value());
    EXPECT_FALSE(hit->differentials.has_value());
}

TEST(TriangleTest, RayDifferentialWithoutDifferentialsLeavesOptionalEmpty) {
    TriangleMeshD mesh = make_unit_mesh();
    TriangleD tri(mesh, 0);

    Ray<T, 3> main_ray(Point<T, 3>(0.25, 0.25, 1.0), Vector<T, 3>(0, 0, -1));
    std::array<Ray<T, 3>, 2> diff_rays{
        Ray<T, 3>(main_ray.origin(), Vector<T, 3>(1.0, 0.0, 0.0)),
        Ray<T, 3>(main_ray.origin(), Vector<T, 3>(0.0, 1.0, 0.0))
    };
    RayDifferential<T, 3> ray_diff(main_ray, diff_rays);
    auto hit = tri.intersect(ray_diff);

    ASSERT_TRUE(hit.has_value());
    EXPECT_FALSE(hit->differentials.has_value());
}

TEST(TriangleTest, SampleOnShapeUniformPdfAndNormal) {
    TriangleMeshD mesh = make_unit_mesh();
    TriangleD tri(mesh, 0);

    auto ss = tri.sample_on_shape(Point<T, 2>(0.2, 0.6));

    EXPECT_NEAR(ss.normal.length(), 1.0, 1e-9);
    EXPECT_NEAR(ss.pdf, 1.0 / tri.area(), 1e-12);
    EXPECT_NEAR(tri.sample_on_shape_pdf(ss.point), 1.0 / tri.area(), 1e-12);
    EXPECT_GE(ss.uv.x(), 0.0);
    EXPECT_LE(ss.uv.x(), 1.0);
    EXPECT_GE(ss.uv.y(), 0.0);
    EXPECT_LE(ss.uv.y(), 1.0);
}

TEST(TriangleTest, SampleOnSolidAngleMatchesAreaConversion) {
    TriangleMeshD mesh = make_unit_mesh();
    TriangleD tri(mesh, 0);

    Point<T, 3> ref(0.2, 0.2, 1.0);
    auto ss = tri.sample_on_solid_angle(ref, Point<T, 2>(0.3, 0.7));

    auto wi = ss.point - ref;
    T dist_sq = wi.length_squared();
    wi /= std::sqrt(dist_sq);
    T abs_cos = std::abs(ss.normal.dot(-wi));
    T expected_pdf = dist_sq / (tri.area() * abs_cos);

    EXPECT_NEAR(ss.pdf, expected_pdf, 1e-9);
    EXPECT_NEAR(tri.sample_on_solid_angle_pdf(ref, ss.point), expected_pdf, 1e-9);
}

}  // namespace pbpt::shape::testing
