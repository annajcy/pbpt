#include <array>
#include <gtest/gtest.h>

#include "pbpt/geometry/interaction.hpp"
#include "pbpt/geometry/ray.hpp"
#include "pbpt/math/point.hpp"
#include "pbpt/shape/shape.hpp"
#include "pbpt/shape/plugin/shape/sphere.hpp"
using namespace pbpt::math;

namespace pbpt::shape::testing {

using pbpt::geometry::offset_ray_origin;
using pbpt::geometry::Ray;
using pbpt::geometry::RayDifferential;
using pbpt::geometry::Transform;
using pbpt::math::Point;
using pbpt::math::Vector;
using pbpt::shape::Shape;
using pbpt::shape::Sphere;

using SphereD = Sphere<double>;
using SphereShapeInterface = Shape<SphereD, double>;

namespace {

SphereD make_sphere(double radius) {
    return SphereD(Transform<double>::identity(), false, radius);
}

SphereD make_partial_sphere(double radius, double z_min, double z_max, double phi_max) {
    return SphereD(Transform<double>::identity(), false, radius, z_min, z_max, phi_max);
}

SphereD make_sphere_with_transform(const Transform<double>& object_to_render, double radius) {
    return SphereD(object_to_render, false, radius);
}

}  // namespace

class SphereTest : public ::testing::Test {
protected:
    void SetUp() override {}
    void TearDown() override {}
};

TEST_F(SphereTest, BoundingBoxMatchesFullSphere) {
    auto sphere = make_sphere(1.0);

    auto bounds = sphere.bounding_box();

    EXPECT_DOUBLE_EQ(bounds.min().x(), -1.0);
    EXPECT_DOUBLE_EQ(bounds.min().y(), -1.0);
    EXPECT_DOUBLE_EQ(bounds.min().z(), -1.0);

    EXPECT_DOUBLE_EQ(bounds.max().x(), 1.0);
    EXPECT_DOUBLE_EQ(bounds.max().y(), 1.0);
    EXPECT_DOUBLE_EQ(bounds.max().z(), 1.0);
}

TEST_F(SphereTest, AreaMatchesFullSphere) {
    auto sphere = make_sphere(2.0);

    const SphereShapeInterface& shape_iface = sphere;
    auto expected_area = static_cast<double>(4.0) * pi_v<double> * sphere.radius() * sphere.radius();

    EXPECT_NEAR(shape_iface.area(), expected_area, 1e-12);
}

TEST_F(SphereTest, AreaRespectsPhiAndZClamps) {
    auto partial = make_partial_sphere(1.5, -0.25, 1.0, static_cast<double>(pi_v<double> / 2.0));

    const SphereShapeInterface& shape_iface = partial;
    auto expected_area = partial.phi_max() * partial.radius() * (partial.z_max() - partial.z_min());

    EXPECT_NEAR(shape_iface.area(), expected_area, 1e-12);
}

TEST_F(SphereTest, AreaRespectsHemisphericalClamp) {
    auto hemisphere = make_partial_sphere(3.0, 0.0, 3.0, static_cast<double>(2 * pi_v<double>));

    const SphereShapeInterface& shape_iface = hemisphere;
    auto expected_area = static_cast<double>(2.0) * pi_v<double> * hemisphere.radius() * hemisphere.radius();

    EXPECT_NEAR(shape_iface.area(), expected_area, 1e-12);
}

TEST_F(SphereTest, RayHitsSphereReturnsClosestIntersection) {
    auto sphere = make_sphere(1.0);
    Point<double, 3> origin(0.0, 0.0, -3.0);
    Vector<double, 3> direction(0.0, 0.0, 1.0);
    Ray<double, 3> ray(origin, direction);

    const SphereShapeInterface& shape_iface = sphere;
    auto t_hit = shape_iface.is_intersected_ray(ray);

    ASSERT_TRUE(t_hit.has_value());
    EXPECT_NEAR(t_hit.value(), 2.0, 1e-10);
}

TEST_F(SphereTest, RayUsesSecondRootWhenFirstBelowTMin) {
    auto sphere = make_sphere(1.0);
    Point<double, 3> origin(0.0, 0.0, -3.0);
    Vector<double, 3> direction(0.0, 0.0, 1.0);
    Ray<double, 3> ray(origin, direction, 10.0, 2.5);  // Ignore the first root at t=2

    const SphereShapeInterface& shape_iface = sphere;
    auto t_hit = shape_iface.is_intersected_ray(ray);

    ASSERT_TRUE(t_hit.has_value());
    EXPECT_NEAR(t_hit.value(), 4.0, 1e-10);
}

TEST_F(SphereTest, RayMissesClampedZRange) {
    auto hemisphere = make_partial_sphere(1.0, 0.0, 1.0, static_cast<double>(2 * pi_v<double>));
    Point<double, 3> origin(0.0, 0.0, -3.0);
    Vector<double, 3> direction(0.0, 0.0, 1.0);
    Ray<double, 3> ray(origin, direction);

    const SphereShapeInterface& shape_iface = hemisphere;
    auto t_hit = shape_iface.is_intersected_ray(ray);

    EXPECT_FALSE(t_hit.has_value());
}

TEST_F(SphereTest, SampleOnShapeReturnsPointOnSphereAndPdf) {
    auto sphere = make_sphere(1.0);
    math::Point<double, 2> u_sample{0.3, 0.7};

    auto ss = sphere.sample_on_shape(u_sample);

    EXPECT_NEAR((ss.point - math::Point<double, 3>(0, 0, 0)).length(), 1.0, 1e-6);
    EXPECT_NEAR(ss.normal.length(), 1.0, 1e-6);
    EXPECT_GE(ss.uv.x(), 0.0);
    EXPECT_LE(ss.uv.x(), 1.0);
    EXPECT_GE(ss.uv.y(), 0.0);
    EXPECT_LE(ss.uv.y(), 1.0);
    auto expected_pdf = 1.0 / sphere.area();
    EXPECT_NEAR(ss.pdf, expected_pdf, 1e-9);
}

TEST_F(SphereTest, SampleOnSolidAngleMatchesPdfFormula) {
    auto sphere = make_sphere(1.0);
    math::Point<double, 3> ref(0, 0, -3);
    math::Point<double, 2> u_sample{0.25, 0.5};

    auto ss = sphere.sample_on_solid_angle(ref, u_sample);

    double dist = (ref - math::Point<double, 3>(0, 0, 0)).length();
    double cos_theta_max = std::sqrt(std::max(0.0, 1.0 - (sphere.radius() * sphere.radius()) / (dist * dist)));
    double expected_pdf = 1.0 / (2.0 * pi_v<double> * (1.0 - cos_theta_max));

    EXPECT_NEAR(ss.pdf, expected_pdf, 1e-6);
    EXPECT_NEAR((ss.point - math::Point<double, 3>(0, 0, 0)).length(), sphere.radius(), 1e-5);

    auto pdf_eval = sphere.sample_on_solid_angle_pdf(ref, ss.point);
    EXPECT_NEAR(pdf_eval, expected_pdf, 1e-6);
}

TEST_F(SphereTest, RayMissesClampedPhiRange) {
    auto partial = make_partial_sphere(1.0, -1.0, 1.0, static_cast<double>(pi_v<double> / 2.0));
    Point<double, 3> origin(-3.0, 0.0, 0.0);
    Vector<double, 3> direction(1.0, 0.0, 0.0);
    Ray<double, 3> ray(origin, direction);

    const SphereShapeInterface& shape_iface = partial;
    auto t_hit = shape_iface.is_intersected_ray(ray);

    EXPECT_FALSE(t_hit.has_value());
}

TEST_F(SphereTest, IntersectReturnsSurfaceInteractionData) {
    auto sphere = make_sphere(1.0);
    Point<double, 3> origin(2.0, 0.0, 0.0);
    Vector<double, 3> direction(-1.0, 0.0, 0.0);
    Ray<double, 3> ray(origin, direction);

    const SphereShapeInterface& shape_iface = sphere;
    auto result = shape_iface.intersect_ray(ray);

    ASSERT_TRUE(result.has_value());
    const auto& record = result.value();
    const auto& interaction = record.interaction;
    auto t_hit = record.t;

    EXPECT_NEAR(t_hit, 1.0, 1e-12);

    auto p = interaction.point();
    EXPECT_NEAR(p.x(), 1.0, 1e-12);
    EXPECT_NEAR(p.y(), 0.0, 1e-12);
    EXPECT_NEAR(p.z(), 0.0, 1e-12);

    auto wo = interaction.wo();
    EXPECT_NEAR(wo.x(), 1.0, 1e-12);
    EXPECT_NEAR(wo.y(), 0.0, 1e-12);
    EXPECT_NEAR(wo.z(), 0.0, 1e-12);

    auto normal = interaction.n().to_vector();
    EXPECT_NEAR(normal.x(), 1.0, 1e-12);
    EXPECT_NEAR(normal.y(), 0.0, 1e-12);
    EXPECT_NEAR(normal.z(), 0.0, 1e-12);

    auto uv = interaction.uv();
    EXPECT_NEAR(uv.x(), 0.0, 1e-12);
    EXPECT_NEAR(uv.y(), 0.5, 1e-12);

    auto dpdu = interaction.dpdu();
    EXPECT_NEAR(dpdu.x(), 0.0, 1e-12);
    EXPECT_NEAR(dpdu.y(), sphere.phi_max(), 1e-12);
    EXPECT_NEAR(dpdu.z(), 0.0, 1e-12);

    auto dpdv = interaction.dpdv();
    EXPECT_NEAR(dpdv.x(), 0.0, 1e-12);
    EXPECT_NEAR(dpdv.y(), 0.0, 1e-12);
    EXPECT_NEAR(dpdv.z(), -(sphere.z_max_theta() - sphere.z_min_theta()), 1e-12);

    auto expected_error = pbpt::math::gamma<double>(5) * 1.0;
    EXPECT_NEAR(interaction.p_lower().x(), p.x() - expected_error, 1e-12);
    EXPECT_NEAR(interaction.p_upper().x(), p.x() + expected_error, 1e-12);
    EXPECT_NEAR(interaction.p_lower().y(), p.y(), 1e-12);
    EXPECT_NEAR(interaction.p_upper().y(), p.y(), 1e-12);
    EXPECT_NEAR(interaction.p_lower().z(), p.z(), 1e-12);
    EXPECT_NEAR(interaction.p_upper().z(), p.z(), 1e-12);
}

TEST_F(SphereTest, RayDifferentialComputesSurfaceDifferentials) {
    auto sphere = make_sphere(1.0);
    Ray<double, 3> main_ray(Point<double, 3>(2.0, 0.0, 0.0), Vector<double, 3>(-1.0, 0.0, 0.0));
    std::array<Ray<double, 3>, 2> diff_rays{
        Ray<double, 3>(Point<double, 3>(2.0, 1.0, 0.0), Vector<double, 3>(-1.0, 0.0, 0.0)),
        Ray<double, 3>(Point<double, 3>(2.0, 0.0, 1.0), Vector<double, 3>(-1.0, 0.0, 0.0))};
    RayDifferential<double, 3> ray_diff(main_ray, diff_rays);

    const SphereShapeInterface& shape_iface = sphere;
    auto hit = shape_iface.intersect_ray_differential(ray_diff);

    ASSERT_TRUE(hit.has_value());
    ASSERT_TRUE(hit->differentials.has_value());
    const auto& surface_diffs = hit->differentials.value();

    EXPECT_NEAR(surface_diffs.dpdx.x(), 0.0, 1e-9);
    EXPECT_NEAR(surface_diffs.dpdx.y(), 1.0, 1e-9);
    EXPECT_NEAR(surface_diffs.dpdx.z(), 0.0, 1e-9);
    EXPECT_NEAR(surface_diffs.dpdy.x(), 0.0, 1e-9);
    EXPECT_NEAR(surface_diffs.dpdy.y(), 0.0, 1e-9);
    EXPECT_NEAR(surface_diffs.dpdy.z(), 1.0, 1e-9);

    double two_pi = 2.0 * pi_v<double>;
    EXPECT_NEAR(surface_diffs.dudx, 1.0 / two_pi, 1e-9);
    EXPECT_NEAR(surface_diffs.dvdx, 0.0, 1e-9);
    EXPECT_NEAR(surface_diffs.dudy, 0.0, 1e-9);
    EXPECT_NEAR(surface_diffs.dvdy, 1.0 / pi_v<double>, 1e-9);
}

TEST_F(SphereTest, RayDifferentialDegenerateWhenDiffEqualsMain) {
    auto sphere = make_sphere(1.0);
    Ray<double, 3> main_ray(Point<double, 3>(1.0, 0.0, -3.0), Vector<double, 3>(0.0, 0.0, 1.0));
    std::array<Ray<double, 3>, 2> diff_rays{main_ray, main_ray};
    RayDifferential<double, 3> ray_diff(main_ray, diff_rays);

    const SphereShapeInterface& shape_iface = sphere;
    auto hit = shape_iface.intersect_ray_differential(ray_diff);

    ASSERT_TRUE(hit.has_value());
    EXPECT_FALSE(hit->differentials.has_value());
}

TEST_F(SphereTest, RayDifferentialWithoutDifferentialsLeavesOptionalEmpty) {
    auto sphere = make_sphere(1.0);
    Ray<double, 3> main_ray(Point<double, 3>(2.0, 0.0, 0.0), Vector<double, 3>(-1.0, 0.0, 0.0));
    std::array<Ray<double, 3>, 2> diff_rays{Ray<double, 3>(main_ray.origin(), Vector<double, 3>(0.0, 1.0, 0.0)),
                                            Ray<double, 3>(main_ray.origin(), Vector<double, 3>(0.0, 0.0, 1.0))};
    RayDifferential<double, 3> ray_diff(main_ray, diff_rays);

    const SphereShapeInterface& shape_iface = sphere;
    auto hit = shape_iface.intersect_ray_differential(ray_diff);

    ASSERT_TRUE(hit.has_value());
    EXPECT_FALSE(hit->differentials.has_value());
}

TEST_F(SphereTest, SampleOnShapeProducesConsistentValues) {
    auto sphere = make_sphere(1.0);
    const SphereShapeInterface& shape_iface = sphere;

    Point<double, 2> u_sample(0.0, 0.0);
    auto sample_opt = shape_iface.sample_on_shape(u_sample);

    const auto& sample = sample_opt;

    EXPECT_DOUBLE_EQ(sample.point.x(), 0.0);
    EXPECT_DOUBLE_EQ(sample.point.y(), 0.0);
    EXPECT_DOUBLE_EQ(sample.point.z(), -1.0);

    auto normal_vec = sample.normal.to_vector();
    EXPECT_DOUBLE_EQ(normal_vec.x(), 0.0);
    EXPECT_DOUBLE_EQ(normal_vec.y(), 0.0);
    EXPECT_DOUBLE_EQ(normal_vec.z(), -1.0);

    EXPECT_DOUBLE_EQ(sample.uv.x(), 0.0);
    EXPECT_DOUBLE_EQ(sample.uv.y(), 0.0);

    auto expected_pdf = 1.0 / shape_iface.area();
    EXPECT_NEAR(sample.pdf, expected_pdf, 1e-12);

    Point<double, 2> uv_sample(0.5, 0.5);
    sample_opt = shape_iface.sample_on_shape(uv_sample);
    const auto& sample2 = sample_opt;
    EXPECT_NEAR(sample2.point.x(), -1.0, 1e-12);
    EXPECT_NEAR(sample2.point.y(), 0.0, 1e-12);
    EXPECT_NEAR(sample2.point.z(), 0.0, 1e-12);
    auto normal_vec2 = sample2.normal.to_vector();
    EXPECT_NEAR(normal_vec2.x(), -1.0, 1e-12);
    EXPECT_NEAR(normal_vec2.y(), 0.0, 1e-12);
    EXPECT_NEAR(normal_vec2.z(), 0.0, 1e-12);
    EXPECT_NEAR(sample2.uv.x(), 0.5, 1e-12);
    EXPECT_NEAR(sample2.uv.y(), 0.5, 1e-12);
    EXPECT_NEAR(sample2.pdf, expected_pdf, 1e-12);
}

class SphereTransformTest : public ::testing::Test {
protected:
    void SetUp() override {}
    void TearDown() override {}
};

TEST_F(SphereTransformTest, BoundingBoxReflectsTranslation) {
    auto object_to_render = Transform<double>::translate(Vector<double, 3>(-2.0, 0.0, 0.0));
    auto sphere = make_sphere_with_transform(object_to_render, 1.0);

    auto bbox = sphere.bounding_box();

    EXPECT_DOUBLE_EQ(bbox.min().x(), -3.0);
    EXPECT_DOUBLE_EQ(bbox.min().y(), -1.0);
    EXPECT_DOUBLE_EQ(bbox.min().z(), -1.0);

    EXPECT_DOUBLE_EQ(bbox.max().x(), -1.0);
    EXPECT_DOUBLE_EQ(bbox.max().y(), 1.0);
    EXPECT_DOUBLE_EQ(bbox.max().z(), 1.0);
}

TEST_F(SphereTransformTest, RayIntersectionAccountsForRenderToObjectTransform) {
    auto object_to_render = Transform<double>::translate(Vector<double, 3>(0.0, 0.0, 5.0));
    auto sphere = make_sphere_with_transform(object_to_render, 1.0);
    Point<double, 3> origin(0.0, 0.0, 0.0);
    Vector<double, 3> direction(0.0, 0.0, 1.0);
    Ray<double, 3> ray(origin, direction);

    const SphereShapeInterface& shape_iface = sphere;
    auto t_hit = shape_iface.is_intersected_ray(ray);

    ASSERT_TRUE(t_hit.has_value());
    EXPECT_NEAR(t_hit.value(), 4.0, 1e-10);
}

TEST_F(SphereTransformTest, StoresTransformsAndBoundingBox) {
    auto object_to_render = Transform<double>::translate(Vector<double, 3>(1.0, 0.0, 0.0));
    auto sphere = make_sphere_with_transform(object_to_render, 1.0);

    auto expected_render_to_object = object_to_render.inversed();
    EXPECT_TRUE(sphere.render_to_object_transform() == expected_render_to_object);
    EXPECT_TRUE(sphere.object_to_render_transform() == object_to_render);

    auto bbox = sphere.bounding_box();

    EXPECT_DOUBLE_EQ(bbox.min().x(), 0.0);
    EXPECT_DOUBLE_EQ(bbox.max().x(), 2.0);
}

TEST_F(SphereTransformTest, IntersectProducesRenderSpaceSurfaceInteraction) {
    auto object_to_render = Transform<double>::translate(Vector<double, 3>(0.0, 2.0, 0.0));
    auto sphere = make_sphere_with_transform(object_to_render, 1.0);

    Point<double, 3> origin(2.0, 2.0, 0.0);
    Vector<double, 3> direction(-1.0, 0.0, 0.0);
    Ray<double, 3> ray(origin, direction);

    const SphereShapeInterface& shape_iface = sphere;
    auto result = shape_iface.intersect_ray(ray);

    ASSERT_TRUE(result.has_value());
    const auto& record = result.value();
    const auto& interaction = record.interaction;
    auto t_hit = record.t;

    EXPECT_NEAR(t_hit, 1.0, 1e-12);

    auto p = interaction.point();
    EXPECT_NEAR(p.x(), 1.0, 1e-12);
    EXPECT_NEAR(p.y(), 2.0, 1e-12);
    EXPECT_NEAR(p.z(), 0.0, 1e-12);

    auto normal = interaction.n().to_vector();
    EXPECT_NEAR(normal.x(), 1.0, 1e-12);
    EXPECT_NEAR(normal.y(), 0.0, 1e-12);
    EXPECT_NEAR(normal.z(), 0.0, 1e-12);

    auto wo = interaction.wo();
    EXPECT_NEAR(wo.x(), 1.0, 1e-12);
    EXPECT_NEAR(wo.y(), 0.0, 1e-12);
    EXPECT_NEAR(wo.z(), 0.0, 1e-12);

    auto dpdu = interaction.dpdu();
    EXPECT_NEAR(dpdu.x(), 0.0, 1e-12);
    EXPECT_NEAR(dpdu.y(), sphere.phi_max(), 1e-12);
    EXPECT_NEAR(dpdu.z(), 0.0, 1e-12);

    auto dpdv = interaction.dpdv();
    EXPECT_NEAR(dpdv.x(), 0.0, 1e-12);
    EXPECT_NEAR(dpdv.y(), 0.0, 1e-12);
    EXPECT_NEAR(dpdv.z(), -(sphere.z_max_theta() - sphere.z_min_theta()), 1e-12);

    auto expected_error = pbpt::math::gamma<double>(5) * 1.0;
    EXPECT_NEAR(interaction.p_lower().x(), p.x() - expected_error, 1e-12);
    EXPECT_NEAR(interaction.p_upper().x(), p.x() + expected_error, 1e-12);
    EXPECT_NEAR(interaction.p_lower().y(), p.y(), 1e-12);
    EXPECT_NEAR(interaction.p_upper().y(), p.y(), 1e-12);
    EXPECT_NEAR(interaction.p_lower().z(), p.z(), 1e-12);
    EXPECT_NEAR(interaction.p_upper().z(), p.z(), 1e-12);
}

TEST_F(SphereTransformTest, ExampleScenariosMatchManualExperiment) {
    auto unit_sphere = make_sphere(1.0);
    auto translate = Transform<double>::translate(Vector<double, 3>(1.0, 2.0, 3.0));
    auto transformed = make_sphere_with_transform(translate, 1.0);

    auto bbox = transformed.bounding_box();
    EXPECT_DOUBLE_EQ(bbox.min().x(), 0.0);
    EXPECT_DOUBLE_EQ(bbox.min().y(), 1.0);
    EXPECT_DOUBLE_EQ(bbox.min().z(), 2.0);
    EXPECT_DOUBLE_EQ(bbox.max().x(), 2.0);
    EXPECT_DOUBLE_EQ(bbox.max().y(), 3.0);
    EXPECT_DOUBLE_EQ(bbox.max().z(), 4.0);

    EXPECT_NEAR(transformed.area(), 4.0 * pi_v<double> * unit_sphere.radius() * unit_sphere.radius(), 1e-12);

    Ray<double, 3> ray_a(Point<double, 3>(1.0, 2.0, 0.0), Vector<double, 3>(0.0, 0.0, 1.0));
    auto hit_a = transformed.intersect_ray(ray_a);
    ASSERT_TRUE(hit_a.has_value());
    const auto& record_a = hit_a.value();
    const auto& intr_a = record_a.interaction;
    auto t_a = record_a.t;
    EXPECT_NEAR(t_a, 2.0, 1e-12);
    auto p_a = intr_a.point();
    EXPECT_NEAR(p_a.x(), 1.0, 1e-12);
    EXPECT_NEAR(p_a.y(), 2.0, 1e-12);
    EXPECT_NEAR(p_a.z(), 2.0, 1e-12);
    auto n_a = intr_a.n().to_vector();
    EXPECT_NEAR(n_a.x(), 0.0, 1e-12);
    EXPECT_NEAR(n_a.y(), 0.0, 1e-12);
    EXPECT_NEAR(n_a.z(), -1.0, 1e-12);

    auto translate_b = Transform<double>::translate(Vector<double, 3>(1.2, 2.2, 3.0));
    auto transformed_b = make_sphere_with_transform(translate_b, 1.0);
    auto hit_b = transformed_b.intersect_ray(ray_a);
    ASSERT_TRUE(hit_b.has_value());
    const auto& record_b = hit_b.value();
    const auto& intr_b = record_b.interaction;
    auto t_b = record_b.t;
    EXPECT_NEAR(t_b, 2.040834, 1e-6);
    auto p_b = intr_b.point();
    EXPECT_NEAR(p_b.x(), 1.0, 1e-6);
    EXPECT_NEAR(p_b.y(), 2.0, 1e-6);
    EXPECT_NEAR(p_b.z(), 2.040834, 1e-6);
    auto n_b = intr_b.n().to_vector();
    EXPECT_NEAR(n_b.x(), -0.2, 1e-6);
    EXPECT_NEAR(n_b.y(), -0.2, 1e-6);
    EXPECT_NEAR(n_b.z(), -0.959166, 1e-6);

    Ray<double, 3> ray_tangent(Point<double, 3>(1.0, 1.0, 2.0), Vector<double, 3>(0.0, 1.0, 0.0));
    auto hit_tangent = transformed.intersect_ray(ray_tangent);
    ASSERT_TRUE(hit_tangent.has_value());
    EXPECT_NEAR(hit_tangent->t, 1.0, 1e-12);
    auto n_tangent = hit_tangent->interaction.n().to_vector();
    EXPECT_NEAR(n_tangent.x(), 0.0, 1e-12);
    EXPECT_NEAR(n_tangent.y(), 0.0, 1e-12);
    EXPECT_NEAR(n_tangent.z(), -1.0, 1e-12);

    Ray<double, 3> ray_miss(Point<double, 3>(1.0, 1.0, 2.0), Vector<double, 3>(1.0, 1.0, 0.0));
    EXPECT_FALSE(transformed.intersect_ray(ray_miss).has_value());

    Ray<double, 3> ray_inside(Point<double, 3>(1.0, 2.0, 3.0), Vector<double, 3>(0.0, 1.0, 0.0));
    auto hit_inside = transformed.intersect_ray(ray_inside);
    ASSERT_TRUE(hit_inside.has_value());
    EXPECT_NEAR(hit_inside->t, 1.0, 1e-12);
    auto p_inside = hit_inside->interaction.point();
    EXPECT_NEAR(p_inside.x(), 1.0, 1e-12);
    EXPECT_NEAR(p_inside.y(), 3.0, 1e-12);
    EXPECT_NEAR(p_inside.z(), 3.0, 1e-12);
    auto n_inside = hit_inside->interaction.n().to_vector();
    EXPECT_NEAR(n_inside.x(), 0.0, 1e-12);
    EXPECT_NEAR(n_inside.y(), 1.0, 1e-12);
    EXPECT_NEAR(n_inside.z(), 0.0, 1e-12);

    Ray<double, 3> ray_surface(Point<double, 3>(1.0, 1.0, 3.0), Vector<double, 3>(0.0, -1.0, 0.0));
    auto hit_surface = transformed.intersect_ray(ray_surface);
    ASSERT_TRUE(hit_surface.has_value());
    EXPECT_NEAR(hit_surface->t, 0.0, 1e-12);

    const auto origin_on_surface = ray_surface.origin();
    Vector<double, 3> tiny_offset(1e-6, 1e-6, 1e-6);
    auto adjusted_origin = offset_ray_origin(origin_on_surface - tiny_offset, origin_on_surface + tiny_offset,
                                             ray_surface.direction(), Normal<double, 3>(0.0, -1.0, 0.0));
    ray_surface.origin() = adjusted_origin;
    EXPECT_FALSE(transformed.intersect_ray(ray_surface).has_value());
}

}  // namespace pbpt::shape::testing
