#include <gtest/gtest.h>

#include "shape/shape.hpp"
#include "shape/sphere.hpp"
using namespace pbpt::math;

namespace pbpt::shape::testing {

using pbpt::geometry::Ray;
using pbpt::geometry::Transform;
using pbpt::math::Point;
using pbpt::math::Vector;
using pbpt::shape::Shape;
using pbpt::shape::Sphere;
using pbpt::shape::TransformedShape;

using SphereD = Sphere<double>;
using SphereShapeInterface = Shape<SphereD, double>;
using TransformedSphereD = TransformedShape<double, Sphere>;
using TransformedSphereShapeInterface = Shape<TransformedSphereD, double>;

class SphereTest : public ::testing::Test {
protected:
    void SetUp() override {}
    void TearDown() override {}
};

TEST_F(SphereTest, BoundingBoxMatchesFullSphere) {
    SphereD sphere(1.0);

    auto bounds = sphere.bounding_box();

    EXPECT_DOUBLE_EQ(bounds.min().x(), -1.0);
    EXPECT_DOUBLE_EQ(bounds.min().y(), -1.0);
    EXPECT_DOUBLE_EQ(bounds.min().z(), -1.0);

    EXPECT_DOUBLE_EQ(bounds.max().x(), 1.0);
    EXPECT_DOUBLE_EQ(bounds.max().y(), 1.0);
    EXPECT_DOUBLE_EQ(bounds.max().z(), 1.0);
}

TEST_F(SphereTest, AreaMatchesFullSphere) {
    SphereD sphere(2.0);

    const SphereShapeInterface& shape_iface = sphere;
    auto expected_area = static_cast<double>(4.0) * pi_v<double> * sphere.radius() * sphere.radius();

    EXPECT_NEAR(shape_iface.area(), expected_area, 1e-12);
}

TEST_F(SphereTest, AreaRespectsPhiAndZClamps) {
    SphereD partial(1.5, -0.25, 1.0, static_cast<double>(pi_v<double> / 2.0));

    const SphereShapeInterface& shape_iface = partial;
    auto expected_area =
        partial.phi_max() * partial.radius() * (partial.z_max() - partial.z_min());

    EXPECT_NEAR(shape_iface.area(), expected_area, 1e-12);
}

TEST_F(SphereTest, AreaRespectsHemisphericalClamp) {
    SphereD hemisphere(3.0, 0.0, 3.0, static_cast<double>(2 * pi_v<double>));

    const SphereShapeInterface& shape_iface = hemisphere;
    auto expected_area = static_cast<double>(2.0) * pi_v<double> * hemisphere.radius() * hemisphere.radius();

    EXPECT_NEAR(shape_iface.area(), expected_area, 1e-12);
}

TEST_F(SphereTest, RayHitsSphereReturnsClosestIntersection) {
    SphereD sphere(1.0);
    Point<double, 3> origin(0.0, 0.0, -3.0);
    Vector<double, 3> direction(0.0, 0.0, 1.0);
    Ray<double, 3> ray(origin, direction);

    const SphereShapeInterface& shape_iface = sphere;
    auto t_hit = shape_iface.is_intersected(ray);

    ASSERT_TRUE(t_hit.has_value());
    EXPECT_NEAR(t_hit.value(), 2.0, 1e-10);
}

TEST_F(SphereTest, RayUsesSecondRootWhenFirstBelowTMin) {
    SphereD sphere(1.0);
    Point<double, 3> origin(0.0, 0.0, -3.0);
    Vector<double, 3> direction(0.0, 0.0, 1.0);
    Ray<double, 3> ray(origin, direction, 10.0, 2.5);  // Ignore the first root at t=2

    const SphereShapeInterface& shape_iface = sphere;
    auto t_hit = shape_iface.is_intersected(ray);

    ASSERT_TRUE(t_hit.has_value());
    EXPECT_NEAR(t_hit.value(), 4.0, 1e-10);
}

TEST_F(SphereTest, RayMissesClampedZRange) {
    SphereD hemisphere(1.0, 0.0, 1.0, static_cast<double>(2 * pi_v<double>));
    Point<double, 3> origin(0.0, 0.0, -3.0);
    Vector<double, 3> direction(0.0, 0.0, 1.0);
    Ray<double, 3> ray(origin, direction);

    const SphereShapeInterface& shape_iface = hemisphere;
    auto t_hit = shape_iface.is_intersected(ray);

    EXPECT_FALSE(t_hit.has_value());
}

TEST_F(SphereTest, RayMissesClampedPhiRange) {
    SphereD partial(1.0, -1.0, 1.0, static_cast<double>(pi_v<double> / 2.0));
    Point<double, 3> origin(-3.0, 0.0, 0.0);
    Vector<double, 3> direction(1.0, 0.0, 0.0);
    Ray<double, 3> ray(origin, direction);

    const SphereShapeInterface& shape_iface = partial;
    auto t_hit = shape_iface.is_intersected(ray);

    EXPECT_FALSE(t_hit.has_value());
}

TEST_F(SphereTest, IntersectReturnsSurfaceInteractionData) {
    SphereD sphere(1.0);
    Point<double, 3> origin(2.0, 0.0, 0.0);
    Vector<double, 3> direction(-1.0, 0.0, 0.0);
    Ray<double, 3> ray(origin, direction);

    const SphereShapeInterface& shape_iface = sphere;
    auto result = shape_iface.intersect(ray);

    ASSERT_TRUE(result.has_value());
    const auto& [interaction, t_hit] = result.value();

    EXPECT_NEAR(t_hit, 1.0, 1e-12);

    auto p = interaction.p();
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

    auto dndu = interaction.dndu().to_vector();
    EXPECT_NEAR(dndu.x(), 0.0, 1e-12);
    EXPECT_NEAR(dndu.y(), sphere.phi_max(), 1e-12);
    EXPECT_NEAR(dndu.z(), 0.0, 1e-12);

    auto dndv = interaction.dndv().to_vector();
    EXPECT_NEAR(dndv.x(), 0.0, 1e-12);
    EXPECT_NEAR(dndv.y(), 0.0, 1e-12);
    EXPECT_NEAR(dndv.z(), -(sphere.z_max_theta() - sphere.z_min_theta()), 1e-12);

    auto expected_error = pbpt::math::gamma<double>(5) * 1.0;
    EXPECT_NEAR(interaction.p_lower().x(), p.x() - expected_error, 1e-12);
    EXPECT_NEAR(interaction.p_upper().x(), p.x() + expected_error, 1e-12);
    EXPECT_NEAR(interaction.p_lower().y(), p.y(), 1e-12);
    EXPECT_NEAR(interaction.p_upper().y(), p.y(), 1e-12);
    EXPECT_NEAR(interaction.p_lower().z(), p.z(), 1e-12);
    EXPECT_NEAR(interaction.p_upper().z(), p.z(), 1e-12);
}

class TransformedSphereTest : public ::testing::Test {
protected:
    void SetUp() override {}
    void TearDown() override {}
};

TEST_F(TransformedSphereTest, BoundingBoxReflectsInverseTranslation) {
    SphereD base_sphere(1.0);
    auto render_to_object = Transform<double>::translate(Vector<double, 3>(-2.0, 0.0, 0.0));
    TransformedSphereD transformed(base_sphere, render_to_object);

    auto bbox = transformed.bounding_box();

    EXPECT_DOUBLE_EQ(bbox.min().x(), 1.0);
    EXPECT_DOUBLE_EQ(bbox.min().y(), -1.0);
    EXPECT_DOUBLE_EQ(bbox.min().z(), -1.0);

    EXPECT_DOUBLE_EQ(bbox.max().x(), 3.0);
    EXPECT_DOUBLE_EQ(bbox.max().y(), 1.0);
    EXPECT_DOUBLE_EQ(bbox.max().z(), 1.0);
}

TEST_F(TransformedSphereTest, RayIntersectionAccountsForRenderToObjectTransform) {
    SphereD base_sphere(1.0);
    auto render_to_object = Transform<double>::translate(Vector<double, 3>(0.0, 0.0, -5.0));
    TransformedSphereD transformed(base_sphere, render_to_object);
    Point<double, 3> origin(0.0, 0.0, 0.0);
    Vector<double, 3> direction(0.0, 0.0, 1.0);
    Ray<double, 3> ray(origin, direction);

    const TransformedSphereShapeInterface& shape_iface = transformed;
    auto t_hit = shape_iface.is_intersected(ray);

    ASSERT_TRUE(t_hit.has_value());
    EXPECT_NEAR(t_hit.value(), 4.0, 1e-10);
}

TEST_F(TransformedSphereTest, UpdateTransformRefreshesInverseAndBoundingBox) {
    SphereD base_sphere(1.0);
    TransformedSphereD transformed(base_sphere, Transform<double>::identity());

    transformed.update_transform(Transform<double>::translate(Vector<double, 3>(-1.0, 0.0, 0.0)));

    auto expected_render_to_object = Transform<double>::translate(Vector<double, 3>(-1.0, 0.0, 0.0));
    auto expected_object_to_render = Transform<double>::translate(Vector<double, 3>(1.0, 0.0, 0.0));

    EXPECT_TRUE(transformed.render_to_object_transform() == expected_render_to_object);
    EXPECT_TRUE(transformed.object_to_render_transform() == expected_object_to_render);

    auto bbox = transformed.bounding_box();

    EXPECT_DOUBLE_EQ(bbox.min().x(), 0.0);
    EXPECT_DOUBLE_EQ(bbox.max().x(), 2.0);
}

TEST_F(TransformedSphereTest, IntersectProducesRenderSpaceSurfaceInteraction) {
    SphereD base_sphere(1.0);
    auto render_to_object = Transform<double>::translate(Vector<double, 3>(0.0, -2.0, 0.0));
    TransformedSphereD transformed(base_sphere, render_to_object);

    Point<double, 3> origin(2.0, 2.0, 0.0);
    Vector<double, 3> direction(-1.0, 0.0, 0.0);
    Ray<double, 3> ray(origin, direction);

    const TransformedSphereShapeInterface& shape_iface = transformed;
    auto result = shape_iface.intersect(ray);

    ASSERT_TRUE(result.has_value());
    const auto& [interaction, t_hit] = result.value();

    EXPECT_NEAR(t_hit, 1.0, 1e-12);

    auto p = interaction.p();
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
    EXPECT_NEAR(dpdu.y(), base_sphere.phi_max(), 1e-12);
    EXPECT_NEAR(dpdu.z(), 0.0, 1e-12);

    auto dpdv = interaction.dpdv();
    EXPECT_NEAR(dpdv.x(), 0.0, 1e-12);
    EXPECT_NEAR(dpdv.y(), 0.0, 1e-12);
    EXPECT_NEAR(dpdv.z(), -(base_sphere.z_max_theta() - base_sphere.z_min_theta()), 1e-12);

    auto dndu = interaction.dndu().to_vector();
    EXPECT_NEAR(dndu.x(), 0.0, 1e-12);
    EXPECT_NEAR(dndu.y(), base_sphere.phi_max(), 1e-12);
    EXPECT_NEAR(dndu.z(), 0.0, 1e-12);

    auto dndv = interaction.dndv().to_vector();
    EXPECT_NEAR(dndv.x(), 0.0, 1e-12);
    EXPECT_NEAR(dndv.y(), 0.0, 1e-12);
    EXPECT_NEAR(dndv.z(), -(base_sphere.z_max_theta() - base_sphere.z_min_theta()), 1e-12);

    auto expected_error = pbpt::math::gamma<double>(5) * 1.0;
    EXPECT_NEAR(interaction.p_lower().x(), p.x() - expected_error, 1e-12);
    EXPECT_NEAR(interaction.p_upper().x(), p.x() + expected_error, 1e-12);
    EXPECT_NEAR(interaction.p_lower().y(), p.y(), 1e-12);
    EXPECT_NEAR(interaction.p_upper().y(), p.y(), 1e-12);
    EXPECT_NEAR(interaction.p_lower().z(), p.z(), 1e-12);
    EXPECT_NEAR(interaction.p_upper().z(), p.z(), 1e-12);
}

}  // namespace pbpt::shape::testing
