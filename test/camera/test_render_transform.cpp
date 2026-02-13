#include "pbpt/camera/render_transform.hpp"
#include "pbpt/geometry/transform.hpp"
#include "pbpt/math/function.hpp"
#include "pbpt/math/type_alias.hpp"
#include "pbpt/math/vector.hpp"
#include <gtest/gtest.h>

using namespace pbpt;
using namespace pbpt::camera;
using namespace pbpt::geometry;
using namespace pbpt::math;

namespace pbpt::camera::testing {

template <typename T>
void check_transform_approx(const Transform<T>& t1, const Transform<T>& t2) {
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            EXPECT_NEAR(t1.matrix()[i][j], t2.matrix()[i][j], 1e-6);
        }
    }
}

TEST(RenderTransform, CameraSpace) {
    auto camera_to_world = Transform<Float>::translate(math::Vector<Float, 3>{1.0, 2.0, 3.0}) * Transform<Float>::rotate_x(math::deg2rad(45.0));
    RenderTransform<Float> render_transform(camera_to_world, RenderSpace::Camera);

    EXPECT_EQ(render_transform.render_space(), RenderSpace::Camera);

    check_transform_approx(render_transform.camera_to_render(), Transform<Float>::identity());
    check_transform_approx(render_transform.render_to_world(), camera_to_world);
    check_transform_approx(render_transform.camera_to_render() * render_transform.render_to_world(), camera_to_world);
}

TEST(RenderTransform, WorldSpace) {
    auto camera_to_world = Transform<Float>::translate(math::Vector<Float, 3>{1.0, 2.0, 3.0}) * Transform<Float>::rotate_x(math::deg2rad(45.0));
    RenderTransform<Float> render_transform(camera_to_world, RenderSpace::World);

    EXPECT_EQ(render_transform.render_space(), RenderSpace::World);

    check_transform_approx(render_transform.camera_to_render(), camera_to_world);
    check_transform_approx(render_transform.render_to_world(), Transform<Float>::identity());
    check_transform_approx(render_transform.camera_to_render() * render_transform.render_to_world(), camera_to_world);
}

TEST(RenderTransform, CameraWorldSpace) {
    auto translation_vec = math::Vector<Float, 3>{1.0, 2.0, 3.0};
    auto rotation_angle = math::deg2rad(45.0);
    auto translation = Transform<Float>::translate(translation_vec);
    auto rotation = Transform<Float>::rotate_x(rotation_angle);
    auto camera_to_world = translation * rotation;

    RenderTransform<Float> render_transform(camera_to_world, RenderSpace::CameraWorld);

    EXPECT_EQ(render_transform.render_space(), RenderSpace::CameraWorld);

    check_transform_approx(render_transform.camera_to_render(), rotation);
    check_transform_approx(render_transform.render_to_world(), translation);
    check_transform_approx(render_transform.render_to_world() * render_transform.camera_to_render(), camera_to_world);
}

TEST(RenderTransform, ChangeSpace) {
    auto camera_to_world = Transform<Float>::translate(math::Vector<Float, 3>{1.0, 2.0, 3.0}) * Transform<Float>::rotate_x(math::deg2rad(45.0));
    RenderTransform<Float> render_transform(camera_to_world, RenderSpace::Camera);

    // Test to_render_space
    auto world_space_transform = render_transform.to_render_space(RenderSpace::World);
    EXPECT_EQ(world_space_transform.render_space(), RenderSpace::World);
    check_transform_approx(world_space_transform.camera_to_render(), camera_to_world);
    check_transform_approx(world_space_transform.render_to_world(), Transform<Float>::identity());

    // Test change_render_space
    render_transform.change_render_space(RenderSpace::World);
    EXPECT_EQ(render_transform.render_space(), RenderSpace::World);
    check_transform_approx(render_transform.camera_to_render(), camera_to_world);
    check_transform_approx(render_transform.render_to_world(), Transform<Float>::identity());
}

}

