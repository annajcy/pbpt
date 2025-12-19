#include <format>
#include <vector>

#include "camera/projective_camera.hpp"
#include "camera/render_transform.hpp"
#include "geometry/transform.hpp"
#include "math/vector.hpp"
#include "radiometry/color.hpp"
#include "scene/light_scene.hpp"
#include "shape/sphere.hpp"
#include "utils/system_info.hpp"

int main() {
    using T = double;

    pbpt::math::Vector<int, 2> resolution(800, 600);
    pbpt::math::Vector<T, 2> film_size(
        T(2 * resolution.x() * 1e-5),
        T(2 * resolution.y() * 1e-5)
    );  // 20 microns per pixel

    pbpt::camera::ThinLensPerspectiveCamera<T> camera(
        resolution,
        film_size,
        T(-0.01),
        T(-100.0),
        T(0.0),
        T(1.0)
    );

    // Transform::look_at() returns a world-to-camera view transform.
    auto world_to_camera = pbpt::geometry::Transform<T>::look_at(
        pbpt::math::Point<T, 3>(T(0), T(0), T(0)),
        pbpt::math::Point<T, 3>(T(0), T(0), T(1)),
        pbpt::math::Vector<T, 3>(T(0), T(1), T(0))
    );

    auto render_transform = pbpt::camera::RenderTransform<T>::from_world_to_camera(
        world_to_camera,
        pbpt::camera::RenderSpace::World
    );

    std::vector<pbpt::scene::LightScene<T>::SceneObject> scene_objects;
    scene_objects.reserve(4);

    // Floor (large sphere).
    scene_objects.push_back({
        pbpt::shape::Sphere<T>(
            render_transform.object_to_render_from_object_to_world(
                pbpt::geometry::Transform<T>::translate(pbpt::math::Vector<T, 3>(T(0), T(-100.7), T(6.0)))
            ),
            false,
            T(100.0)
        ),
        pbpt::radiometry::RGB<T>(T(0.8), T(0.8), T(0.8))
    });

    // Three diffuse spheres.
    scene_objects.push_back({
        pbpt::shape::Sphere<T>(
            render_transform.object_to_render_from_object_to_world(
                pbpt::geometry::Transform<T>::translate(pbpt::math::Vector<T, 3>(T(-2.0), T(0.0), T(5.0)))
            ),
            false,
            T(0.7)
        ),
        pbpt::radiometry::RGB<T>(T(0.9), T(0.1), T(0.1))
    });

    scene_objects.push_back({
        pbpt::shape::Sphere<T>(
            render_transform.object_to_render_from_object_to_world(
                pbpt::geometry::Transform<T>::translate(pbpt::math::Vector<T, 3>(T(0.0), T(0.5), T(5.0)))
            ),
            false,
            T(0.7)
        ),
        pbpt::radiometry::RGB<T>(T(0.1), T(0.9), T(0.1))
    });

    scene_objects.push_back({
        pbpt::shape::Sphere<T>(
            render_transform.object_to_render_from_object_to_world(
                pbpt::geometry::Transform<T>::translate(pbpt::math::Vector<T, 3>(T(2.0), T(0.0), T(5.0)))
            ),
            false,
            T(0.7)
        ),
        pbpt::radiometry::RGB<T>(T(0.1), T(0.1), T(0.9))
    });

    // Emissive sphere area light with CIE D65 spectrum.
    pbpt::scene::LightScene<T>::SceneAreaLight area_light{
        pbpt::shape::Sphere<T>(
            render_transform.object_to_render_from_object_to_world(
                pbpt::geometry::Transform<T>::translate(pbpt::math::Vector<T, 3>(T(0.0), T(3.0), T(4.0)))
            ),
            false,
            T(0.6)
        ),
        // The CIE D65 table is roughly normalized to Y=100; using 1.0 keeps
        // brightness in a similar range to the existing simple_scene example.
        T(1.0)
    };

    std::cout << to_string(pbpt::utils::system_info()) << std::endl;

    pbpt::scene::LightScene<T> scene(camera, scene_objects, area_light);
    scene.render(
        std::format("output/light_scene_{}.exr", to_string(pbpt::utils::current_datetime())),
        render_transform.camera_to_render()
    );

    return 0;
}
