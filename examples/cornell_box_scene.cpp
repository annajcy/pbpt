#include <format>
#include <vector>

#include "camera/projective_camera.hpp"
#include "camera/render_transform.hpp"
#include "geometry/transform.hpp"
#include "math/vector.hpp"
#include "radiometry/color.hpp"
#include "scene/triangle_scene.hpp"
#include "shape/triangle.hpp"
#include "shape/sphere.hpp"
#include "utils/system_info.hpp"

namespace {

template <typename T>
pbpt::shape::TriangleMesh<T> make_face_mesh(
    const std::vector<pbpt::math::Point<T, 3>>& corners,
    const pbpt::math::Normal<T, 3>& n
) {
    // indices: 0-1-2, 1-3-2
    std::vector<int> indices{0, 1, 2, 1, 3, 2};
    std::vector<pbpt::math::Normal<T, 3>> normals(4, n);
    return pbpt::shape::TriangleMesh<T>(
        pbpt::geometry::Transform<T>::identity(), // already in render space
        indices,
        corners,
        normals
    );
}

} // namespace

int main() {
    using T = double;

    // Camera setup
    pbpt::math::Vector<int, 2> resolution(800, 800);
    pbpt::math::Vector<T, 2> film_size(
        T(resolution.x() * 2e-5),
        T(resolution.y() * 2e-5)
    );  // 10 microns per pixel (narrower FOV)

    // Camera looking down +Z, box in front
    const bool enable_depth_of_field = true;
    const int samples_per_pixel = 4;
    const pbpt::math::Point<T, 3> camera_eye(T(0), T(1), T(1.5));
    const pbpt::math::Point<T, 3> camera_target(T(0), T(1), T(4));
    T lens_radius{}, focal_distance{};

    if (enable_depth_of_field) {
        lens_radius = 0.2;
        focal_distance = 2.5;  // focus on the box
    } else {
        lens_radius = T(0);
        focal_distance = T(1);
    }

    pbpt::camera::ThinLensPerspectiveCamera<T> camera(
        resolution,
        film_size,
        T(-0.01),
        T(-100.0),
        lens_radius,
        focal_distance
    );

    auto world_to_camera = pbpt::geometry::Transform<T>::look_at(
        camera_eye,     // move closer for tighter framing
        camera_target,  // look at box center
        pbpt::math::Vector<T, 3>(T(0), T(1), T(0))
    );

    auto render_transform = pbpt::camera::RenderTransform<T>::from_world_to_camera(
        world_to_camera,
        pbpt::camera::RenderSpace::World
    );

    // Cornell box bounds in render space
    const T x0 = T(-1.0), x1 = T(1.0);
    const T y0 = T(0.0),  y1 = T(2.0);
    const T z0 = T(3.0),  z1 = T(5.0);

    // Keep meshes alive across all triangles.
    std::vector<std::unique_ptr<pbpt::shape::TriangleMesh<T>>> meshes;

    // Walls (each as a mesh of two triangles)
    std::vector<pbpt::scene::TriangleScene<T>::SceneObject> scene_objects;
    scene_objects.reserve(5);

    // Back wall (facing -Z)
    {
        std::vector<pbpt::math::Point<T, 3>> corners = {
            pbpt::math::Point<T, 3>(x0, y0, z1),
            pbpt::math::Point<T, 3>(x1, y0, z1),
            pbpt::math::Point<T, 3>(x0, y1, z1),
            pbpt::math::Point<T, 3>(x1, y1, z1)
        };
        meshes.push_back(std::make_unique<pbpt::shape::TriangleMesh<T>>(make_face_mesh(corners, pbpt::math::Normal<T, 3>(0, 0, -1))));
        scene_objects.push_back({pbpt::shape::Triangle<T>(*meshes.back(), 0), pbpt::radiometry::RGB<T>(T(0.8), T(0.8), T(0.8))});
        scene_objects.push_back({pbpt::shape::Triangle<T>(*meshes.back(), 1), pbpt::radiometry::RGB<T>(T(0.8), T(0.8), T(0.8))});
    }

    // Floor (facing +Y)
    {
        std::vector<pbpt::math::Point<T, 3>> corners = {
            pbpt::math::Point<T, 3>(x0, y0, z0),
            pbpt::math::Point<T, 3>(x1, y0, z0),
            pbpt::math::Point<T, 3>(x0, y0, z1),
            pbpt::math::Point<T, 3>(x1, y0, z1)
        };
        meshes.push_back(std::make_unique<pbpt::shape::TriangleMesh<T>>(make_face_mesh(corners, pbpt::math::Normal<T, 3>(0, 1, 0))));
        scene_objects.push_back({pbpt::shape::Triangle<T>(*meshes.back(), 0), pbpt::radiometry::RGB<T>(T(0.8), T(0.8), T(0.8))});
        scene_objects.push_back({pbpt::shape::Triangle<T>(*meshes.back(), 1), pbpt::radiometry::RGB<T>(T(0.8), T(0.8), T(0.8))});
    }

    // Ceiling (facing -Y)
    {
        std::vector<pbpt::math::Point<T, 3>> corners = {
            pbpt::math::Point<T, 3>(x0, y1, z0),
            pbpt::math::Point<T, 3>(x1, y1, z0),
            pbpt::math::Point<T, 3>(x0, y1, z1),
            pbpt::math::Point<T, 3>(x1, y1, z1)
        };
        meshes.push_back(std::make_unique<pbpt::shape::TriangleMesh<T>>(make_face_mesh(corners, pbpt::math::Normal<T, 3>(0, -1, 0))));
        scene_objects.push_back({pbpt::shape::Triangle<T>(*meshes.back(), 0), pbpt::radiometry::RGB<T>(T(0.8), T(0.8), T(0.8))});
        scene_objects.push_back({pbpt::shape::Triangle<T>(*meshes.back(), 1), pbpt::radiometry::RGB<T>(T(0.8), T(0.8), T(0.8))});
    }

    // Left wall (facing +X) red
    {
        std::vector<pbpt::math::Point<T, 3>> corners = {
            pbpt::math::Point<T, 3>(x0, y0, z0),
            pbpt::math::Point<T, 3>(x0, y1, z0),
            pbpt::math::Point<T, 3>(x0, y0, z1),
            pbpt::math::Point<T, 3>(x0, y1, z1)
        };
        meshes.push_back(std::make_unique<pbpt::shape::TriangleMesh<T>>(make_face_mesh(corners, pbpt::math::Normal<T, 3>(1, 0, 0))));
        scene_objects.push_back({pbpt::shape::Triangle<T>(*meshes.back(), 0), pbpt::radiometry::RGB<T>(T(0.8), T(0.1), T(0.1))});
        scene_objects.push_back({pbpt::shape::Triangle<T>(*meshes.back(), 1), pbpt::radiometry::RGB<T>(T(0.8), T(0.1), T(0.1))});
    }

    // Right wall (facing -X) green
    {
        std::vector<pbpt::math::Point<T, 3>> corners = {
            pbpt::math::Point<T, 3>(x1, y0, z0),
            pbpt::math::Point<T, 3>(x1, y1, z0),
            pbpt::math::Point<T, 3>(x1, y0, z1),
            pbpt::math::Point<T, 3>(x1, y1, z1)
        };
        meshes.push_back(std::make_unique<pbpt::shape::TriangleMesh<T>>(make_face_mesh(corners, pbpt::math::Normal<T, 3>(-1, 0, 0))));
        scene_objects.push_back({pbpt::shape::Triangle<T>(*meshes.back(), 0), pbpt::radiometry::RGB<T>(T(0.1), T(0.8), T(0.1))});
        scene_objects.push_back({pbpt::shape::Triangle<T>(*meshes.back(), 1), pbpt::radiometry::RGB<T>(T(0.1), T(0.8), T(0.1))});
    }

    // Stanford bunny mesh (OBJ)
    {
        auto bunny_transform =
            pbpt::geometry::Transform<T>::translate(pbpt::math::Vector<T, 3>(T(0.0), T(0.0), T(4.0)))
            * pbpt::geometry::Transform<T>::scale(T(8.0));

        meshes.push_back(std::make_unique<pbpt::shape::TriangleMesh<T>>(
            bunny_transform,
            "asset/model/stanford_bunny.obj",
            false
        ));

        const auto& bunny_mesh = *meshes.back();
        int bunny_triangles = static_cast<int>(bunny_mesh.indices().size() / 3);
        for (int i = 0; i < bunny_triangles; ++i) {
            scene_objects.push_back({
                pbpt::shape::Triangle<T>(bunny_mesh, i),
                pbpt::radiometry::RGB<T>(T(0.75), T(0.75), T(0.75))
            });
        }
    }

    // Sphere area light at the center of the box.
    const pbpt::math::Vector<T, 3> light_center(
        (x0 + x1) / T(2),
        (y0 + y1) / T(2),
        z0
    );
    pbpt::scene::TriangleScene<T>::SceneAreaLight area_light{
        pbpt::shape::Sphere<T>(
            pbpt::geometry::Transform<T>::translate(light_center),
            false,
            T(0.1)
        ),
        T(10.0)
    };

    std::cout << pbpt::utils::to_string(pbpt::utils::system_info()) << std::endl;

    pbpt::scene::TriangleScene<T> scene(camera, scene_objects, area_light);
    scene.render(
        std::format("output/cornell_box_{}.exr", pbpt::utils::to_string(pbpt::utils::current_datetime())),
        render_transform.camera_to_render(),
        samples_per_pixel
    );

    return 0;
}
