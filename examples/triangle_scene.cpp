#include <format>

#include "camera/projective_camera.hpp"
#include "camera/render_transform.hpp"
#include "scene/triangle_scene.hpp"
#include "geometry/transform.hpp"
#include "math/vector.hpp"
#include "radiometry/color.hpp"
#include "shape/sphere.hpp"
#include "shape/triangle.hpp"
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

    auto world_to_camera = pbpt::geometry::Transform<T>::look_at(
        pbpt::math::Point<T, 3>(T(0), T(0), T(0)),
        pbpt::math::Point<T, 3>(T(0), T(0), T(1)),
        pbpt::math::Vector<T, 3>(T(0), T(1), T(0))
    );

    auto render_transform = pbpt::camera::RenderTransform<T>::from_world_to_camera(
        world_to_camera,
        pbpt::camera::RenderSpace::World
    );

    // Keep meshes alive for the lifetime of the scene.
    std::vector<std::unique_ptr<pbpt::shape::TriangleMesh<T>>> meshes;

    // Geometry: a simple rectangle made of two triangles facing the camera.
    std::vector<int> quad_indices = {0, 2, 1, 1, 2, 3};
    std::vector<pbpt::math::Point<T, 3>> quad_positions = {
        pbpt::math::Point<T, 3>{-1.5, -1.0, 6.0},
        pbpt::math::Point<T, 3>{ 1.5, -1.0, 6.0},
        pbpt::math::Point<T, 3>{-1.5,  1.0, 6.0},
        pbpt::math::Point<T, 3>{ 1.5,  1.0, 6.0}
    };
    std::vector<pbpt::math::Normal<T, 3>> quad_normals(4, pbpt::math::Normal<T, 3>(0, 0, -1));
    std::vector<pbpt::math::Point<T, 2>> quad_uvs = {
        pbpt::math::Point<T, 2>{0, 0}, 
        pbpt::math::Point<T, 2>{1, 0}, 
        pbpt::math::Point<T, 2>{0, 1}, 
        pbpt::math::Point<T, 2>{1, 1}
    };

    meshes.push_back(std::make_unique<pbpt::shape::TriangleMesh<T>>(
        render_transform.object_to_render_from_object_to_world(pbpt::geometry::Transform<T>::identity()),
        quad_indices,
        quad_positions,
        quad_normals,
        quad_uvs,
        false
    ));

    std::vector<pbpt::shape::Triangle<T>> triangles;
    triangles.reserve(quad_indices.size() / 3);
    for (int i = 0; i < static_cast<int>(quad_indices.size() / 3); ++i) {
        triangles.emplace_back(*meshes.back(), i);
    }

    std::vector<pbpt::scene::TriangleScene<T>::SceneObject> scene_objects{
        {triangles[0], pbpt::radiometry::RGB<T>(T(0.8), T(0.2), T(0.2))},
        {triangles[1], pbpt::radiometry::RGB<T>(T(0.2), T(0.8), T(0.8))}
    };

    // Area light: small emissive sphere in front of the rectangle.
    pbpt::scene::TriangleScene<T>::SceneAreaLight area_light{
        pbpt::shape::Sphere<T>(
            render_transform.object_to_render_from_object_to_world(
                pbpt::geometry::Transform<T>::translate(pbpt::math::Vector<T, 3>(T(0.0), T(1.4), T(3.0)))
            ),
            false,
            T(0.4)
        ),
        T(3.0)
    };

    std::cout << pbpt::utils::to_string(pbpt::utils::system_info()) << std::endl;

    pbpt::scene::TriangleScene<T> scene(camera, scene_objects, area_light);
    scene.render(
        std::format("output/triangle_scene_{}.exr", pbpt::utils::to_string(pbpt::utils::current_datetime())),
        render_transform.camera_to_render()
    );

    return 0;
}
