#include <format>
#include <vector>

#include "camera/projective_camera.hpp"
#include "camera/render_transform.hpp"
#include "geometry/transform.hpp"
#include "math/function.hpp"
#include "math/vector.hpp"
#include "radiometry/color.hpp"
#include "scene/scene.hpp"
#include "shape/sphere.hpp"
#include "utils/system_info.hpp"

int main() {
    using T = double;

    pbpt::math::Vector<int, 2> resolution(800, 600);
    pbpt::math::Vector<T, 2> film_size(
        T(2 * resolution.x() * 1e-5), 
        T(2 * resolution.y() * 1e-5)
    ); // 20 microns per pixel

    pbpt::camera::ThinLensPerspectiveCamera<T> camera(
        resolution,
        film_size,
        T(-0.01f),
        T(-100.0f),
        T(0.0f),
        T(1.0f)
    );

    auto camera_to_world = pbpt::geometry::Transform<T>::look_at(
        pbpt::math::Point<T, 3>(T(0.0f), T(0.0f), T(0.0f)), 
        pbpt::math::Point<T, 3>(T(0.0f), T(0.0f), T(1.0f)), 
        pbpt::math::Vector<T, 3>(T(0.0f), T(1.0f), T(0.0f))
    );

    //auto camera_to_world = pbpt::geometry::Transform<T>::rotate_y(pbpt::math::deg2rad(180.0));
    //auto camera_to_world = pbpt::geometry::Transform<T>::identity();

    pbpt::camera::RenderTransform<T> render_transform{
        camera_to_world.inversed(),
        pbpt::camera::RenderSpace::CameraWorld
    };

    T offset_x = T(0.5f);
    T offset_y = T(1.0f);

    std::vector<pbpt::shape::TransformedShape<T, pbpt::shape::Sphere>> spheres;
    spheres.emplace_back(
        pbpt::shape::Sphere<T>(T(0.7f)),
        pbpt::geometry::Transform<T>::translate(pbpt::math::Vector<T, 3>(T(-2.0f) + offset_x, T(0.0f) + offset_y, T(5.0f))),
        render_transform
    ); // red sphere
    spheres.emplace_back(
        pbpt::shape::Sphere<T>(T(0.7f)),
        pbpt::geometry::Transform<T>::translate(pbpt::math::Vector<T, 3>(T(0.0f) + offset_x, T(0.5f) + offset_y, T(5.0f))),
        render_transform
    ); // green sphere
    spheres.emplace_back(
        pbpt::shape::Sphere<T>(T(0.7f)),
        pbpt::geometry::Transform<T>::translate(pbpt::math::Vector<T, 3>(T(2.0f) + offset_x, T(0.0f) + offset_y, T(5.0f))),
        render_transform
    ); // blue sphere

    std::vector<pbpt::scene::SimpleScene<T>::SceneObject> scene_objects {
        {spheres[0], pbpt::radiometry::RGB<T>(T(0.9f), T(0.1f), T(0.1f))}, // Red
        {spheres[1], pbpt::radiometry::RGB<T>(T(0.1f), T(0.9f), T(0.1f))}, // Green
        {spheres[2], pbpt::radiometry::RGB<T>(T(0.1f), T(0.1f), T(0.9f))}  // Blue
    };

    std::cout << to_string(pbpt::utils::system_info()) << std::endl;
    pbpt::scene::SimpleScene<T> scene(camera, scene_objects, pbpt::radiometry::constant::SwatchReflectance::Black);
    scene.render(
        std::format("output/simple_scene_tent_{}.exr", to_string(pbpt::utils::current_datetime())), 
        render_transform.camera_to_render()
    );
    return 0;
}
