#include <iostream>
#include <vector>

#include "camera/projective_camera.hpp"
#include "geometry/transform.hpp"
#include "math/vector.hpp"
#include "radiometry/color.hpp"
#include "scene/scene.hpp"
#include "shape/sphere.hpp"

int main() {
    using T = double;

    pbpt::math::Vector<int, 2> resolution(1920, 1080);
    pbpt::math::Vector<T, 2> film_size(T(2 * 0.0192f), T(2 * 0.0108f));
    pbpt::camera::ThinLensPerspectiveCamera<T> camera(
        resolution,
        film_size,
        T(0.01f),
        T(100.0f),
        T(0.0f),
        T(1.0f)
    );

    std::vector<pbpt::shape::TransformedShape<T, pbpt::shape::Sphere>> spheres;
    spheres.emplace_back(
        pbpt::shape::Sphere<T>(T(1.0f)),
        pbpt::geometry::Transform<T>::translate(pbpt::math::Vector<T, 3>(T(-1.2f), T(-0.5f), T(4.0f)))
    );
    spheres.emplace_back(
        pbpt::shape::Sphere<T>(T(0.8f)),
        pbpt::geometry::Transform<T>::translate(pbpt::math::Vector<T, 3>(T(1.4f), T(0.4f), T(5.5f)))
    );
    spheres.emplace_back(
        pbpt::shape::Sphere<T>(T(0.6f)),
        pbpt::geometry::Transform<T>::translate(pbpt::math::Vector<T, 3>(T(0.0f), T(0.9f), T(3.2f)))
    );

    std::vector<pbpt::scene::SimpleScene<T>::SceneObject> scene_objects {
        {spheres[0], pbpt::radiometry::RGB<T>(T(0.9f), T(0.1f), T(0.1f))},
        {spheres[1], pbpt::radiometry::RGB<T>(T(0.1f), T(0.9f), T(0.1f))},
        {spheres[2], pbpt::radiometry::RGB<T>(T(0.1f), T(0.1f), T(0.9f))}  
    };

    pbpt::scene::SimpleScene<T> scene(camera, scene_objects, pbpt::radiometry::constant::SwatchReflectance::Black);
    scene.render("output/simple_scene.exr");
    return 0;
}
