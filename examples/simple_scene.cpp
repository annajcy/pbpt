#include <iostream>
#include <vector>

#include "camera/projective_camera.hpp"
#include "geometry/transform.hpp"
#include "math/vector.hpp"
#include "radiometry/color.hpp"
#include "scene/scene.hpp"
#include "shape/sphere.hpp"

int main() {
    using T = float;

    pbpt::math::Vector<int, 2> resolution(640, 360);
    pbpt::math::Vector<T, 2> film_size(T(0.036f), T(0.020f));
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

    pbpt::radiometry::RGB<T> albedo_white(T(0.95f), T(0.95f), T(0.95f));
    pbpt::scene::Scene<T> scene_white(camera, spheres, albedo_white);
    scene_white.render("simple_scene_white.png");

    std::cout << "Rendered image: simple_scene_white.png" << std::endl;

    pbpt::radiometry::RGB<T> albedo_brown(T(0.85f), T(0.35f), T(0.25f));
    pbpt::scene::Scene<T> scene(camera, spheres, albedo_brown);
    scene.render("simple_scene_brown.png");

    std::cout << "Rendered image: simple_scene_brown.png" << std::endl;
    return 0;
}
