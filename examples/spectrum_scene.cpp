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

    pbpt::math::Vector<int, 2> resolution(960, 540);
    pbpt::math::Vector<T, 2> film_size(T(0.036f), T(0.020f));
    pbpt::camera::ThinLensPerspectiveCamera<T> camera(
        resolution,
        film_size,
        T(0.1f),
        T(120.0f),
        T(0.0f),
        T(1.25f)
    );

    std::vector<pbpt::shape::TransformedShape<T, pbpt::shape::Sphere>> spheres;
    spheres.emplace_back(
        pbpt::shape::Sphere<T>(T(1.2f)),
        pbpt::geometry::Transform<T>::translate(pbpt::math::Vector<T, 3>(T(-1.5f), T(-0.8f), T(5.2f)))
    );
    spheres.emplace_back(
        pbpt::shape::Sphere<T>(T(0.9f)),
        pbpt::geometry::Transform<T>::translate(pbpt::math::Vector<T, 3>(T(1.7f), T(0.1f), T(6.8f)))
    );
    spheres.emplace_back(
        pbpt::shape::Sphere<T>(T(0.6f)),
        pbpt::geometry::Transform<T>::translate(pbpt::math::Vector<T, 3>(T(0.2f), T(1.2f), T(4.5f)))
    );

    //pbpt::radiometry::RGB<T> albedo(T(0.25f), T(0.65f), T(0.95f));
    pbpt::radiometry::RGB<T> albedo(T(0.1f), T(0.9f), T(0.1f));
    pbpt::scene::Scene<T> spectral_scene(camera, spheres, albedo);
    spectral_scene.render("spectrum_scene.png");

    std::cout << "Rendered spectrum_scene.png" << std::endl;
    return 0;
}
