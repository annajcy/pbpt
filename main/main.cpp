#include <iostream>

#include "math/geometry/bounding_box.hpp"
#include "math/geometry/homogeneous.hpp"
#include "math/geometry/point.hpp"
#include "math/geometry/quaternion.hpp"
#include "math/geometry/ray.hpp"
#include "math/geometry/spherical.hpp"
#include "math/geometry/transform.hpp"
#include "math/geometry/vector.hpp"
#include "math/global/function.hpp"

using namespace pbpt;

int main() {
    std::cout << "Hello, World!" << std::endl;

    math::Sphere3 sp(math::Vec2{math::deg2rad(45.0), math::deg2rad(45.0)}, 1.0);
    std::cout << sp.to_cartesian() << std::endl;

    math::Ray3 ray(math::Pt3{0.0, 0.0, 0.0}, math::Vec3{1.0, 1.0, 1.0});
    std::cout << "Ray Origin: " << ray.origin() << ", Direction: " << ray.direction() << std::endl;

    math::RayDiff3 ray_diff(ray);
    std::cout << "Ray Differential Origin: " << ray_diff.ray().origin() << ", Direction: " << ray_diff.ray().direction()
              << std::endl;

    math::Bound3 box(math::Pt3{0.0, 0.0, 0.0}, math::Pt3{1.0, 1.0, 1.0});
    std::cout << "Bounding Box: " << box << std::endl;

    math::Quat quaternion(1.0, math::Vec3{0.0, 1.0, 0.0});
    std::cout << "Quaternion: " << quaternion << std::endl;
    std::cout << "Quaternion Axis: " << quaternion.to_axis_angle().second << std::endl;
    std::cout << "Quaternion Angle: " << quaternion.to_axis_angle().first << std::endl;

    math::Transform transform = math::Transform::translate(math::Vec3{1.0, 2.0, 3.0});
    std::cout << "Transform Matrix: " << transform.mat() << std::endl;

    math::Homo3 homo3(math::Vec3{1.0, 2.0, 3.0});
    std::cout << "Homo3: " << homo3 << std::endl;

    math::Vec3 vec(1.0, 2.0, 3.0);

    auto vec_d = vec.type_cast<double>();

    constexpr math::Vec3 vec2(4.0, 5.0, 6.0);
    return 0;
}