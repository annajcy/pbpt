#include <iostream>

#include "math/geometry/bounds.hpp"
#include "math/geometry/homogeneous.hpp"
#include "math/geometry/normal.hpp"
#include "math/geometry/octahedral.hpp"
#include "math/geometry/point.hpp"
#include "math/geometry/quaternion.hpp"
#include "math/geometry/ray.hpp"
#include "math/geometry/spherical.hpp"
#include "math/geometry/transform.hpp"
#include "math/geometry/vector.hpp"
#include "math/global/function.hpp"
#include "math/geometry/directional_cone.hpp"
#include "math/global/type_alias.hpp"

using namespace pbpt;

int main() {
    std::cout << "Hello, World!" << std::endl;

    math::Sphere3 sp(math::Vec2{math::deg2rad(45.0), math::deg2rad(45.0)}, 1.0);
    std::cout << sp.to_cartesian() << std::endl;

    math::Ray3 ray(math::Pt3{0.0, 0.0, 0.0}, math::Vec3{1.0, 1.0, 1.0});
    std::cout << "Ray Origin: " << ray.origin() << ", Direction: " << ray.direction() << std::endl;

    std::array<math::Ray3, 2> rds = {
        math::Ray3(math::Pt3{0.1, 0.0, 0.0}, math::Vec3{1.0, 0.1, 0.0}),
        math::Ray3(math::Pt3{0.0, 0.1, 0.0}, math::Vec3{1.0, 0.0, 0.1})
    };

    math::RayDiff3 ray_diff(ray, rds);
    ray_diff.scale(2.0);
    std::cout << "Ray Differential Origin: " << ray_diff.main_ray().origin() << ", Direction: " << ray_diff.main_ray().direction()
              << std::endl;
    std::cout << "Ray Differential Origin: " << ray_diff.x().origin() << ", Direction: " << ray_diff.x().direction()
              << std::endl;
    std::cout << "Ray Differential Origin: " << ray_diff.y().origin() << ", Direction: " << ray_diff.y().direction()
              << std::endl;

    math::Bounds3 box(math::Pt3{0.0, 0.0, 0.0}, math::Pt3{1.0, 1.0, 1.0});
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
    math::Vec3 vec3 = vec + vec2;
    std::cout << "Vector Addition: " << vec3 << std::endl;

    math::Normal3 normal(vec);
    std::cout << "Normal3 from Vector: " << normal << std::endl;
    math::Normal3 normal3 = math::Normal3::from_vector(vec);
    std::cout << "Normal3: " << normal3 << std::endl;

    std::cout << "Vector (float): " << vec.normalized() << std::endl;
    math::OctahedralVector<float> octahedral_vec(vec.normalized());
    std::cout << "Octahedral Vector: " << octahedral_vec.decode() << std::endl;

    std::cout << "Vector (double): " << vec_d.normalized().type_cast<double>() << std::endl;
    math::OctahedralVector<double> octahedral_vec_d(vec_d.normalized());
    std::cout << "Octahedral Vector (double): " << octahedral_vec_d.decode() << std::endl;

    math::DirectionalCone<math::Float> dc(
        math::Vector<math::Float, 3>(0, 0, 1), math::deg2rad(80.0)
    );

    std::cout << "Directional Cone Direction: " << dc.direction() << std::endl;
    std::cout << "Directional Cone Angle: " << math::rad2deg(dc.angle()) << std::endl;

    bool contains = dc.contains(math::Vector<math::Float, 3>(0.5, 0.5, 0.5));
    std::cout << "Contains Point? " << contains << std::endl;

    return 0;
}