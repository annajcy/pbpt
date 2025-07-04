#include <iostream>

#include "math/geometry/spherical.hpp"
#include "math/geometry/vector.hpp"
#include "math/global/function.hpp"
#include "math/geometry/ray.hpp"

using namespace pbpt;

int main() {
    std::cout << "Hello, World!" << std::endl;
    
    math::Sphere3 sp(math::Vec2{math::deg2rad(45.0), math::deg2rad(45.0)}, 1.0);
    std::cout << sp.to_cartesian() << std::endl;

    
    return 0;
}