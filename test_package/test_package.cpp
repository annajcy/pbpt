#include "pbpt/math/vector.hpp"

int main() {
    const pbpt::math::Vec3 value{1.0f, 2.0f, 3.0f};
    return value[0] > 0.0f ? 0 : 1;
}
