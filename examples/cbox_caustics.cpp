#include <format>
#include <variant>
#include <vector>

#include "pbpt/serde/scene_loader.hpp"

int main() {
    auto result = pbpt::serde::load_scene<double>(
        "/Users/jinceyang/Desktop/codebase/graphics/rtr2/external/pbpt/asset/scene/cbox/cbox_caustic.xml");

    std::visit([&](auto& integrator) { integrator.render(result.scene, std::format("output/cbox_caustic.exr"), false, result.spp); },
               result.integrator);
    return 0;
}
