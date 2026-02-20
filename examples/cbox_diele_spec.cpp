#include <format>
#include <variant>

#include "pbpt/serde/scene_loader.hpp"

int main() {
    auto result = pbpt::serde::load_scene<double>(
        "/Users/jinceyang/Desktop/codebase/graphics/pbpt/asset/scene/cbox/cbox_diele_spec.xml");

    std::visit([&](auto& integrator) { integrator.render(result.scene, "output/cbox_diele_spec.exr", false, result.spp); },
               result.integrator);
    return 0;
}
