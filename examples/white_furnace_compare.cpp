#include <format>
#include <variant>

#include "pbpt/serde/scene_loader.hpp"

int main() {
    auto result = pbpt::serde::load_scene<double>(
        "/Users/jinceyang/Desktop/codebase/graphics/pbpt/asset/scene/white_furnace/"
        "white_furnace_dielectric_compare.xml");

    std::visit(
        [&](auto& integrator) {
            integrator.render(result.scene, "output/white_furnace_dielectric_compare.exr", false, result.spp);
        },
        result.integrator);
    return 0;
}
