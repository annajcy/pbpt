#include <format>
#include <iostream>
#include <variant>
#include <vector>
#include "pbpt/serde/scene_loader.hpp"
#include "pbpt/scene/cbox_scene.hpp"
#include "pbpt/scene/scene.hpp"
#include "pbpt/texture/texture.hpp"

int main() {
    auto result =
        pbpt::serde::load_scene<double>("/Users/jinceyang/Desktop/codebase/graphics/pbpt/asset/scene/cbox/cbox.xml");

    std::visit([&](auto& integrator) { integrator.render(result.scene, "output/cbox_loader.exr", false, result.spp); },
               result.integrator);
    return 0;
}
