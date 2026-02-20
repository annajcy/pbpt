#include <format>
#include <variant>
#include <vector>

#include "pbpt/serde/scene_writer.hpp"
#include "pbpt/serde/scene_loader.hpp"
#include "pbpt/scene/scene.hpp"
#include "pbpt/utils/image_io.hpp"

int main() {
    auto result = pbpt::serde::load_scene<double>("asset/scene/cbox/cbox_checkerboard_texture.xml");

    pbpt::serde::write_scene(result, "output/cbox/cbox_checkerboard_texture.xml");
    result = pbpt::serde::load_scene<double>("output/cbox/cbox_checkerboard_texture.xml");

    std::visit([&](auto& integrator) { 
        for (int spp = 1; spp <= 1024; spp *= 2) {
            integrator.render(result.scene, "output/cbox_checkerboard_texture_" + std::to_string(spp) + ".exr", false,
                              spp); 
        }
    }, result.integrator);
    return 0;
}
