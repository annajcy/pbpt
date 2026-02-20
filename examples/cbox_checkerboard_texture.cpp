#include <format>
#include <vector>

#include "pbpt/integrator/plugin/integrator/path_integrator.hpp"
#include "pbpt/serde/scene_writer.hpp"
#include "pbpt/serde/scene_loader.hpp"
#include "pbpt/scene/scene.hpp"
#include "pbpt/utils/image_io.hpp"

int main() {
    pbpt::scene::Scene<double> scene =
        pbpt::serde::load_scene<double>("asset/scene/cbox/cbox_checkerboard_texture.xml");

    pbpt::serde::write_scene(scene, "output/cbox/cbox_checkerboard_texture.xml");
    scene = pbpt::serde::load_scene<double>("output/cbox/cbox_checkerboard_texture.xml");

    std::vector<int> spps = {1, 4, 16, 64, 256, 1024, 4096};
    for (int spp : spps) {
        pbpt::integrator::PathIntegrator<double, 4> integrator(-1, 0.9);
        integrator.render(scene, spp, std::format("output/cbox_checkerboard_texture_{}.exr", spp));
    }
    return 0;
}
