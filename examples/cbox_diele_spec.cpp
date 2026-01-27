#include <format>
#include <vector>

#include "integrator/plugin/integrator/path_integrator.hpp"
#include "loader/scene_loader.hpp"
#include "scene/scene.hpp"

int main() {
    pbpt::scene::Scene<double> scene =
        pbpt::loader::load_scene<double>(
            "/Users/jinceyang/Desktop/codebase/graphics/pbpt/asset/scene/cbox/cbox_diele_spec.xml"
        );

    std::vector<int> spps = {256};
    for (int spp : spps) {
        pbpt::integrator::PathIntegrator<double, 4> integrator(-1, 0.9);
        integrator.render(scene, spp, std::format("output/cbox_diele_spec_{}.exr", spp));
    }
    return 0;
}
