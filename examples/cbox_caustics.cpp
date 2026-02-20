#include <format>
#include <vector>

#include "pbpt/integrator/plugin/integrator/path_integrator.hpp"
#include "pbpt/serde/scene_loader.hpp"
#include "pbpt/scene/scene.hpp"

int main() {
    pbpt::scene::Scene<double> scene =
        pbpt::serde::load_scene<double>(
            "/Users/jinceyang/Desktop/codebase/graphics/rtr2/external/pbpt/asset/scene/cbox/cbox_caustic.xml"
        );

    std::vector<int> spps = {256};
    for (int spp : spps) {
        pbpt::integrator::PathIntegrator<double, 4> integrator(-1, 0.9);
        integrator.render(scene, spp, std::format("output/cbox_caustic_{}.exr", spp));
    }
    return 0;
}
