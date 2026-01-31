#include <format>
#include <vector>

#include "integrator/plugin/integrator/path_integrator.hpp"
#include "loader/scene_loader.hpp"

int main() {
    auto scene = pbpt::loader::load_scene<double>(
        "/Users/jinceyang/Desktop/codebase/graphics/pbpt/asset/scene/white_furnace/white_furnace_dielectric_compare.xml"
    );

    // Keep SPP modest for quick comparison.
    std::vector<int> spps = {128};
    for (int spp : spps) {
        pbpt::integrator::PathIntegrator<double, 4> integrator(-1, 0.9);
        integrator.render(scene, spp, std::format("output/white_furnace_dielectric_compare_{}.exr", spp));
    }
    return 0;
}
