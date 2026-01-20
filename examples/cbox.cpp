#include <format>
#include <vector>
#include "aggregate/aggregate.hpp"
#include "aggregate/embree_aggregate.hpp"
#include "scene/cbox_scene.hpp"
#include "scene/scene.hpp"
#include "integrator/path_integrator.hpp"

int main() {
    std::vector<int> spps = {1, 4, 16, 64, 256};
    for (int spp : spps) {
        pbpt::scene::Scene<double> scene = pbpt::scene::create_cbox_scene<double>("/Users/jinceyang/Desktop/codebase/graphics/pbpt/asset/scene/cbox");
        pbpt::integrator::PathIntegrator<double, 4> integrator(
            5, 0.9
        );
        integrator.render(scene, spp, std::format("output/cbox_path_{}.exr", spp));
    }
    return 0;
}