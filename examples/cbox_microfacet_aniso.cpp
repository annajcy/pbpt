#include <format>
#include <vector>

#include "pbpt/integrator/plugin/integrator/path_integrator.hpp"
#include "pbpt/loader/scene_loader.hpp"
#include "pbpt/scene/scene.hpp"

int main() {
    pbpt::scene::Scene<double> scene =
        pbpt::loader::load_scene<double>(
            "/Users/jinceyang/Desktop/codebase/graphics/pbpt/asset/scene/cbox/cbox_microfacet_aniso_sphere.xml"
        );

    // Keep SPP modest for quick anisotropic validation.
    std::vector<int> spps = {64};
    for (int spp : spps) {
        pbpt::integrator::PathIntegrator<double, 4> integrator(-1, 0.9);
        integrator.render(scene, spp, std::format("output/cbox_microfacet_aniso_sphere_{}.exr", spp));
    }
    return 0;
}
