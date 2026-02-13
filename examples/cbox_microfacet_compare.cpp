#include <format>
#include <vector>

#include "pbpt/integrator/plugin/integrator/path_integrator.hpp"
#include "pbpt/loader/scene_loader.hpp"

int main() {
    auto scene_aniso = pbpt::loader::load_scene<double>(
        "/Users/jinceyang/Desktop/codebase/graphics/pbpt/asset/scene/cbox/cbox_microfacet_aniso_sphere.xml"
    );
    auto scene_iso = pbpt::loader::load_scene<double>(
        "/Users/jinceyang/Desktop/codebase/graphics/pbpt/asset/scene/cbox/cbox_microfacet_iso_sphere.xml"
    );

    // Keep SPP modest for side-by-side comparison.
    std::vector<int> spps = {1024};
    for (int spp : spps) {
        pbpt::integrator::PathIntegrator<double, 4> integrator(-1, 0.9);
        integrator.render(scene_aniso, spp, std::format("output/cbox_microfacet_aniso_sphere_{}.exr", spp));
        std::cout << "Rendered aniso sphere with SPP " << spp << std::endl;
        integrator.render(scene_iso, spp, std::format("output/cbox_microfacet_iso_sphere_{}.exr", spp));
        std::cout << "Rendered iso sphere with SPP " << spp << std::endl;
    }
    return 0;
}
