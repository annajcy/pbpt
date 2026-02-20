#include <format>
#include <iostream>
#include <variant>

#include "pbpt/serde/scene_loader.hpp"

int main() {
    auto result_aniso = pbpt::serde::load_scene<double>(
        "/Users/jinceyang/Desktop/codebase/graphics/pbpt/asset/scene/cbox/cbox_microfacet_aniso_sphere.xml");
    auto result_iso = pbpt::serde::load_scene<double>(
        "/Users/jinceyang/Desktop/codebase/graphics/pbpt/asset/scene/cbox/cbox_microfacet_iso_sphere.xml");

    std::visit(
        [&](auto& integrator) {
            integrator.render(result_aniso.scene, "output/cbox_microfacet_aniso_sphere.exr", false, result_aniso.spp);
            std::cout << "Rendered aniso sphere" << std::endl;
        },
        result_aniso.integrator);

    std::visit(
        [&](auto& integrator) {
            integrator.render(result_iso.scene, "output/cbox_microfacet_iso_sphere.exr", false, result_iso.spp);
            std::cout << "Rendered iso sphere" << std::endl;
        },
        result_iso.integrator);
    return 0;
}
