#include <exception>
#include <format>
#include <iostream>
#include <string>

#include "pbpt/integrator/plugin/integrator/path_integrator.hpp"
#include "pbpt/serde/scene_loader.hpp"
#include "pbpt/scene/scene.hpp"

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: render_scene <scene.xml> [spp] [output.exr]\n";
        return 1;
    }

    const std::string scene_path = argv[1];
    int spp = 16;
    if (argc >= 3) {
        spp = std::stoi(argv[2]);
    }

    std::string output_path = "output/render_scene.exr";
    if (argc >= 4) {
        output_path = argv[3];
    }

    try {
        auto scene = pbpt::serde::load_scene<double>(scene_path);
        pbpt::integrator::PathIntegrator<double, 4> integrator(-1, 0.9);
        integrator.render(scene, spp, output_path);
        std::cout << std::format("Rendered '{}' with spp={} -> {}\n", scene_path, spp, output_path);
    } catch (const std::exception& e) {
        std::cerr << "render_scene failed: " << e.what() << '\n';
        return 2;
    }

    return 0;
}
