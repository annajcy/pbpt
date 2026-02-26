#include <exception>
#include <format>
#include <iostream>
#include <stdexcept>
#include <string>
#include <variant>

#include "pbpt/serde/scene_loader.hpp"

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: pbpt_cli <scene.xml> [output.exr] [spp]\n";
        return 1;
    }

    const std::string scene_path = argv[1];
    std::string output_path = "output/pbpt_cli.exr";
    if (argc >= 3) {
        output_path = argv[2];
    }

    try {
        auto result = pbpt::serde::load_scene<double>(scene_path, {.to_left_handed = false});
        int spp = result.spp;
        if (argc >= 4) {
            spp = std::stoi(argv[3]);
            if (spp <= 0) {
                throw std::invalid_argument("spp must be > 0");
            }
        }
        std::visit([&](auto& integrator) { integrator.render(result.scene, output_path, false, spp); },
                   result.integrator);
        std::cout << std::format("Rendered '{}' -> {}\n", scene_path, output_path);
    } catch (const std::exception& e) {
        std::cerr << "pbpt_cli failed: " << e.what() << '\n';
        return 2;
    }

    return 0;
}
