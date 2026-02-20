#include <format>
#include <iostream>
#include <vector>
#include "pbpt/serde/scene_loader.hpp"
#include "pbpt/scene/cbox_scene.hpp"
#include "pbpt/scene/scene.hpp"
#include "pbpt/integrator/plugin/integrator/path_integrator.hpp"
#include "pbpt/texture/texture.hpp"

int main() {
    // // Debug: compare both scenes - load scene2 FIRST to avoid static id issue
    // {
    //     auto scene2 = pbpt::serde::load_scene<double>("/Users/jinceyang/Desktop/codebase/graphics/pbpt/asset/scene/cbox/cbox.xml");
    //     auto scene1 = pbpt::scene::create_cbox_scene<double>("/Users/jinceyang/Desktop/codebase/graphics/pbpt/asset/scene/cbox");
        
    //     std::cout << "=== Scene Comparison ===" << std::endl;
    //     std::cout << "Scene1 (hardcoded):" << std::endl;
    //     std::cout << "  Meshes: " << scene1.resources.mesh_library.size() << std::endl;
    //     std::cout << "  Materials: " << scene1.resources.any_material_library.size() << std::endl;
    //     std::cout << "  Lights: " << scene1.resources.any_light_library.size() << std::endl;
    //     std::cout << "  mesh_material_map size: " << scene1.resources.mesh_material_map.size() << std::endl;
    //     std::cout << "  mesh_light_map size: " << scene1.resources.mesh_light_map.size() << std::endl;
        
    //     std::cout << "Scene2 (loaded):" << std::endl;
    //     std::cout << "  Meshes: " << scene2.resources.mesh_library.size() << std::endl;
    //     std::cout << "  Materials: " << scene2.resources.any_material_library.size() << std::endl;
    //     std::cout << "  Lights: " << scene2.resources.any_light_library.size() << std::endl;
    //     std::cout << "  mesh_material_map size: " << scene2.resources.mesh_material_map.size() << std::endl;
    //     std::cout << "  mesh_light_map size: " << scene2.resources.mesh_light_map.size() << std::endl;
        
    //     std::cout << "\nScene2 mesh names:" << std::endl;
    //     for (const auto& [name, id] : scene2.resources.mesh_library.name_to_id()) {
    //         std::cout << "  " << name << " -> " << id << std::endl;
    //     }
    //     std::cout << "\nScene2 material names:" << std::endl;
    //     for (const auto& [name, id] : scene2.resources.any_material_library.name_to_id()) {
    //         std::cout << "  " << name << " -> " << id << std::endl;
    //     }
    //     std::cout << "\nScene2 light names:" << std::endl;
    //     for (const auto& [name, id] : scene2.resources.any_light_library.name_to_id()) {
    //         std::cout << "  " << name << " -> " << id << std::endl;
    //     }
        
    //     std::cout << "\nScene2 mesh_light_map:" << std::endl;
    //     for (const auto& [name, id] : scene2.resources.mesh_light_map) {
    //         std::cout << "  " << name << " -> " << id << std::endl;
    //     }
        
    //     std::cout << "\nScene1 mesh_light_map:" << std::endl;
    //     for (const auto& [name, id] : scene1.resources.mesh_light_map) {
    //         std::cout << "  " << name << " -> " << id << std::endl;
    //     }
    //     std::cout << "========================" << std::endl;
    // }

    pbpt::scene::Scene<double> scene = pbpt::serde::load_scene<double>("/Users/jinceyang/Desktop/codebase/graphics/pbpt/asset/scene/cbox/cbox.xml");
    //pbpt::scene::Scene<double> scene = pbpt::scene::create_cbox_scene<double>("/Users/jinceyang/Desktop/codebase/graphics/pbpt/asset/scene/cbox");
        
    std::vector<int> spps = {16};
    for (int spp : spps) {
        pbpt::integrator::PathIntegrator<double, 4> integrator(
            -1, 0.9
        );
        integrator.render(scene, spp, std::format("output/cbox_loader_{}.exr", spp));
    }
    return 0;
}
