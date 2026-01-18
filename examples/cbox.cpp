#include <format>
#include "scene/cbox_scene.hpp"
#include "scene/cbox_scene_test.hpp"

int main() {
    int spp = 128;
    auto scene = pbpt::scene::CornellBoxScene();
    scene.ssp() = spp;
    std::cout << "Cornell Box Scene created successfully." << std::endl;
    scene.render(std::format("output/cbox_{}spp.exr", spp));
    return 0;
}