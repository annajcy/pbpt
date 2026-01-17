#include "scene/cbox_scene.hpp"

int main() {
    auto scene = pbpt::scene::CornellBoxScene();
    std::cout << "Cornell Box Scene created successfully." << std::endl;
    scene.render("output/cbox.exr");
    return 0;
}