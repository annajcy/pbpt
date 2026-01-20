#include "utils/image_io.hpp"
#include <iostream>

#include "camera/camera.hpp"

using namespace pbpt;

int main() {

    auto image = utils::read_image<double>("/Users/jinceyang/Desktop/codebase/graphics/pbpt/asset/scene/matpreview/envmap.exr");
    utils::write_image<double>(
        "envmap.exr",
        image
    );

    auto ldr_image = utils::read_image<double>("/Users/jinceyang/Desktop/codebase/graphics/pbpt/asset/scene/sponza/textures/00_skap.JPG");
    utils::write_image<double>(
        "00_skap.png",
        ldr_image
    );

}