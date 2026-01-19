#include <format>
#include <vector>
#include "aggregate/aggregate.hpp"
#include "aggregate/embree_aggregate.hpp"
#include "scene/cbox_scene.hpp"
#include "scene/cbox_scene_test.hpp"
#include "scene/scene.hpp"

int main() {
    std::vector<int> spps = {1, 4, 16, 64, 256};
    for (int spp : spps) {
        auto scene = pbpt::scene::CornellBoxScene<pbpt::aggregate::LinearAggregate>();
        scene.ssp() = spp;
        scene.render(std::format("output/cbox_{}spp_linear.exr", spp));

        auto scene_ = pbpt::scene::CornellBoxScene<pbpt::aggregate::EmbreeAggregate>();
        scene_.ssp() = spp;
        scene_.render(std::format("output/cbox_{}spp_embree.exr", spp));
    }
    return 0;
}