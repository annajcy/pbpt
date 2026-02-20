#pragma once

#include <string_view>
#include <pugixml.hpp>
#include "pbpt/serde/domain/trait_contracts.hpp"
#include "pbpt/scene/scene.hpp"

namespace pbpt::serde {

template <typename T>
struct PathIntegratorSerde {
    static constexpr std::string_view domain = "integrator";
    static constexpr std::string_view xml_type = "path";

    static void load(const pugi::xml_node& node, scene::Scene<T>& scene) {
        scene.serialization_meta.integrator_type = "path";
    }

    static void write(const scene::Scene<T>& scene, pugi::xml_node& node) {
        node.append_attribute("type") = xml_type.data();
    }
};

static_assert(IntegratorSerdeConcept<float, PathIntegratorSerde<float>>);

}  // namespace pbpt::serde
