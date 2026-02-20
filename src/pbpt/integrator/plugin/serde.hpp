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
    using load_result = void;
    using write_target = SceneWriteTarget<T>;

    static load_result load(const pugi::xml_node& node, LoadContext<T>& ctx) {
        auto& scene = ctx.scene;
        scene.serialization_meta.integrator_type = "path";
    }

    static void write(const write_target& target, pugi::xml_node& node, WriteContext<T>& ctx) {
        (void)ctx;
        (void)target;
        node.append_attribute("type") = xml_type.data();
    }
};

static_assert(IntegratorSerdeConcept<float, PathIntegratorSerde<float>>);

}  // namespace pbpt::serde
