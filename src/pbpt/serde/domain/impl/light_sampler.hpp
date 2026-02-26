#pragma once

#include <string_view>
#include <vector>
#include <pugixml.hpp>
#include "pbpt/serde/context.hpp"
#include "pbpt/light_sampler/plugin/light_sampler/uniform_light_sampler.hpp"
#include "pbpt/serde/domain/trait_contracts.hpp"

namespace pbpt::serde {

template <typename T>
struct UniformLightSamplerSerde {
    static constexpr std::string_view domain = "light_sampler";
    static constexpr std::string_view xml_type = "uniform";
    using value_type = light_sampler::UniformLightSampler<T>;
    using load_result = void;  // Direct assignment in LoadContext
    using write_target = ValueWriteTarget<value_type>;

    static void load(const pugi::xml_node& node, LoadContext<T>& ctx) {
        // Collect all lights into a vector of pointers
        std::vector<const light::AnyLight<T>*> light_ptrs;

        // Assuming ctx.result.scene.resources.any_light_library has the lights
        // need to check how it's structured in the library.hpp
        // NamedLibrary provides id_to_name and name_to_id.
        // Also provides size() and get(id).
        const auto& library = ctx.result.scene.resources.any_light_library;
        light_ptrs.reserve(library.size());

        for (const auto& [name, id] : library.name_to_id()) {
            light_ptrs.push_back(&library.get(id));
        }

        ctx.result.scene.light_sampler = light_sampler::UniformLightSampler<T>(light_ptrs);
    }

    static void write(const write_target& target, pugi::xml_node& node, WriteContext<T>& ctx) {
        node.append_attribute("type") = xml_type.data();
    }
};

}  // namespace pbpt::serde
