#pragma once

#include <string>
#include <string_view>

#include <pugixml.hpp>

#include "pbpt/serde/domain/trait_contracts.hpp"
#include "pbpt/scene/scene.hpp"

namespace pbpt::serde {

template <typename T>
struct LdsSamplerSerde {
    static constexpr std::string_view domain = "sampler";
    static constexpr std::string_view xml_type = "ldsampler";

    static void load(const pugi::xml_node& node, scene::Scene<T>& scene) {
        scene.serialization_meta.sampler_type = std::string(xml_type);
        for (auto child : node.children("integer")) {
            if (std::string(child.attribute("name").value()) == "sampleCount") {
                scene.serialization_meta.sample_count = child.attribute("value").as_int(4);
            }
        }
    }

    static void write(const scene::Scene<T>& scene, pugi::xml_node& node) {
        auto sampler = node.append_child("sampler");
        sampler.append_attribute("type") = xml_type.data();

        auto sample_count = sampler.append_child("integer");
        sample_count.append_attribute("name") = "sampleCount";
        sample_count.append_attribute("value") = scene.serialization_meta.sample_count;
    }
};

static_assert(SamplerSerdeConcept<float, LdsSamplerSerde<float>>);

}  // namespace pbpt::serde
