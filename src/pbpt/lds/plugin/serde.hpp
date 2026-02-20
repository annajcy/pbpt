#pragma once

#include <string>
#include <string_view>
#include <stdexcept>
#include <variant>

#include <pugixml.hpp>

#include "pbpt/serde/domain/trait_contracts.hpp"
#include "pbpt/lds/plugin/lds/independent.hpp"
#include "pbpt/lds/plugin/sampler_type.hpp"

namespace pbpt::serde {

template <typename T>
struct LdsSamplerSerde {
    static constexpr std::string_view domain = "sampler";
    static constexpr std::string_view xml_type = "ldsampler";
    using value_type = lds::IndependentSampler<T>;
    using load_result = void;
    using write_target = ValueWriteTarget<value_type>;

    static load_result load(const pugi::xml_node& node, LoadContext<T>& ctx) {
        int sample_count = 4;
        for (auto child : node.children("integer")) {
            if (std::string(child.attribute("name").value()) == "sampleCount") {
                sample_count = child.attribute("value").as_int(4);
            }
        }
        if (sample_count <= 0) {
            throw std::runtime_error("invalid sampleCount for ldsampler: must be > 0");
        }
        ctx.result.spp = sample_count;

        // Construct an IndependentSampler with a default seed.
        auto sampler = lds::IndependentSampler<T>(42);

        // Inject sampler into the integrator variant.
        std::visit([&](auto& integ) { integ.set_sampler(lds::AnySampler<T>(std::move(sampler))); },
                   ctx.result.integrator);
    }

    static void write(const write_target& target, pugi::xml_node& node, WriteContext<T>& ctx) {
        (void)target;
        if (ctx.result.spp <= 0) {
            throw std::runtime_error("invalid sampleCount for ldsampler: must be > 0");
        }
        auto sampler_node = node.append_child("sampler");
        sampler_node.append_attribute("type") = xml_type.data();

        auto sample_count_node = sampler_node.append_child("integer");
        sample_count_node.append_attribute("name") = "sampleCount";
        sample_count_node.append_attribute("value") = ctx.result.spp;
    }
};

static_assert(SamplerSerdeConcept<float, LdsSamplerSerde<float>>);

}  // namespace pbpt::serde
