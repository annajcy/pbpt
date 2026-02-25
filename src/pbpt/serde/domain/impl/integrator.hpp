#pragma once

#include <string_view>
#include <pugixml.hpp>
#include "pbpt/serde/domain/trait_contracts.hpp"
#include "pbpt/serde/value/value_codec_dispatch.hpp"
#include "pbpt/integrator/plugin/integrator/path_integrator.hpp"

namespace pbpt::serde {

template <typename T>
struct PathIntegratorSerde {
    static constexpr std::string_view domain = "integrator";
    static constexpr std::string_view xml_type = "path";
    using value_type = integrator::PathIntegrator<T, 4>;
    using load_result = void;
    using write_target = ValueWriteTarget<value_type>;

    static load_result load(const pugi::xml_node& node, LoadContext<T>& ctx) {
        const ValueCodecReadEnv<T> read_env{ctx.result.scene.resources, ctx.base_path};

        unsigned max_depth = static_cast<unsigned>(-1);
        T rr = T(0.9);

        for (auto child : node.children("integer")) {
            if (std::string(child.attribute("name").value()) == "max_depth") {
                int val = child.attribute("value").as_int(-1);
                max_depth = static_cast<unsigned>(val);
            }
        }
        for (auto child : node.children("float")) {
            if (std::string(child.attribute("name").value()) == "rr_threshold") {
                rr = static_cast<T>(child.attribute("value").as_float(0.9f));
            }
        }

        ctx.result.integrator = integrator::PathIntegrator<T, 4>(max_depth, rr);
    }

    static void write(const write_target& target, pugi::xml_node& node, WriteContext<T>& ctx) {
        (void)ctx;
        const auto& integ = target.value;
        node.append_attribute("type") = xml_type.data();

        auto max_depth_node = node.append_child("integer");
        max_depth_node.append_attribute("name") = "max_depth";
        max_depth_node.append_attribute("value") = static_cast<int>(integ.max_depth());
    }
};

static_assert(IntegratorSerdeConcept<float, PathIntegratorSerde<float>>);

}  // namespace pbpt::serde
