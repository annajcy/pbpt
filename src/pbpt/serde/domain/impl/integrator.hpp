#pragma once

#include <string_view>
#include <pugixml.hpp>
#include "pbpt/serde/domain/trait_contracts.hpp"
#include "pbpt/serde/value/value_codec_dispatch.hpp"
#include "pbpt/integrator/plugin/integrator/path_integrator.hpp"
#include "pbpt/integrator/plugin/integrator/simple_path_integrator.hpp"

namespace pbpt::serde {

template <typename T>
struct SimplePathIntegratorSerde {
    static constexpr std::string_view domain = "integrator";
    static constexpr std::string_view xml_type = "simple_path";
    using value_type = integrator::SimplePathIntegrator<T, 4>;
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
        if (ctx.config.load_integrator_rr) {
            for (auto child : node.children("float")) {
                if (std::string(child.attribute("name").value()) == "rr_threshold") {
                    rr = static_cast<T>(child.attribute("value").as_float(0.9f));
                }
            }
        }

        ctx.result.integrator = integrator::SimplePathIntegrator<T, 4>(max_depth, rr);
    }

    static void write(const write_target& target, pugi::xml_node& node, WriteContext<T>& ctx) {
        node.append_attribute("type") = xml_type.data();
        auto max_depth = node.append_child("integer");
        max_depth.append_attribute("name") = "max_depth";
        max_depth.append_attribute("value") = std::to_string(target.value.max_depth()).c_str();

        if (ctx.config.write_integrator_rr) {
            if (std::abs(target.value.rr_threshold() - T(0.9)) > T(1e-5)) {
                auto rr = node.append_child("float");
                rr.append_attribute("name") = "rr_threshold";
                rr.append_attribute("value") = std::to_string(target.value.rr_threshold()).c_str();
            }
        }
    }
};

template <typename T>
struct PathIntegratorSerde {
    static constexpr std::string_view domain = "integrator";
    static constexpr std::string_view xml_type = "path";
    using load_result = void;
    using value_type = integrator::PathIntegrator<T, 4>;
    using write_target = ValueWriteTarget<value_type>;

    static load_result load(const pugi::xml_node& node, LoadContext<T>& ctx) {
        int max_depth_str = -1;
        T rr = T(0.9);
        for (auto child : node.children("integer")) {
            if (std::string(child.attribute("name").value()) == "max_depth") {
                max_depth_str = child.attribute("value").as_int(-1);
            }
        }
        if (ctx.config.load_integrator_rr) {
            for (auto child : node.children("float")) {
                if (std::string(child.attribute("name").value()) == "rr_threshold") {
                    rr = static_cast<T>(child.attribute("value").as_float(0.9f));
                }
            }
        }
        ctx.result.integrator = integrator::PathIntegrator<T, 4>(max_depth_str, rr);
    }

    static void write(const write_target& target, pugi::xml_node& node, WriteContext<T>& ctx) {
        node.append_attribute("type") = xml_type.data();
        auto max_depth = node.append_child("integer");
        max_depth.append_attribute("name") = "max_depth";
        max_depth.append_attribute("value") = std::to_string(target.value.max_depth()).c_str();

        if (ctx.config.write_integrator_rr) {
            if (std::abs(target.value.rr_threshold() - T(0.9)) > T(1e-5)) {
                auto rr = node.append_child("float");
                rr.append_attribute("name") = "rr_threshold";
                rr.append_attribute("value") = std::to_string(target.value.rr_threshold()).c_str();
            }
        }
    }
};

static_assert(IntegratorSerdeConcept<float, SimplePathIntegratorSerde<float>>);

}  // namespace pbpt::serde
