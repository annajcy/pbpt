#pragma once

#include <algorithm>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <pugixml.hpp>

namespace pbpt::serde {

namespace detail {

inline std::string make_node_path(const pugi::xml_node& node) {
    std::vector<std::string> parts;
    for (auto cur = node; cur && cur.type() == pugi::node_element; cur = cur.parent()) {
        std::string part = cur.name();
        const auto id = cur.attribute("id");
        const auto name = cur.attribute("name");
        if (id && id.value()[0] != '\0') {
            part += "[@id='" + std::string(id.value()) + "']";
        } else if (name && name.value()[0] != '\0') {
            part += "[@name='" + std::string(name.value()) + "']";
        }
        parts.push_back(std::move(part));
    }
    std::reverse(parts.begin(), parts.end());

    std::string out;
    for (const auto& part : parts) {
        out += "/";
        out += part;
    }
    return out.empty() ? "/" : out;
}

[[noreturn]] inline void throw_schema_error(const std::string& scene_path, const pugi::xml_node& node,
                                            const std::string& kind, const std::string& value,
                                            const std::string& expected_keys) {
    throw std::runtime_error("load_scene Mitsuba3 schema error: scene path='" + scene_path + "', node='" +
                             make_node_path(node) + "', " + kind + "='" + value +
                             "', expected keys='" + expected_keys + "'");
}

}  // namespace detail

inline void validate_mi3_scene_schema(const pugi::xml_node& root, const std::string& scene_path) {
    if (!root || std::string(root.name()) != "scene") {
        throw std::runtime_error("load_scene Mitsuba3 schema error: scene path='" + scene_path +
                                 "', expected root <scene>.");
    }

    const std::string version = root.attribute("version").value();
    if (version != "3.0.0") {
        throw std::runtime_error("load_scene Mitsuba3 schema error: scene path='" + scene_path +
                                 "', node='/scene', field='version', value='" + version +
                                 "', expected keys='3.0.0'");
    }

    static const std::unordered_map<std::string, std::string> legacy_name_map = {
        {"toWorld", "to_world"},
        {"fovAxis", "fov_axis"},
        {"focusDistance", "focus_distance"},
        {"nearClip", "near_clip"},
        {"farClip", "far_clip"},
        {"sampleCount", "sample_count"},
        {"maxDepth", "max_depth"},
        {"emitterSamples", "emitter_samples"},
        {"bsdfSamples", "bsdf_samples"},
        {"shapeIndex", "shape_index"},
        {"maxSmoothAngle", "max_smooth_angle"},
        {"intIOR", "int_ior"},
        {"specularReflectance", "specular_reflectance"},
        {"specularTransmittance", "specular_transmittance"},
        {"diffuseReflectance", "diffuse_reflectance"},
        {"alphaU", "alpha_u"},
        {"alphaV", "alpha_v"},
        {"roughnessU", "roughness_u"},
        {"roughnessV", "roughness_v"},
        {"specularTransmission", "specular_transmission"},
        {"baseColor", "base_color"},
        {"specularTint", "specular_tint"},
        {"sheenTint", "sheen_tint"},
        {"clearcoatGloss", "clearcoat_gloss"},
        {"pixelFormat", "pixel_format"},
        {"sigmaA", "sigma_a"},
        {"sigmaS", "sigma_s"},
        {"stepSize", "step_size"},
        {"wrapModeU", "wrap_mode_u"},
        {"wrapModeV", "wrap_mode_v"},
    };

    static const std::unordered_map<std::string, std::string> legacy_tag_map = {
        {"lookAt", "lookat"},
    };

    static const std::unordered_map<std::string, std::string> legacy_plugin_map = {
        {"sampler:ldsampler", "independent"},
        {"bsdf:dielectric_specular", "dielectric"},
        {"bsdf:dielectric_rough", "roughdielectric"},
        {"bsdf:conductor_specular", "conductor"},
        {"bsdf:conductor_rough", "roughconductor"},
        {"bsdf:disneybsdf", "principled"},
        {"bsdf:disneydiffuse", "principled"},
        {"bsdf:disneymetal", "principled"},
        {"bsdf:disneyglass", "principled"},
        {"bsdf:disneyclearcoat", "principled"},
        {"bsdf:disneysheen", "principled"},
    };

    std::vector<pugi::xml_node> stack{root};
    while (!stack.empty()) {
        const auto node = stack.back();
        stack.pop_back();

        const std::string node_name = node.name();
        if (auto it = legacy_tag_map.find(node_name); it != legacy_tag_map.end()) {
            detail::throw_schema_error(scene_path, node, "tag", node_name, it->second);
        }

        const std::string field_name = node.attribute("name").value();
        if (!field_name.empty()) {
            if (auto it = legacy_name_map.find(field_name); it != legacy_name_map.end()) {
                detail::throw_schema_error(scene_path, node, "name", field_name, it->second);
            }
        }

        const std::string type_name = node.attribute("type").value();
        if (!type_name.empty()) {
            const std::string key = node_name + ":" + type_name;
            if (auto it = legacy_plugin_map.find(key); it != legacy_plugin_map.end()) {
                detail::throw_schema_error(scene_path, node, "plugin", type_name, it->second);
            }
        }

        for (auto child : node.children()) {
            if (child.type() == pugi::node_element) {
                stack.push_back(child);
            }
        }
    }
}

}  // namespace pbpt::serde
