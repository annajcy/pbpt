#pragma once

#include <optional>
#include <string>
#include <string_view>

#include <pugixml.hpp>

namespace pbpt::serde {

inline std::optional<std::string> find_child_value(const pugi::xml_node& node, std::string_view tag,
                                                   std::string_view name) {
    const std::string tag_name(tag);
    for (auto child : node.children(tag_name.c_str())) {
        if (std::string_view(child.attribute("name").value()) == name) {
            const char* value = child.attribute("value").value();
            if (value && value[0] != '\0') {
                return std::string(value);
            }
        }
    }
    return std::nullopt;
}

inline std::optional<std::string> find_child_reference_id(const pugi::xml_node& node, std::string_view name) {
    for (auto child : node.children("ref")) {
        if (std::string_view(child.attribute("name").value()) == name) {
            const char* id = child.attribute("id").value();
            if (id && id[0] != '\0') {
                return std::string(id);
            }
        }
    }
    return std::nullopt;
}

inline std::optional<std::string> find_first_reference_id(const pugi::xml_node& node) {
    auto child = node.child("ref");
    if (!child) {
        return std::nullopt;
    }
    const char* id = child.attribute("id").value();
    if (id && id[0] != '\0') {
        return std::string(id);
    }
    return std::nullopt;
}

}  // namespace pbpt::serde
