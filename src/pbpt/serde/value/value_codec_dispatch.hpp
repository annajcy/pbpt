#pragma once

#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>

#include <pugixml.hpp>

#include "pbpt/serde/value/value_codec_traits.hpp"
#include "pbpt/serde/value/xml_field_ops.hpp"

namespace pbpt::serde {

template <typename T, typename ValueT>
std::optional<ValueT> parse_child_value(const pugi::xml_node& node, std::string_view tag, std::string_view name,
                                        const ValueCodecReadEnv<T>& env) {
    static_assert(ValueCodecConcept<T, ValueT>, "Missing ValueCodec specialization for parse_child_value.");
    auto text = find_child_value(node, tag, name);
    if (!text) {
        return std::nullopt;
    }
    return ValueCodec<T, ValueT>::parse_text(*text, env);
}

template <typename T, typename ValueT>
ValueT parse_required_child_value(const pugi::xml_node& node, std::string_view tag, std::string_view name,
                                  const ValueCodecReadEnv<T>& env, const std::string& err_prefix) {
    auto parsed = parse_child_value<T, ValueT>(node, tag, name, env);
    if (!parsed) {
        throw std::runtime_error(err_prefix + ": missing <" + std::string(tag) + " name=\"" + std::string(name) +
                                 "\"> value");
    }
    return *parsed;
}

template <typename T, typename ValueT>
void write_child_value(pugi::xml_node& node, std::string_view tag, std::string_view name, const ValueT& value,
                       const ValueCodecWriteEnv<T>& env) {
    static_assert(ValueCodecConcept<T, ValueT>, "Missing ValueCodec specialization for write_child_value.");
    const std::string tag_name(tag);
    const std::string field_name(name);
    auto child = node.append_child(tag_name.c_str());
    child.append_attribute("name") = field_name.c_str();
    ValueCodec<T, ValueT>::write_node(value, child, env);
}

inline std::optional<std::string> parse_named_ref(const pugi::xml_node& node, std::string_view name) {
    return find_child_reference_id(node, name);
}

}  // namespace pbpt::serde
