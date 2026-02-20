#pragma once

#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <string>
#include <string_view>

#include "pbpt/serde/value/value_codec_traits.hpp"
#include "pbpt/texture/texture.hpp"

namespace pbpt::texture {

inline WrapMode wrap_mode_from_string(std::string_view value) {
    std::string mode(value);
    std::transform(mode.begin(), mode.end(), mode.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (mode == "clamp") {
        return WrapMode::Clamp;
    }
    if (mode == "repeat") {
        return WrapMode::Repeat;
    }
    throw std::runtime_error("Unsupported wrap mode: " + mode);
}

inline std::string wrap_mode_to_string(WrapMode mode) {
    switch (mode) {
        case WrapMode::Clamp:
            return "clamp";
        case WrapMode::Repeat:
            return "repeat";
    }
    return "repeat";
}

}  // namespace pbpt::texture

namespace pbpt::serde {

template <typename T>
struct ValueCodec<T, texture::WrapMode> {
    static texture::WrapMode parse_node(const pugi::xml_node& node, const ValueCodecReadEnv<T>& env) {
        const char* value = node.attribute("value").value();
        if (!value || value[0] == '\0') {
            throw std::runtime_error("wrap mode node is missing value attribute");
        }
        return parse_text(std::string_view(value), env);
    }

    static void write_node(const texture::WrapMode& value, pugi::xml_node& node, const ValueCodecWriteEnv<T>& env) {
        const auto text = write_text(value, env);
        node.append_attribute("value") = text.c_str();
    }

    static texture::WrapMode parse_text(std::string_view text, const ValueCodecReadEnv<T>&) {
        return texture::wrap_mode_from_string(text);
    }

    static std::string write_text(const texture::WrapMode& value, const ValueCodecWriteEnv<T>&) {
        return texture::wrap_mode_to_string(value);
    }
};

}  // namespace pbpt::serde
