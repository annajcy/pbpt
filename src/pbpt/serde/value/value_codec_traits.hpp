#pragma once

#include <cmath>
#include <cctype>
#include <concepts>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>

#include <pugixml.hpp>

#include "pbpt/scene/scene.hpp"

namespace pbpt::serde {

template <typename T>
struct ValueCodecReadEnv {
    const scene::RenderResources<T>& resources;
    std::filesystem::path base_dir;
    bool load_microfacet_dist{false};

    std::string resolve_path(const std::string& rel_path) const { return (base_dir / rel_path).string(); }
};

template <typename T>
struct ValueCodecWriteEnv {
    const scene::RenderResources<T>& resources;
    std::filesystem::path scene_dir;
    std::filesystem::path mesh_dir;
    std::filesystem::path texture_dir;
    bool write_microfacet_dist{false};
};

template <typename T, typename ValueT, typename Enable = void>
struct ValueCodec {
    static ValueT parse_node(const pugi::xml_node&, const ValueCodecReadEnv<T>&) = delete;
    static void write_node(const ValueT&, pugi::xml_node&, const ValueCodecWriteEnv<T>&) = delete;
    static ValueT parse_text(std::string_view, const ValueCodecReadEnv<T>&) = delete;
    static std::string write_text(const ValueT&, const ValueCodecWriteEnv<T>&) = delete;
};

template <typename T, typename ValueT>
struct ValueCodec<T, ValueT, std::enable_if_t<std::is_arithmetic_v<ValueT>>> {
    static ValueT parse_node(const pugi::xml_node& node, const ValueCodecReadEnv<T>& env) {
        const char* value = node.attribute("value").value();
        if (!value || value[0] == '\0') {
            throw std::runtime_error("missing 'value' attribute for numeric value codec");
        }
        return parse_text(std::string_view(value), env);
    }

    static void write_node(const ValueT& value, pugi::xml_node& node, const ValueCodecWriteEnv<T>& env) {
        const auto text = write_text(value, env);
        node.append_attribute("value") = text.c_str();
    }

    static ValueT parse_text(std::string_view text, const ValueCodecReadEnv<T>&) {
        std::istringstream iss{std::string(text)};

        if constexpr (std::is_same_v<ValueT, bool>) {
            std::string token;
            if (!(iss >> token)) {
                throw std::runtime_error("failed to parse bool value");
            }
            for (char& c : token) {
                c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            }
            if (token == "true" || token == "1") {
                return true;
            }
            if (token == "false" || token == "0") {
                return false;
            }
            throw std::runtime_error("invalid bool value: " + token);
        } else {
            ValueT parsed{};
            if (!(iss >> parsed)) {
                throw std::runtime_error("failed to parse numeric value");
            }
            if constexpr (std::is_floating_point_v<ValueT>) {
                if (!std::isfinite(parsed)) {
                    throw std::runtime_error("numeric value is not finite");
                }
            }
            iss >> std::ws;
            if (!iss.eof()) {
                throw std::runtime_error("numeric value has unexpected trailing tokens");
            }
            return parsed;
        }
    }

    static std::string write_text(const ValueT& value, const ValueCodecWriteEnv<T>&) {
        if constexpr (std::is_same_v<ValueT, bool>) {
            return value ? "true" : "false";
        } else {
            std::ostringstream oss;
            if constexpr (std::is_floating_point_v<ValueT>) {
                oss << std::setprecision(std::numeric_limits<ValueT>::max_digits10);
            }
            oss << value;
            return oss.str();
        }
    }
};

template <typename T>
struct ValueCodec<T, std::string, void> {
    static std::string parse_node(const pugi::xml_node& node, const ValueCodecReadEnv<T>& env) {
        const char* value = node.attribute("value").value();
        if (!value) {
            return std::string{};
        }
        return parse_text(std::string_view(value), env);
    }

    static void write_node(const std::string& value, pugi::xml_node& node, const ValueCodecWriteEnv<T>& env) {
        const auto text = write_text(value, env);
        node.append_attribute("value") = text.c_str();
    }

    static std::string parse_text(std::string_view text, const ValueCodecReadEnv<T>&) { return std::string(text); }

    static std::string write_text(const std::string& value, const ValueCodecWriteEnv<T>&) { return value; }
};

template <typename T, typename ValueT>
concept ValueCodecConcept = requires(const pugi::xml_node& node, pugi::xml_node& out_node, const ValueT& value,
                                     const ValueCodecReadEnv<T>& read_env,
                                     const ValueCodecWriteEnv<T>& write_env) {
    { ValueCodec<T, ValueT>::parse_node(node, read_env) } -> std::same_as<ValueT>;
    { ValueCodec<T, ValueT>::write_node(value, out_node, write_env) } -> std::same_as<void>;
    { ValueCodec<T, ValueT>::parse_text(std::string_view{}, read_env) } -> std::same_as<ValueT>;
    { ValueCodec<T, ValueT>::write_text(value, write_env) } -> std::same_as<std::string>;
};

}  // namespace pbpt::serde
