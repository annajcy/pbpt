#pragma once

#include <algorithm>
#include <cctype>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>

#include "pbpt/material/model.hpp"
#include "pbpt/serde/value/value_codec_dispatch.hpp"

namespace pbpt::serde {

template <typename T>
struct ValueCodec<T, material::MicrofacetModel<T>> {
    static material::MicrofacetModel<T> parse_node(const pugi::xml_node& node, const ValueCodecReadEnv<T>& env) {
        T alpha_x = T(0.1);
        T alpha_y = T(0.1);
        bool alpha_found = false;
        bool roughness_found = false;
        auto distribution = material::MicrofacetDistribution::Beckmann;

        if (auto alpha = parse_child_value<T, T>(node, "float", "alpha", env)) {
            alpha_x = alpha_y = *alpha;
            alpha_found = true;
        }
        if (auto alpha = parse_child_value<T, T>(node, "float", "alpha_x", env)) {
            alpha_x = *alpha;
            alpha_found = true;
        }
        if (auto alpha = parse_child_value<T, T>(node, "float", "alpha_y", env)) {
            alpha_y = *alpha;
            alpha_found = true;
        }
        if (auto alpha = parse_child_value<T, T>(node, "float", "alpha_u", env)) {
            alpha_x = *alpha;
            alpha_found = true;
        }
        if (auto alpha = parse_child_value<T, T>(node, "float", "alpha_v", env)) {
            alpha_y = *alpha;
            alpha_found = true;
        }

        if (parse_child_value<T, T>(node, "float", "roughness", env) ||
            parse_child_value<T, T>(node, "float", "roughness_x", env) ||
            parse_child_value<T, T>(node, "float", "roughness_y", env) ||
            parse_child_value<T, T>(node, "float", "roughness_u", env) ||
            parse_child_value<T, T>(node, "float", "roughness_v", env)) {
            roughness_found = true;
        }

        if (alpha_found && roughness_found) {
            throw std::runtime_error("microfacet model cannot mix alpha and roughness parameters");
        }

        if (!alpha_found) {
            if (auto rough = parse_child_value<T, T>(node, "float", "roughness", env)) {
                alpha_x = alpha_y = roughness_to_alpha(*rough);
            }
            if (auto rough = parse_child_value<T, T>(node, "float", "roughness_x", env)) {
                alpha_x = roughness_to_alpha(*rough);
            }
            if (auto rough = parse_child_value<T, T>(node, "float", "roughness_y", env)) {
                alpha_y = roughness_to_alpha(*rough);
            }
            if (auto rough = parse_child_value<T, T>(node, "float", "roughness_u", env)) {
                alpha_x = roughness_to_alpha(*rough);
            }
            if (auto rough = parse_child_value<T, T>(node, "float", "roughness_v", env)) {
                alpha_y = roughness_to_alpha(*rough);
            }
        }

        if (env.load_microfacet_dist) {
            if (auto distribution_text = parse_child_value<T, std::string>(node, "string", "distribution", env)) {
                distribution = parse_distribution(*distribution_text);
            }
        }

        return material::MicrofacetModel<T>(alpha_x, alpha_y, distribution);
    }

    static void write_node(const material::MicrofacetModel<T>& value, pugi::xml_node& node,
                           const ValueCodecWriteEnv<T>& env) {
        write_child_value<T, T>(node, "float", "alpha_x", value.alpha_x(), env);
        write_child_value<T, T>(node, "float", "alpha_y", value.alpha_y(), env);
        if (env.write_microfacet_dist && value.distribution() == material::MicrofacetDistribution::GGX) {
            write_child_value<T, std::string>(node, "string", "distribution", "ggx", env);
        }
    }

    static material::MicrofacetModel<T> parse_text(std::string_view text, const ValueCodecReadEnv<T>& env) {
        std::string normalized(text);
        std::replace(normalized.begin(), normalized.end(), ',', ' ');

        std::istringstream iss(normalized);
        T alpha_x = T(0.1);
        T alpha_y = T(0.1);
        if (!(iss >> alpha_x)) {
            throw std::runtime_error("invalid microfacet model text");
        }
        if (!(iss >> alpha_y)) {
            alpha_y = alpha_x;
        }
        return material::MicrofacetModel<T>(alpha_x, alpha_y);
    }

    static std::string write_text(const material::MicrofacetModel<T>& value, const ValueCodecWriteEnv<T>& env) {
        std::ostringstream oss;
        oss << ValueCodec<T, T>::write_text(value.alpha_x(), env) << " " << ValueCodec<T, T>::write_text(value.alpha_y(), env);
        return oss.str();
    }

private:
    static T roughness_to_alpha(T roughness) { return std::sqrt(std::max(T(0), roughness)); }

    static material::MicrofacetDistribution parse_distribution(std::string value) {
        std::transform(value.begin(), value.end(), value.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        if (value == "beckmann" || value == "beckman") {
            return material::MicrofacetDistribution::Beckmann;
        }
        if (value == "ggx") {
            return material::MicrofacetDistribution::GGX;
        }
        throw std::runtime_error("invalid microfacet distribution '" + value + "', expected beckmann|ggx");
    }
};

}  // namespace pbpt::serde
