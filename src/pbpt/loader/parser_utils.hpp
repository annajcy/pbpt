#pragma once

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <pugixml.hpp>

#include "pbpt/loader/basic_parser.hpp"
#include "pbpt/loader/loader_context.hpp"
#include "pbpt/material/model.hpp"
#include "pbpt/radiometry/constant/lambda.hpp"
#include "pbpt/radiometry/color_spectrum_lut.hpp"
#include "pbpt/radiometry/plugin/spectrum_distribution/piecewise_linear.hpp"

namespace pbpt::loader {

inline bool ends_with(const std::string& str, const std::string& suffix) {
    if (str.size() < suffix.size())
        return false;
    return std::equal(suffix.rbegin(), suffix.rend(), str.rbegin());
}

inline bool starts_with(const std::string& str, const std::string& prefix) {
    if (str.size() < prefix.size())
        return false;
    return std::equal(prefix.begin(), prefix.end(), str.begin());
}

inline std::optional<std::string> find_child_value(const pugi::xml_node& node, const std::string& tag,
                                                   const std::string& name) {
    for (auto child : node.children(tag.c_str())) {
        if (std::string(child.attribute("name").value()) == name) {
            const char* value = child.attribute("value").value();
            if (value && value[0] != '\0') {
                return std::string(value);
            }
        }
    }
    return std::nullopt;
}

inline std::optional<std::string> find_child_reference_id(const pugi::xml_node& node, const std::string& name) {
    for (auto child : node.children("ref")) {
        if (std::string(child.attribute("name").value()) == name) {
            const char* id = child.attribute("id").value();
            if (id && id[0] != '\0') {
                return std::string(id);
            }
        }
    }
    return std::nullopt;
}

template <typename T>
inline std::optional<T> find_float_property(const pugi::xml_node& node, const std::string& name) {
    for (auto child : node.children("float")) {
        if (std::string(child.attribute("name").value()) == name) {
            return parse_value<T>(child.attribute("value").value());
        }
    }
    return std::nullopt;
}

template <typename T>
inline T roughness_to_alpha(T roughness) {
    return std::sqrt(std::max(T(0), roughness));
}

template <typename T>
inline material::MicrofacetModel<T> parse_microfacet_model(const pugi::xml_node& node) {
    T alpha_x = T(0.1);
    T alpha_y = T(0.1);
    bool alpha_found = false;

    if (auto alpha = find_float_property<T>(node, "alpha")) {
        alpha_x = alpha_y = *alpha;
        alpha_found = true;
    }
    if (auto alpha = find_float_property<T>(node, "alpha_x")) {
        alpha_x = *alpha;
        alpha_found = true;
    }
    if (auto alpha = find_float_property<T>(node, "alpha_y")) {
        alpha_y = *alpha;
        alpha_found = true;
    }
    if (auto alpha = find_float_property<T>(node, "alphaU")) {
        alpha_x = *alpha;
        alpha_found = true;
    }
    if (auto alpha = find_float_property<T>(node, "alphaV")) {
        alpha_y = *alpha;
        alpha_found = true;
    }
    if (auto alpha = find_float_property<T>(node, "alpha_u")) {
        alpha_x = *alpha;
        alpha_found = true;
    }
    if (auto alpha = find_float_property<T>(node, "alpha_v")) {
        alpha_y = *alpha;
        alpha_found = true;
    }

    if (!alpha_found) {
        if (auto rough = find_float_property<T>(node, "roughness")) {
            alpha_x = alpha_y = roughness_to_alpha<T>(*rough);
        }
        if (auto rough = find_float_property<T>(node, "roughness_x")) {
            alpha_x = roughness_to_alpha<T>(*rough);
        }
        if (auto rough = find_float_property<T>(node, "roughness_y")) {
            alpha_y = roughness_to_alpha<T>(*rough);
        }
        if (auto rough = find_float_property<T>(node, "roughnessU")) {
            alpha_x = roughness_to_alpha<T>(*rough);
        }
        if (auto rough = find_float_property<T>(node, "roughnessV")) {
            alpha_y = roughness_to_alpha<T>(*rough);
        }
    }

    return material::MicrofacetModel<T>(alpha_x, alpha_y);
}

template <typename T>
inline radiometry::PiecewiseLinearSpectrumDistribution<T> constant_spectrum(T value) {
    return radiometry::PiecewiseLinearSpectrumDistribution<T>(
        {{radiometry::constant::lambda_min<T>, value}, {radiometry::constant::lambda_max<T>, value}});
}

template <typename T>
inline radiometry::PiecewiseLinearSpectrumDistribution<T> load_piecewise_spectrum_from_csv(
    const std::string& abs_path) {
    std::ifstream fin(abs_path);
    if (!fin) {
        throw std::runtime_error("Cannot open spectrum CSV: " + abs_path);
    }

    std::vector<std::pair<T, T>> points;
    std::string line;
    while (std::getline(fin, line)) {
        if (line.empty() || line[0] == '#')
            continue;
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream iss(line);
        T lambda = T(0);
        T value = T(0);
        if (!(iss >> lambda >> value))
            continue;
        points.emplace_back(lambda, value);
    }
    if (points.empty()) {
        throw std::runtime_error("Spectrum CSV contains no data: " + abs_path);
    }
    return radiometry::PiecewiseLinearSpectrumDistribution<T>(points);
}

template <typename T>
inline radiometry::PiecewiseLinearSpectrumDistribution<T> parse_piecewise_spectrum_value(const std::string& value,
                                                                                         const LoaderContext<T>& ctx) {
    if (starts_with(value, "file:")) {
        auto rel_path = value.substr(5);
        return load_piecewise_spectrum_from_csv<T>(ctx.resolve_path(rel_path));
    }
    if (ends_with(value, ".csv")) {
        return load_piecewise_spectrum_from_csv<T>(ctx.resolve_path(value));
    }
    return radiometry::PiecewiseLinearSpectrumDistribution<T>::from_string(value);
}

template <typename T>
inline radiometry::RGB<T> parse_rgb_triplet(const std::string& value) {
    std::string normalized = value;
    std::replace(normalized.begin(), normalized.end(), ',', ' ');

    std::istringstream iss(normalized);
    T r = T(0);
    T g = T(0);
    T b = T(0);
    if (!(iss >> r)) {
        throw std::runtime_error("Invalid rgb reflectance value: " + value);
    }
    if (!(iss >> g))
        g = r;
    if (!(iss >> b))
        b = r;
    return radiometry::RGB<T>(r, g, b);
}

inline texture::WrapMode parse_wrap_mode(const std::string& value) {
    std::string mode = value;
    std::transform(mode.begin(), mode.end(), mode.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (mode == "clamp") {
        return texture::WrapMode::Clamp;
    }
    return texture::WrapMode::Repeat;
}

template <typename T>
inline radiometry::PiecewiseLinearSpectrumDistribution<T> srgb_rgb_to_piecewise(const radiometry::RGB<T>& rgb) {
    const auto rsp = radiometry::lookup_srgb_to_rsp(rgb);
    const auto albedo_spectrum =
        radiometry::RGBAlbedoSpectrumDistribution<T, radiometry::RGBSigmoidPolynomialNormalized>(rsp);

    std::vector<std::pair<T, T>> points{};
    points.reserve(830 - 360 + 1);
    for (int lambda = 360; lambda <= 830; ++lambda) {
        points.emplace_back(static_cast<T>(lambda), albedo_spectrum.at(static_cast<T>(lambda)));
    }

    return radiometry::PiecewiseLinearSpectrumDistribution<T>(points);
}

}  // namespace pbpt::loader
