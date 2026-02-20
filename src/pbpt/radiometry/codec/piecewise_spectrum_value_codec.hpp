#pragma once

#include <algorithm>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "pbpt/radiometry/color.hpp"
#include "pbpt/radiometry/color_spectrum_lut.hpp"
#include "pbpt/radiometry/constant/lambda.hpp"
#include "pbpt/radiometry/plugin/spectrum_distribution/piecewise_linear.hpp"
#include "pbpt/serde/value/value_codec_traits.hpp"

namespace pbpt::serde {

inline bool starts_with(std::string_view str, std::string_view prefix) {
    return str.size() >= prefix.size() && str.substr(0, prefix.size()) == prefix;
}

inline bool ends_with(std::string_view str, std::string_view suffix) {
    return str.size() >= suffix.size() && str.substr(str.size() - suffix.size(), suffix.size()) == suffix;
}

template <typename T>
inline radiometry::PiecewiseLinearSpectrumDistribution<T> make_constant_piecewise_spectrum(T value) {
    return radiometry::PiecewiseLinearSpectrumDistribution<T>(
        {{radiometry::constant::lambda_min<T>, value}, {radiometry::constant::lambda_max<T>, value}});
}

template <typename T>
inline radiometry::PiecewiseLinearSpectrumDistribution<T> load_piecewise_spectrum_from_csv(const std::string& abs_path) {
    std::ifstream fin(abs_path);
    if (!fin) {
        throw std::runtime_error("Cannot open spectrum CSV: " + abs_path);
    }

    std::vector<std::pair<T, T>> points;
    std::string line;
    while (std::getline(fin, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream iss(line);
        T lambda = T(0);
        T value = T(0);
        if (!(iss >> lambda >> value)) {
            continue;
        }
        points.emplace_back(lambda, value);
    }

    if (points.empty()) {
        throw std::runtime_error("Spectrum CSV contains no data: " + abs_path);
    }

    return radiometry::PiecewiseLinearSpectrumDistribution<T>(points);
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

template <typename T>
struct ValueCodec<T, radiometry::PiecewiseLinearSpectrumDistribution<T>> {
    using value_type = radiometry::PiecewiseLinearSpectrumDistribution<T>;

    static value_type parse_node(const pugi::xml_node& node, const ValueCodecReadEnv<T>& env) {
        const char* value = node.attribute("value").value();
        if (!value || value[0] == '\0') {
            throw std::runtime_error("spectrum node is missing value attribute");
        }
        return parse_text(std::string_view(value), env);
    }

    static void write_node(const value_type& value, pugi::xml_node& node, const ValueCodecWriteEnv<T>& env) {
        const auto text = write_text(value, env);
        node.append_attribute("value") = text.c_str();
    }

    static value_type parse_text(std::string_view text, const ValueCodecReadEnv<T>& env) {
        if (starts_with(text, "file:")) {
            const auto rel_path = std::string(text.substr(5));
            return load_piecewise_spectrum_from_csv<T>(env.resolve_path(rel_path));
        }
        if (ends_with(text, ".csv")) {
            return load_piecewise_spectrum_from_csv<T>(env.resolve_path(std::string(text)));
        }
        return value_type::from_string(std::string(text));
    }

    static std::string write_text(const value_type& value, const ValueCodecWriteEnv<T>&) {
        std::ostringstream oss;
        oss << std::setprecision(std::numeric_limits<T>::max_digits10);
        const auto& points = value.points();
        for (std::size_t i = 0; i < points.size(); ++i) {
            if (i > 0) {
                oss << ", ";
            }
            oss << points[i].first << ":" << points[i].second;
        }
        return oss.str();
    }
};

}  // namespace pbpt::serde
