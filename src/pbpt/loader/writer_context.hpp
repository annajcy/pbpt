#pragma once

#include <filesystem>
#include <iomanip>
#include <sstream>
#include <string>
#include <unordered_map>

#include "pbpt/radiometry/color.hpp"
#include "pbpt/radiometry/plugin/spectrum_distribution/piecewise_linear.hpp"
#include "pbpt/scene/scene.hpp"

namespace pbpt::loader {

template <typename T>
struct WriterContext {
    const scene::RenderResources<T>& resources;
    std::filesystem::path scene_dir;
    std::filesystem::path mesh_dir;
    std::filesystem::path texture_dir;

    std::unordered_map<int, std::string> texture_name_by_id;
    std::unordered_map<int, std::string> material_name_by_id;

    std::string relative_texture_path(const std::string& name, const std::string& ext) const {
        std::string normalized_ext = ext;
        if (!normalized_ext.empty() && normalized_ext[0] != '.') {
            normalized_ext = "." + normalized_ext;
        }
        return (std::filesystem::path("textures") / (name + normalized_ext)).generic_string();
    }

    std::string relative_mesh_path(const std::string& name) const {
        return (std::filesystem::path("meshes") / (name + ".obj")).generic_string();
    }
};

namespace detail {

template <typename T>
inline std::string serialize_transform_row_major(const geometry::Transform<T>& transform) {
    std::ostringstream oss;
    oss << std::setprecision(9);
    const auto& matrix = transform.matrix();
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            if (row != 0 || col != 0) {
                oss << ", ";
            }
            oss << matrix.at(row, col);
        }
    }
    return oss.str();
}

}  // namespace detail

/**
 * @brief Serialize a piecewise-linear spectrum to the "λ:v, λ:v" string format.
 */
template <typename T>
inline std::string serialize_piecewise_spectrum(const radiometry::PiecewiseLinearSpectrumDistribution<T>& spectrum) {
    std::ostringstream oss;
    oss << std::setprecision(9);
    const auto& points = spectrum.points();
    for (std::size_t i = 0; i < points.size(); ++i) {
        if (i > 0)
            oss << ", ";
        oss << points[i].first << ":" << points[i].second;
    }
    return oss.str();
}

/**
 * @brief Serialize an RGB triplet to a space-separated string.
 */
template <typename T>
inline std::string serialize_rgb(const radiometry::RGB<T>& rgb) {
    std::ostringstream oss;
    oss << std::setprecision(9) << rgb.r() << ' ' << rgb.g() << ' ' << rgb.b();
    return oss.str();
}

}  // namespace pbpt::loader
