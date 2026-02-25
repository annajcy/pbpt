#pragma once

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include "pbpt/radiometry/plugin/spectrum_distribution/piecewise_linear.hpp"

namespace pbpt::radiometry::constant {

template <typename T>
using MetalIorSpectrum = PiecewiseLinearSpectrumDistribution<T>;

template <typename T>
struct MetalEtaKSpectrum {
    MetalIorSpectrum<T> eta;
    MetalIorSpectrum<T> k;
};

inline std::string normalize_metal_name(std::string name) {
    std::transform(name.begin(), name.end(), name.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return name;
}

inline std::filesystem::path resolve_metal_spectrum_root(const std::filesystem::path& base_path) {
    auto try_root = [&](const std::filesystem::path& p) -> std::filesystem::path {
        const auto candidate = p / "asset" / "spectrum" / "metal";
        if (std::filesystem::exists(candidate)) {
            return candidate;
        }
        return {};
    };

    if (!base_path.empty()) {
        auto abs = std::filesystem::absolute(base_path);
        if (!std::filesystem::is_directory(abs)) {
            abs = abs.parent_path();
        }

        for (auto current = abs; !current.empty(); current = current.parent_path()) {
            if (const auto found = try_root(current); !found.empty()) {
                return found;
            }
            if (current == current.parent_path()) {
                break;
            }
        }
    }

    if (const auto found = try_root(std::filesystem::current_path()); !found.empty()) {
        return found;
    }

    throw std::runtime_error("Cannot resolve metal spectrum directory: expected asset/spectrum/metal.");
}

template <typename T>
inline MetalIorSpectrum<T> load_metal_ior_csv(const std::filesystem::path& abs_path) {
    std::ifstream fin(abs_path);
    if (!fin) {
        throw std::runtime_error("Cannot open metal IOR CSV: " + abs_path.string());
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
        throw std::runtime_error("Metal IOR CSV contains no data: " + abs_path.string());
    }

    return MetalIorSpectrum<T>(std::move(points));
}

inline std::pair<std::string_view, std::string_view> metal_csv_pair_from_name(std::string_view name) {
    static const std::unordered_map<std::string, std::pair<std::string_view, std::string_view>> kMetalFileMap = {
        {"ag", {"Ag_eta_Johnson_1972.csv", "Ag_k_Johnson_1972.csv"}},
        {"silver", {"Ag_eta_Johnson_1972.csv", "Ag_k_Johnson_1972.csv"}},
        {"al", {"Al_eta_McPeak_2015.csv", "Al_k_McPeak_2015.csv"}},
        {"aluminum", {"Al_eta_McPeak_2015.csv", "Al_k_McPeak_2015.csv"}},
        {"aluminium", {"Al_eta_McPeak_2015.csv", "Al_k_McPeak_2015.csv"}},
        {"au", {"Au_eta_Johnson_1972.csv", "Au_k_Johnson_1972.csv"}},
        {"gold", {"Au_eta_Johnson_1972.csv", "Au_k_Johnson_1972.csv"}},
        {"cu", {"Cu_eta_Johnson_1972.csv", "Cu_k_Johnson_1972.csv"}},
        {"copper", {"Cu_eta_Johnson_1972.csv", "Cu_k_Johnson_1972.csv"}},
        {"fe", {"Fe_eta_Johnson_1974.csv", "Fe_k_Johnson_1974.csv"}},
        {"iron", {"Fe_eta_Johnson_1974.csv", "Fe_k_Johnson_1974.csv"}},
        {"ni", {"Ni_eta_Johnson_1974.csv", "Ni_k_Johnson_1974.csv"}},
        {"nickel", {"Ni_eta_Johnson_1974.csv", "Ni_k_Johnson_1974.csv"}},
        {"ti", {"Ti_eta_Johnson_1974.csv", "Ti_k_Johnson_1974.csv"}},
        {"titanium", {"Ti_eta_Johnson_1974.csv", "Ti_k_Johnson_1974.csv"}},
    };

    const auto normalized = normalize_metal_name(std::string(name));
    if (const auto it = kMetalFileMap.find(normalized); it != kMetalFileMap.end()) {
        return it->second;
    }

    throw std::runtime_error("Unknown metal material: '" + std::string(name) +
                             "'. Supported: Ag, Al, Au, Cu, Fe, Ni, Ti.");
}

template <typename T>
inline MetalEtaKSpectrum<T> get_metal_eta_k_spectrum(const std::string& material_name,
                                                      const std::filesystem::path& base_path = {}) {
    const auto metal_root = resolve_metal_spectrum_root(base_path);
    const auto [eta_csv, k_csv] = metal_csv_pair_from_name(material_name);

    return MetalEtaKSpectrum<T>{
        .eta = load_metal_ior_csv<T>(metal_root / eta_csv),
        .k = load_metal_ior_csv<T>(metal_root / k_csv),
    };
}

}  // namespace pbpt::radiometry::constant
