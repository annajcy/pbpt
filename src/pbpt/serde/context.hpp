#pragma once

#include <filesystem>
#include <string>
#include <unordered_map>

#include "pbpt/serde/xml_result.hpp"

namespace pbpt::serde {

template <typename T>
struct LoadContext {
    PbptXmlResult<T>& result;
    std::filesystem::path base_path;
    std::string resolve_path(const std::string& rel_path) const { return (base_path / rel_path).string(); }
};

template <typename T>
struct WriteContext {
    const PbptXmlResult<T>& result;
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

}  // namespace pbpt::serde
