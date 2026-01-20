#pragma once

#include <string>
#include <filesystem>
#include "scene/scene.hpp"

namespace pbpt::loader {

template <typename T>
struct LoaderContext {
    scene::RenderResources<T>& resources;
    std::filesystem::path base_path;
    
    LoaderContext(scene::RenderResources<T>& res, const std::filesystem::path& path)
        : resources(res), base_path(path) {}
    
    std::string resolve_path(const std::string& rel_path) const {
        return (base_path / rel_path).string();
    }
};

}
