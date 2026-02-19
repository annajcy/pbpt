#pragma once

#include <string>
#include <filesystem>
#include "pbpt/scene/scene.hpp"

namespace pbpt::loader {

template <typename T>
struct LoaderContext {
    scene::RenderResources<T>& resources;
    const camera::RenderTransform<T>& render_transform;
    std::filesystem::path base_path;

    LoaderContext(scene::RenderResources<T>& res, const camera::RenderTransform<T>& rt,
                  const std::filesystem::path& path)
        : resources(res), render_transform(rt), base_path(path) {}

    std::string resolve_path(const std::string& rel_path) const { return (base_path / rel_path).string(); }
};

}  // namespace pbpt::loader
