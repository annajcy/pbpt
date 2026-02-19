#pragma once

#include <string>
#include <filesystem>
#include "pbpt/scene/scene.hpp"

#include "pbpt/loader/component_loader.hpp"

namespace pbpt::loader {

template <typename T>
struct LoaderContext {
    scene::RenderResources<T>& resources;
    const camera::RenderTransform<T>& render_transform;
    std::filesystem::path base_path;

    const MaterialLoaderRegistry<T>& material_registry;
    const TextureLoaderRegistry<T>& texture_registry;
    const ShapeLoaderRegistry<T>& shape_registry;

    LoaderContext(scene::RenderResources<T>& res, const camera::RenderTransform<T>& rt,
                  const std::filesystem::path& path, const MaterialLoaderRegistry<T>& mat_reg,
                  const TextureLoaderRegistry<T>& tex_reg, const ShapeLoaderRegistry<T>& shape_reg)
        : resources(res),
          render_transform(rt),
          base_path(path),
          material_registry(mat_reg),
          texture_registry(tex_reg),
          shape_registry(shape_reg) {}

    std::string resolve_path(const std::string& rel_path) const { return (base_path / rel_path).string(); }
};

}  // namespace pbpt::loader
