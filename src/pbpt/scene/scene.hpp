#pragma once

#include <cstddef>
#include <string>
#include <unordered_map>
#include <optional>
#include <vector>

#include "pbpt/aggregate/plugin/aggregate/aggregate_type.hpp"
#include "pbpt/camera/plugin/pixel_filter/pixel_filter_type.hpp"
#include "pbpt/material/plugin/material/material_type.hpp"
#include "pbpt/camera/render_transform.hpp"
#include "pbpt/radiometry/plugin/spectrum_distribution/spectrum_distribution_type.hpp"
#include "pbpt/texture/plugin/texture/texture_type.hpp"

#include "pbpt/light/plugin/light/light_type.hpp"
#include "pbpt/camera/plugin/camera/camera_type.hpp"
#include "pbpt/shape/plugin/shape/shape_type.hpp"
#include "pbpt/geometry/transform.hpp"

namespace pbpt::scene {

struct MeshTriangleKey {
    std::string mesh_name;
    int triangle_index{-1};

    bool operator==(const MeshTriangleKey& other) const = default;
};

struct MeshTriangleKeyHash {
    std::size_t operator()(const MeshTriangleKey& key) const {
        std::size_t h1 = std::hash<std::string>{}(key.mesh_name);
        std::size_t h2 = std::hash<int>{}(key.triangle_index);
        return h1 ^ (h2 + 0x9e3779b9 + (h1 << 6) + (h1 >> 2));
    }
};

inline MeshTriangleKey make_mesh_triangle_key(const std::string& mesh_name, int triangle_index) {
    return MeshTriangleKey{mesh_name, triangle_index};
}

template <typename T>
struct SceneSerializationMeta {
    std::string integrator_type{"path"};
    std::string camera_type{"perspective"};
    std::string sampler_type{"ldsampler"};
    int sample_count{4};
};

template <typename T>
struct ShapeInstanceRecord {
    std::string shape_id;
    std::string shape_type;
    std::string mesh_name;
    std::string material_ref_name;
    geometry::Transform<T> object_to_world{geometry::Transform<T>::identity()};
    std::optional<std::string> emission_spectrum_name;
};

template <typename T>
struct RenderResources {
    // 资源库
    // light material
    light::NamedAnyLightLibrary<T> any_light_library;
    material::NamedAnyMaterialLibrary<T> any_material_library;

    // mesh and spectrum
    shape::NamedMeshLibrary<T> mesh_library;
    radiometry::NamedReflectanceSpectrumLibrary<T> reflectance_spectrum_library;
    texture::NamedTextureLibrary<T> reflectance_texture_library;

    // mesh name to material id map
    std::unordered_map<std::string, int> mesh_material_map;
    // mesh triangle to light id map
    std::unordered_map<MeshTriangleKey, int, MeshTriangleKeyHash> mesh_light_map;

    std::vector<ShapeInstanceRecord<T>> shape_instances;
};

template <typename T, typename CameraT, typename FilmT, typename PixelFilterT, typename AggregateT>
struct SceneContext {
    const CameraT& camera;
    FilmT& film;
    const PixelFilterT& pixel_filter;
    const AggregateT& aggregate;
    const camera::RenderTransform<T>& render_transform;
    const RenderResources<T>& resources;
};

/**
 * @brief 数据驱动的 Scene 类
 *
 * 不再是一个巨大的模板类，而是持有 Variant 数据的容器。
 * 它本身是非模板的（或者仅保留 T 模板如果非常必要），
 * 但通过 scene_types.hpp 中定义的 Types 实现了具体类型的解耦。
 */
template <typename T>
struct Scene {
    // 渲染变换
    camera::RenderTransform<T> render_transform;

    // 场景组件
    // Note: film is now stored inside camera as AnyFilm<T>
    camera::AnyCamera<T> camera;
    camera::AnyPixelFilter<T> pixel_filter;

    // 几何加速结构
    aggregate::AnyAggregate<T> aggregate;

    // 场景几何与材质
    RenderResources<T> resources;

    SceneSerializationMeta<T> serialization_meta;
};

}  // namespace pbpt::scene
