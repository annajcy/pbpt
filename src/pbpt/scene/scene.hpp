#pragma once

#include <string>
#include <unordered_map>

#include "pbpt/aggregate/plugin/aggregate/aggregate_type.hpp"
#include "pbpt/camera/plugin/film/film_type.hpp"
#include "pbpt/camera/plugin/pixel_filter/pixel_filter_type.hpp"
#include "pbpt/material/plugin/material/material_type.hpp"
#include "pbpt/camera/render_transform.hpp"
#include "pbpt/radiometry/plugin/spectrum_distribution/spectrum_distribution_type.hpp"
#include "pbpt/texture/plugin/texture/texture_type.hpp"

#include "pbpt/light/plugin/light/light_type.hpp" 
#include "pbpt/camera/plugin/camera/camera_type.hpp"
#include "pbpt/shape/plugin/shape/shape_type.hpp"

namespace pbpt::scene {

template<typename T>
struct RenderResources {
    // 资源库
    // light material
    light::NamedAnyLightLibrary<T> any_light_library;
    material::NamedAnyMaterialLibrary<T> any_material_library;

    // mesh and spectrum
    shape::NamedMeshLibrary<T> mesh_library;
    radiometry::NamedReflectanceSpectrumLibrary<T> reflectance_spectrum_library;
    texture::NamedReflectanceTextureLibrary<T> reflectance_texture_library;

    // mesh name to material id map
    std::unordered_map<std::string, int> mesh_material_map;
    // mesh name to light id map, per triangle
    // format: "mesh_name_triangleIndex" -> lightID
    std::unordered_map<std::string, int> mesh_light_map;
};

template<typename T, typename CameraT, typename FilmT, typename PixelFilterT, typename AggregateT>
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
template<typename T>
class Scene {
public:
    // 渲染变换
    camera::RenderTransform<T> render_transform;

    // 场景组件
    camera::AnyCamera<T> camera;
    camera::AnyFilm<T> film;
    camera::AnyPixelFilter<T> pixel_filter;
    
    // 几何加速结构
    aggregate::AnyAggregate<T> aggregate;
    
    // 场景几何与材质
    RenderResources<T> resources;
};

}
