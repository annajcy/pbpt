#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <iostream>

#include "aggregate/aggregate_type.hpp"
#include "camera/film_type.hpp"
#include "camera/pixel_filter_type.hpp"
#include "material/material_type.hpp"
#include "camera/render_transform.hpp"
#include "material/material.hpp"
#include "radiometry/spectrum_distribution_type.hpp"
#include "shape/primitive.hpp"

#include "light/light_type.hpp" 
#include "camera/camera_type.hpp"
#include "shape/shape_type.hpp"

namespace pbpt::scene {
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
    // 场景组件
    camera::AnyCamera<T> camera;
    camera::AnyFilm<T> film;
    camera::AnyPixelFilter<T> pixel_filter;
    camera::RenderTransform<T> render_transform;

    // 场景几何与材质
    aggregate::AnyAggregate<T> aggregate;

    // 资源库
    // light material
    light::NamedAnyLightLibrary<T> light_library;
    material::NamedAnyMaterialLibrary<T> material_library;

    // mesh and spectrum
    shape::NamedMeshLibrary<T> mesh_library;
    radiometry::NamedReflectanceSpectrumLibrary<T> reflectance_spectrum_library;
    
public:
    Scene() = default;

};

}
