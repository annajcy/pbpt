#pragma once

#include <filesystem>
#include <stdexcept>
#include <variant>

#include <pugixml.hpp>

#include "pbpt/scene/scene.hpp"
#include "pbpt/serde/xml_result.hpp"
#include "pbpt/serde/context.hpp"
#include "pbpt/serde/dispatch.hpp"
#include "pbpt/serde/domain/typelist.hpp"

namespace pbpt::serde {

template <typename T>
void write_texture_nodes(pugi::xml_node& root, WriteContext<T>& ctx) {
    for (const auto& [texture_name, texture_id] : ctx.result.scene.resources.reflectance_texture_library.name_to_id()) {
        const auto& any_texture = ctx.result.scene.resources.reflectance_texture_library.get(texture_id);
        auto texture_node = root.append_child("texture");
        dispatch_write_texture<T, TextureSerdeList<T>>(any_texture, texture_name, texture_node, ctx);
        ctx.texture_name_by_id[texture_id] = texture_name;
    }
}

template <typename T>
void write_bsdf_nodes(pugi::xml_node& root, WriteContext<T>& ctx) {
    for (const auto& [material_name, material_id] : ctx.result.scene.resources.any_material_library.name_to_id()) {
        const auto& any_material = ctx.result.scene.resources.any_material_library.get(material_id);
        auto bsdf = root.append_child("bsdf");
        dispatch_write_material<T, MaterialSerdeList<T>>(any_material, material_name, bsdf, ctx);
        ctx.material_name_by_id[material_id] = material_name;
    }
}

template <typename T>
void write_shape_nodes(pugi::xml_node& root, WriteContext<T>& ctx) {
    if (ctx.result.scene.resources.shape_instances.empty() && !ctx.result.scene.resources.mesh_library.name_to_id().empty()) {
        for (const auto& [mesh_name, mesh_id] : ctx.result.scene.resources.mesh_library.name_to_id()) {
            const auto& mesh = ctx.result.scene.resources.mesh_library.get(mesh_id);

            scene::ShapeInstanceRecord<T> record;
            record.shape_id = mesh_name;
            record.shape_type = "obj";
            record.mesh_name = mesh_name;
            record.object_to_world = mesh.object_to_world_transform();

            if (!ctx.result.scene.resources.mesh_material_map.contains(mesh_name)) {
                throw std::runtime_error("Mesh has no material assignment: " + mesh_name);
            }
            int material_id = ctx.result.scene.resources.mesh_material_map.at(mesh_name);
            if (!ctx.material_name_by_id.contains(material_id)) {
                throw std::runtime_error("Mesh references unknown material id.");
            }
            record.material_ref_name = ctx.material_name_by_id.at(material_id);

            bool has_light = false;
            for (const auto& [light_key, light_id] : ctx.result.scene.resources.mesh_light_map) {
                if (light_key.mesh_name != mesh_name)
                    continue;
                has_light = true;
                break;
            }

            if (has_light) {
                const auto emission_name = mesh_name + "_emission";
                record.emission_spectrum_name = emission_name;
            }
            auto shape = root.append_child("shape");
            dispatch_write_shape<T, ShapeSerdeList<T>>(record, shape, ctx);
        }
    } else {
        for (const auto& record : ctx.result.scene.resources.shape_instances) {
            auto shape = root.append_child("shape");
            dispatch_write_shape<T, ShapeSerdeList<T>>(record, shape, ctx);
        }
    }
}

template <typename T>
void write_scene(const PbptXmlResult<T>& result, const std::string& filename) {
    if (filename.empty()) {
        throw std::invalid_argument("write_scene: filename must not be empty.");
    }

    const auto xml_path = std::filesystem::path(filename);
    const auto scene_dir = xml_path.parent_path();
    const auto mesh_dir = scene_dir / "meshes";
    const auto texture_dir = scene_dir / "textures";

    std::filesystem::create_directories(mesh_dir);
    std::filesystem::create_directories(texture_dir);

    WriteContext<T> ctx{result, scene_dir, mesh_dir, texture_dir};

    pugi::xml_document doc;
    auto root = doc.append_child("scene");
    root.append_attribute("version") = "3.0.0";

    // Write integrator by dispatching on the runtime AnyIntegrator variant
    auto integrator_node = root.append_child("integrator");
    dispatch_write_integrator<T, IntegratorSerdeList<T>>(result.integrator, integrator_node, ctx);

    // Write camera by dispatching on the runtime AnyCamera variant
    auto sensor = root.append_child("sensor");
    dispatch_write_camera<T, CameraSerdeList<T>>(result.scene.camera, sensor, ctx);

    // Write sampler by dispatching on integrator's sampler variant
    dispatch_write_sampler<T, SamplerSerdeList<T>>(result.integrator, sensor, ctx);

    write_texture_nodes<T>(root, ctx);
    write_bsdf_nodes<T>(root, ctx);
    write_shape_nodes<T>(root, ctx);

    if (!doc.save_file(xml_path.string().c_str(), "  ")) {
        throw std::runtime_error("write_scene: failed to save XML to: " + xml_path.string());
    }
}

}  // namespace pbpt::serde
