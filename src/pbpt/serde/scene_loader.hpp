#pragma once

#include <algorithm>
#include <filesystem>
#include <format>
#include <iostream>
#include <stdexcept>
#include <variant>
#include <vector>

#include <pugixml.hpp>

#include "pbpt/scene/scene.hpp"
#include "pbpt/serde/context.hpp"
#include "pbpt/aggregate/plugin/aggregate/embree_aggregate.hpp"
#include "pbpt/serde/dispatch.hpp"
#include "pbpt/serde/domain/typelist.hpp"

namespace pbpt::serde {

template <typename T>
void parse_texture_node(const pugi::xml_node& node, LoadContext<T>& ctx) {
    const std::string type = node.attribute("type").value();
    const std::string id = node.attribute("id").value();
    if (id.empty())
        return;

    auto tex = dispatch_load_texture<T, TextureSerdeList<T>>(type, node, ctx);
    ctx.resources.reflectance_texture_library.add_item(id, std::move(tex));
}

template <typename T>
void parse_bsdf(const pugi::xml_node& node, LoadContext<T>& ctx) {
    const std::string type = node.attribute("type").value();
    const std::string id = node.attribute("id").value();

    auto mat = dispatch_load_material<T, MaterialSerdeList<T>>(type, node, ctx);
    ctx.resources.any_material_library.add_item(id, std::move(mat));
}

template <typename T>
void parse_shape(const pugi::xml_node& node, LoadContext<T>& ctx) {
    const std::string type = node.attribute("type").value();
    dispatch_load_shape<T, ShapeSerdeList<T>>(type, node, ctx);
}

template <typename T>
std::vector<shape::Primitive<T>> build_primitives_from_resources(const scene::RenderResources<T>& resources) {
    std::vector<shape::Primitive<T>> primitives;
    for (const auto& record : resources.shape_instances) {
        const auto& mesh_name = record.mesh_name;
        if (!resources.mesh_library.name_to_id().contains(mesh_name)) {
            throw std::runtime_error(
                std::format("load_scene: shape '{}' references missing mesh '{}'", record.shape_id, mesh_name));
        }

        const auto& mesh = resources.mesh_library.get(mesh_name);

        if (!resources.mesh_material_map.contains(mesh_name)) {
            throw std::runtime_error("load_scene: no material for mesh: " + mesh_name);
        }
        const int mat_id = resources.mesh_material_map.at(mesh_name);
        if (!resources.any_material_library.id_to_name().contains(mat_id)) {
            throw std::runtime_error("load_scene: invalid material id for mesh: " + mesh_name);
        }

        for (int i = 0; i < mesh.triangle_count(); ++i) {
            int light_id = -1;
            const auto key = scene::make_mesh_triangle_key(mesh_name, i);
            if (resources.mesh_light_map.contains(key)) {
                light_id = resources.mesh_light_map.at(key);
                if (!resources.any_light_library.id_to_name().contains(light_id)) {
                    throw std::runtime_error(std::format("load_scene: invalid light id for {}/{}", mesh_name, i));
                }
            }
            primitives.emplace_back(shape::Triangle<T>(mesh, i), mat_id, light_id);
        }
    }
    return primitives;
}

template <typename T>
scene::Scene<T> load_scene(const std::string& filename) {
    pugi::xml_document doc;
    if (pugi::xml_parse_result r = doc.load_file(filename.c_str()); !r) {
        throw std::runtime_error(std::string("load_scene XML error: ") + r.description());
    }

    scene::Scene<T> scene;
    pugi::xml_node root = doc.child("scene");

    // parse XML -> load integrator -> load sensor/camera + sampler -> load textures -> load bsdfs -> load shapes
    auto integrator_node = root.child("integrator");
    if (integrator_node) {
        std::string type = integrator_node.attribute("type").value();
        dispatch_load_integrator<T, IntegratorSerdeList<T>>(type, integrator_node, scene);
    }

    auto sensor_node = root.child("sensor");
    const ValueCodecReadEnv<T> root_read_env{scene.resources, std::filesystem::path(filename).parent_path()};
    if (sensor_node) {
        if (auto tf = sensor_node.child("transform")) {
            scene.render_transform = ValueCodec<T, camera::RenderTransform<T>>::parse_node(tf, root_read_env);
        }
    }

    LoadContext<T> ctx(scene.resources, scene.render_transform, std::filesystem::path(filename).parent_path());

    if (sensor_node) {
        std::string sensor_type = sensor_node.attribute("type").value();
        dispatch_load_camera<T, CameraSerdeList<T>>(sensor_type, sensor_node, scene, ctx);

        auto sampler_node = sensor_node.child("sampler");
        if (sampler_node) {
            std::string sampler_type = sampler_node.attribute("type").value();
            dispatch_load_sampler<T, SamplerSerdeList<T>>(sampler_type, sampler_node, scene);
        }
    }

    for (auto node : root.children("texture")) {
        parse_texture_node(node, ctx);
    }
    for (auto node : root.children("bsdf")) {
        parse_bsdf(node, ctx);
    }
    for (auto node : root.children("shape")) {
        parse_shape(node, ctx);
    }

    auto primitives = build_primitives_from_resources<T>(scene.resources);
    scene.aggregate = aggregate::EmbreeAggregate<T>(std::move(primitives));

    return scene;
}

}  // namespace pbpt::serde
