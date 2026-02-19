#pragma once

#include <algorithm>
#include <filesystem>
#include <format>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <pugixml.hpp>

#include "pbpt/scene/scene.hpp"
#include "pbpt/loader/loader_context.hpp"
#include "pbpt/loader/component_loader.hpp"
#include "pbpt/loader/transform_loader.hpp"

// Include component loaders (header-only) to ensure registration functions are available
#include "pbpt/texture/plugin/loader.hpp"
#include "pbpt/material/plugin/loader.hpp"
#include "pbpt/shape/plugin/loader.hpp"

// Component headers for Scene assembly
#include "pbpt/camera/plugin/camera/projective_cameras.hpp"
#include "pbpt/camera/plugin/film/hdr_film.hpp"
#include "pbpt/camera/plugin/pixel_filter/gaussian_filter.hpp"
#include "pbpt/camera/pixel_sensor.hpp"
#include "pbpt/aggregate/plugin/aggregate/embree_aggregate.hpp"

namespace pbpt::loader {

template <typename T>
void parse_texture_node(const pugi::xml_node& node, LoaderContext<T>& ctx) {
    const std::string type = node.attribute("type").value();
    const std::string id = node.attribute("id").value();
    if (id.empty()) {
        return;
    }

    if (ctx.texture_registry.has_loader(type)) {
        auto tex = ctx.texture_registry.create(type, node, ctx);
        ctx.resources.reflectance_texture_library.add_item(id, std::move(tex));
    } else {
        std::cerr << "Warning: Unknown texture type: " << type << std::endl;
    }
}

template <typename T>
void parse_bsdf(const pugi::xml_node& node, LoaderContext<T>& ctx) {
    std::string type = node.attribute("type").value();
    std::string id = node.attribute("id").value();

    if (ctx.material_registry.has_loader(type)) {
        auto mat = ctx.material_registry.create(type, node, ctx);
        ctx.resources.any_material_library.add_item(id, std::move(mat));
    } else {
        std::cerr << "Warning: Unknown BSDF type: " << type << std::endl;
    }
}

template <typename T>
void parse_shape(const pugi::xml_node& node, LoaderContext<T>& ctx) {
    std::string type = node.attribute("type").value();

    if (ctx.shape_registry.has_loader(type)) {
        ctx.shape_registry.create(type, node, ctx);
    } else {
        std::cerr << "Warning: Unknown shape type: " << type << std::endl;
    }
}

template <typename T>
scene::Scene<T> load_scene(const std::string& filename) {
    // 1. Instantiate Registries locally
    TextureLoaderRegistry<T> texture_registry;
    MaterialLoaderRegistry<T> material_registry;
    ShapeLoaderRegistry<T> shape_registry;

    // 2. Register known loaders
    register_texture_loaders(texture_registry);
    register_material_loaders(material_registry);
    register_shape_loaders(shape_registry);

    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(filename.c_str());
    if (!result) {
        throw std::runtime_error(std::string("XML Load Error: ") + result.description());
    }

    scene::Scene<T> scene;
    pugi::xml_node root = doc.child("scene");

    // 0. Pre-parse sensor to get RenderTransform (needed for Context)
    auto sensor_node = root.child("sensor");
    if (sensor_node) {
        auto transform_node = sensor_node.child("transform");
        if (transform_node) {
            scene.render_transform = load_render_transform<T>(transform_node);
        }
    }

    // 3. Create Context with registries
    LoaderContext<T> ctx(scene.resources, scene.render_transform, std::filesystem::path(filename).parent_path(),
                         material_registry, texture_registry, shape_registry);

    // 4. Standard Parsing Flow
    for (auto node : root.children("texture")) {
        parse_texture_node(node, ctx);
    }

    for (auto node : root.children("bsdf")) {
        parse_bsdf(node, ctx);
    }

    if (sensor_node) {
        float fov = parse_property<float>(sensor_node, "fov");
        float focus_dist = parse_property<float>(sensor_node, "focusDistance");
        float near_clip = -parse_property<float>(sensor_node, "nearClip", 0.1f);
        float far_clip = -parse_property<float>(sensor_node, "farClip", 10000.0f);

        int width = 512, height = 512;
        auto film_node = sensor_node.child("film");
        if (film_node) {
            width = parse_property<int>(film_node, "width", width);
            height = parse_property<int>(film_node, "height", height);
        }

        scene.camera = camera::ThinLensPerspectiveCamera<T>(math::Vector<int, 2>(width, height), fov, "smaller",
                                                            near_clip, far_clip, T(focus_dist));

        // Pixel Filter
        scene.pixel_filter = camera::GaussianFilter<T>(T(1.5), T(0.5));  // Hardcoded for now or parse

        // Film
        scene.film = std::visit(
            [&](auto& cam) {
                auto pixel_sensor = camera::PixelSensor<T, radiometry::constant::CIED65SpectrumType<T>,
                                                        radiometry::constant::CIED65SpectrumType<T>,
                                                        radiometry::constant::XYZSpectrumType<T>>(
                    radiometry::constant::CIE_D65_ilum<T>, radiometry::constant::CIE_D65_ilum<T>,
                    radiometry::constant::sRGB<T>,
                    radiometry::ResponseSpectrum<radiometry::constant::XYZSpectrumType<T>>(
                        radiometry::constant::CIE_X<T>, radiometry::constant::CIE_Y<T>, radiometry::constant::CIE_Z<T>),
                    T{1.0});
                // Construct HDRFilm with this sensor
                return camera::AnyFilm<T>(camera::HDRFilm<T, decltype(pixel_sensor)>(cam, pixel_sensor));
            },
            scene.camera);
    }

    // 4. Shapes
    for (auto node : root.children("shape")) {
        parse_shape(node, ctx);
    }

    // 5. Aggregate
    std::vector<shape::Primitive<T>> primitives;
    for (const auto& [mesh_name, mesh_id] : scene.resources.mesh_library.name_to_id()) {
        const auto& mesh = scene.resources.mesh_library.get(mesh_id);
        if (!scene.resources.mesh_material_map.contains(mesh_name)) {
            throw std::runtime_error("Mesh has no material assignment: " + mesh_name);
        }

        int material_id = scene.resources.mesh_material_map.at(mesh_name);
        if (!scene.resources.any_material_library.id_to_name().contains(material_id)) {
            throw std::runtime_error("Mesh material id is invalid for mesh: " + mesh_name);
        }

        for (int i = 0; i < mesh.triangle_count(); ++i) {
            int light_id = -1;
            auto light_key = scene::make_mesh_triangle_key(mesh_name, i);
            if (scene.resources.mesh_light_map.contains(light_key)) {
                light_id = scene.resources.mesh_light_map.at(light_key);
                if (!scene.resources.any_light_library.id_to_name().contains(light_id)) {
                    throw std::runtime_error(std::format("Mesh light id is invalid for key: {}_{}", mesh_name, i));
                }
            }
            primitives.push_back(shape::Primitive<T>(shape::Triangle<T>(mesh, i), material_id, light_id));
        }
    }
    scene.aggregate = aggregate::EmbreeAggregate<T>(std::move(primitives));

    return scene;
}

}  // namespace pbpt::loader
