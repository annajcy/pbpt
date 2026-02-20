#pragma once

/**
 * @file scene_loader.hpp
 * @brief Scene loading: load_scene<T>() and internal parse helpers.
 *
 * Also re-exports scene_writer.hpp so that existing callers that only include
 * this header continue to find write_scene<T>() without modification.
 */

#include <algorithm>
#include <filesystem>
#include <format>
#include <iostream>
#include <stdexcept>
#include <variant>
#include <vector>

#include <pugixml.hpp>

#include "pbpt/camera/fov_axis.hpp"
#include "pbpt/scene/scene.hpp"
#include "pbpt/loader/loader_context.hpp"
#include "pbpt/loader/component_loader.hpp"
#include "pbpt/loader/transform_loader.hpp"

// Component loaders — must be included to register their factories.
#include "pbpt/texture/plugin/loader.hpp"
#include "pbpt/material/plugin/loader.hpp"
#include "pbpt/shape/plugin/loader.hpp"

// Camera / film construction for the sensor node.
#include "pbpt/camera/plugin/camera/projective_cameras.hpp"
#include "pbpt/camera/plugin/film/hdr_film.hpp"
#include "pbpt/camera/plugin/pixel_filter/gaussian_filter.hpp"
#include "pbpt/camera/pixel_sensor.hpp"
#include "pbpt/aggregate/plugin/aggregate/embree_aggregate.hpp"

// Backward-compat: expose write_scene<T>() from the same include.
#include "pbpt/loader/scene_writer.hpp"

namespace pbpt::loader {

// ─────────────────────────────────────────────────────────────────────────────
// Internal parse helpers
// ─────────────────────────────────────────────────────────────────────────────

template <typename T>
void parse_texture_node(const pugi::xml_node& node, LoaderContext<T>& ctx) {
    const std::string type = node.attribute("type").value();
    const std::string id = node.attribute("id").value();
    if (id.empty())
        return;

    if (ctx.texture_registry.has_loader(type)) {
        auto tex = ctx.texture_registry.create(type, node, ctx);
        ctx.resources.reflectance_texture_library.add_item(id, std::move(tex));
    } else {
        std::cerr << "Warning: Unknown texture type: " << type << '\n';
    }
}

template <typename T>
void parse_bsdf(const pugi::xml_node& node, LoaderContext<T>& ctx) {
    const std::string type = node.attribute("type").value();
    const std::string id = node.attribute("id").value();

    if (ctx.material_registry.has_loader(type)) {
        auto mat = ctx.material_registry.create(type, node, ctx);
        ctx.resources.any_material_library.add_item(id, std::move(mat));
    } else {
        std::cerr << "Warning: Unknown BSDF type: " << type << '\n';
    }
}

template <typename T>
void parse_shape(const pugi::xml_node& node, LoaderContext<T>& ctx) {
    const std::string type = node.attribute("type").value();

    if (ctx.shape_registry.has_loader(type)) {
        ctx.shape_registry.create(type, node, ctx);
    } else {
        std::cerr << "Warning: Unknown shape type: " << type << '\n';
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Load a Mitsuba-compatible XML scene file and return a Scene<T>.
 *
 * @param filename  Path to the scene XML file.
 * @return          Fully assembled Scene<T> ready for rendering.
 */
template <typename T>
scene::Scene<T> load_scene(const std::string& filename) {
    // 1. Instantiate registries locally (no global singletons).
    TextureLoaderRegistry<T> texture_registry;
    MaterialLoaderRegistry<T> material_registry;
    ShapeLoaderRegistry<T> shape_registry;

    register_texture_loaders(texture_registry);
    register_material_loaders(material_registry);
    register_shape_loaders(shape_registry);

    // 2. Parse XML.
    pugi::xml_document doc;
    if (pugi::xml_parse_result r = doc.load_file(filename.c_str()); !r) {
        throw std::runtime_error(std::string("load_scene XML error: ") + r.description());
    }

    scene::Scene<T> scene;
    pugi::xml_node root = doc.child("scene");

    // 3. Pre-parse sensor transform (needed before creating LoaderContext).
    auto sensor_node = root.child("sensor");
    if (sensor_node) {
        if (auto tf = sensor_node.child("transform")) {
            scene.render_transform = load_render_transform<T>(tf);
        }
    }

    // 4. Build context.
    LoaderContext<T> ctx(scene.resources, scene.render_transform, std::filesystem::path(filename).parent_path(),
                         material_registry, texture_registry, shape_registry);

    // 5. Textures → BSDFs → sensor → shapes.
    for (auto node : root.children("texture")) {
        parse_texture_node(node, ctx);
    }
    for (auto node : root.children("bsdf")) {
        parse_bsdf(node, ctx);
    }

    if (sensor_node) {
        const float fov = parse_property<float>(sensor_node, "fov");
        const auto axis_str = parse_property<std::string>(sensor_node, "fovAxis", "smaller");
        const float focus_d = parse_property<float>(sensor_node, "focusDistance");
        const float near_clip = -parse_property<float>(sensor_node, "nearClip", 0.1f);
        const float far_clip = -parse_property<float>(sensor_node, "farClip", 10000.0f);

        int width = 512, height = 512;
        if (auto film_node = sensor_node.child("film")) {
            width = parse_property<int>(film_node, "width", width);
            height = parse_property<int>(film_node, "height", height);
        }

        // Build standard sRGB/D65 pixel sensor + HDR film.
        auto pixel_sensor =
            camera::PixelSensor<T, radiometry::constant::CIED65SpectrumType<T>,
                                radiometry::constant::CIED65SpectrumType<T>, radiometry::constant::XYZSpectrumType<T>>(
                radiometry::constant::CIE_D65_ilum<T>, radiometry::constant::CIE_D65_ilum<T>,
                radiometry::constant::sRGB<T>,
                radiometry::ResponseSpectrum<radiometry::constant::XYZSpectrumType<T>>(
                    radiometry::constant::CIE_X<T>, radiometry::constant::CIE_Y<T>, radiometry::constant::CIE_Z<T>),
                T{1.0});

        auto hdr_film = camera::HDRFilm<T, decltype(pixel_sensor)>(math::Vector<int, 2>(width, height), pixel_sensor);

        scene.camera = camera::ThinLensPerspectiveCamera<T>(camera::AnyFilm<T>(std::move(hdr_film)), T(fov),
                                                            camera::fov_axis_from_string(axis_str), T(near_clip),
                                                            T(far_clip), T(focus_d));

        scene.pixel_filter = camera::GaussianFilter<T>(T(1.5), T(0.5));
    }

    for (auto node : root.children("shape")) {
        parse_shape(node, ctx);
    }

    // 6. Build acceleration structure.
    std::vector<shape::Primitive<T>> primitives;
    for (const auto& [mesh_name, mesh_id] : scene.resources.mesh_library.name_to_id()) {
        const auto& mesh = scene.resources.mesh_library.get(mesh_id);

        if (!scene.resources.mesh_material_map.contains(mesh_name)) {
            throw std::runtime_error("load_scene: no material for mesh: " + mesh_name);
        }
        const int mat_id = scene.resources.mesh_material_map.at(mesh_name);
        if (!scene.resources.any_material_library.id_to_name().contains(mat_id)) {
            throw std::runtime_error("load_scene: invalid material id for mesh: " + mesh_name);
        }

        for (int i = 0; i < mesh.triangle_count(); ++i) {
            int light_id = -1;
            const auto key = scene::make_mesh_triangle_key(mesh_name, i);
            if (scene.resources.mesh_light_map.contains(key)) {
                light_id = scene.resources.mesh_light_map.at(key);
                if (!scene.resources.any_light_library.id_to_name().contains(light_id)) {
                    throw std::runtime_error(std::format("load_scene: invalid light id for {}/{}", mesh_name, i));
                }
            }
            primitives.emplace_back(shape::Triangle<T>(mesh, i), mat_id, light_id);
        }
    }
    scene.aggregate = aggregate::EmbreeAggregate<T>(std::move(primitives));

    return scene;
}

}  // namespace pbpt::loader
