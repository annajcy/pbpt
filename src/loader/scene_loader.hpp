#pragma once

#include <iostream>
#include <filesystem>
#include <format>
#include <stdexcept>
#include <pugixml.hpp>

#include "scene/scene.hpp"
#include "loader/basic_parser.hpp"
#include "loader/transform_loader.hpp"
#include "loader/loader_context.hpp"

// Component headers
#include "camera/plugin/camera/projective_cameras.hpp"
#include "camera/plugin/film/hdr_film.hpp"
#include "camera/plugin/pixel_filter/gaussian_filter.hpp"
#include "camera/pixel_sensor.hpp"
#include "material/plugin/material/lambertian_material.hpp"
#include "shape/plugin/shape/triangle.hpp" // Contains Triangle and TriangleMesh
#include "light/plugin/light/area_light.hpp"
#include "aggregate/plugin/aggregate/embree_aggregate.hpp"
#include "radiometry/constant/illuminant_spectrum.hpp"
#include "radiometry/constant/xyz_spectrum.hpp"
#include "radiometry/plugin/spectrum_distribution/piecewise_linear.hpp"

namespace pbpt::loader {

template <typename T>
void parse_bsdf(const pugi::xml_node& node, LoaderContext<T>& ctx) {
    std::string type = node.attribute("type").value();
    std::string id = node.attribute("id").value();

    if (type == "diffuse") {
        std::string spectrum_str;
        for (auto child : node.children("spectrum")) {
            if (std::string(child.attribute("name").value()) == "reflectance") {
                spectrum_str = child.attribute("value").value();
            }
        }
        
        std::string spec_name = id + "_reflectance";
        auto spectrum = radiometry::PiecewiseLinearSpectrumDistribution<T>::from_string(spectrum_str);
        ctx.resources.reflectance_spectrum_library.add_item(spec_name, std::move(spectrum));
        
        auto mat = material::LambertianMaterial<T>(ctx.resources.reflectance_spectrum_library.get(spec_name));
        ctx.resources.any_material_library.add_item(id, std::move(mat));
    }
}

template <typename T>
void parse_shape(const pugi::xml_node& node, LoaderContext<T>& ctx, const camera::RenderTransform<T>& render_transform) {
    std::string type = node.attribute("type").value();
    if (type == "obj") {
        std::string filename = "";
        // Find filename property
        for (auto child : node.children("string")) {
            if (std::string(child.attribute("name").value()) == "filename") {
                filename = child.attribute("value").value();
            }
        }
        if (filename.empty()) return;

        std::string abs_path = ctx.resolve_path(filename);
        
        auto transform_node = node.child("transform");
        geometry::Transform<T> obj_to_world = geometry::Transform<T>::identity();
        if (transform_node) {
             obj_to_world = load_transform<T>(transform_node);
        }
        
        auto to_render = render_transform.object_to_render_from_object_to_world(obj_to_world);
        
        // Generate name from filename (e.g. "meshes/cbox_floor.obj" -> "cbox_floor")
        std::filesystem::path mesh_path(filename);
        std::string name = mesh_path.stem().string();
        
        ctx.resources.mesh_library.add_item(name, shape::TriangleMesh<T>(to_render, abs_path, false));
        
        // Ref material
        auto ref_node = node.child("ref");
        if (ref_node) {
            std::string mat_id = ref_node.attribute("id").value();
            auto& lib = ctx.resources.any_material_library;
            if (lib.name_to_id().count(mat_id)) {
                ctx.resources.mesh_material_map[name] = lib.name_to_id().at(mat_id);
            }
        }
        
        // Emitter
        auto emitter_node = node.child("emitter");
        if (emitter_node) {
            std::string radiance_str;
            for (auto child : emitter_node.children("spectrum")) {
                if (std::string(child.attribute("name").value()) == "radiance") {
                     radiance_str = child.attribute("value").value();
                }
            }
            
            std::string spec_name = name + "_emission";
            ctx.resources.reflectance_spectrum_library.add_item(spec_name, 
                radiometry::PiecewiseLinearSpectrumDistribution<T>::from_string(radiance_str));
            
            const auto& emission_spec = ctx.resources.reflectance_spectrum_library.get(spec_name);
            auto light_spectrum_dist = radiometry::StandardEmissionSpectrum<T>(
                emission_spec,
                radiometry::constant::CIE_D65_ilum<T>
            );
            
            const auto& mesh = ctx.resources.mesh_library.get(name);
            for (int i = 0; i < mesh.triangle_count(); ++i) {
                std::string light_name = std::format("{}_{}", name, i);
                auto al = light::AreaLight<T, shape::Triangle<T>, decltype(light_spectrum_dist)>(
                    shape::Triangle<T>(mesh, i),
                    light_spectrum_dist,
                    light::AreaLightSamplingDomain::Shape
                );
                int lid = ctx.resources.any_light_library.add_item(light_name, std::move(al));
                ctx.resources.mesh_light_map[light_name] = lid; // This logic needs to match Aggregate construction
            }
        }
    }
}

template <typename T>
scene::Scene<T> load_scene(const std::string& filename) {
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(filename.c_str());
    if (!result) {
        throw std::runtime_error(std::string("XML Load Error: ") + result.description());
    }
    
    scene::Scene<T> scene;
    pugi::xml_node root = doc.child("scene");
    LoaderContext<T> ctx(scene.resources, std::filesystem::path(filename).parent_path());
    
    // 1. BSDFs
    for (auto node : root.children("bsdf")) {
        parse_bsdf(node, ctx);
    }
    
    // 2. Sensor
    auto sensor_node = root.child("sensor");
    if (sensor_node) {
        // Transform
        auto transform_node = sensor_node.child("transform");
        if (transform_node) {
             scene.render_transform = load_render_transform<T>(transform_node);
        }
        
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
        
        scene.camera = camera::ThinLensPerspectiveCamera<T>(
            math::Vector<int, 2>(width, height),
            fov, "smaller",
            near_clip, far_clip, T(focus_dist)
        );
        
        // Pixel Filter
        scene.pixel_filter = camera::GaussianFilter<T>(T(1.5), T(0.5)); // Hardcoded for now or parse
        
        // Film
        scene.film = std::visit([&](auto& cam) {
            auto pixel_sensor = camera::PixelSensor<T, 
                radiometry::constant::CIED65SpectrumType<T>, 
                radiometry::constant::CIED65SpectrumType<T>, 
                radiometry::constant::XYZSpectrumType<T>
            >(
                radiometry::constant::CIE_D65_ilum<T>,
                radiometry::constant::CIE_D65_ilum<T>,
                radiometry::constant::sRGB<T>,
                radiometry::ResponseSpectrum<radiometry::constant::XYZSpectrumType<T>>(
                    radiometry::constant::CIE_X<T>,
                    radiometry::constant::CIE_Y<T>,
                    radiometry::constant::CIE_Z<T>
                ),
                T{1.0}
            );
            // Construct HDRFilm with this sensor
             return camera::AnyFilm<T>(camera::HDRFilm<T, decltype(pixel_sensor)>(cam, pixel_sensor));
        }, scene.camera);
    }
    
    // 3. Shapes
    for (auto node : root.children("shape")) {
        parse_shape(node, ctx, scene.render_transform);
    }
    
    // 4. Aggregate
    std::vector<shape::Primitive<T>> primitives;
    for (const auto& [mesh_name, mesh_id] : scene.resources.mesh_library.name_to_id()) {
        const auto& mesh = scene.resources.mesh_library.get(mesh_id);
        int material_id = 0;
        if (scene.resources.mesh_material_map.count(mesh_name))
            material_id = scene.resources.mesh_material_map.at(mesh_name);
            
        for (int i = 0; i < mesh.triangle_count(); ++i) {
            int light_id = -1;
            std::string light_key = std::format("{}_{}", mesh_name, i);
            if (scene.resources.mesh_light_map.contains(light_key)) {
                light_id = scene.resources.mesh_light_map.at(light_key);
            }
            primitives.push_back(
                shape::Primitive<T>(
                    shape::Triangle<T>(mesh, i),
                    material_id,
                    light_id
                )
            );
        }
    }
    scene.aggregate = aggregate::EmbreeAggregate<T>(std::move(primitives));
    
    return scene;
}

}
