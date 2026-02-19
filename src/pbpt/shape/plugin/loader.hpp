#pragma once

#include "pbpt/loader/component_loader.hpp"
#include "pbpt/loader/parser_utils.hpp"
#include "pbpt/loader/transform_loader.hpp"

#include "pbpt/shape/plugin/shape/triangle.hpp"
#include "pbpt/light/plugin/light/area_light.hpp"

namespace pbpt::loader {

template <typename T>
void register_shape_loaders(ShapeLoaderRegistry<T>& registry) {
    registry.register_loader("obj", [](const pugi::xml_node& node, LoaderContext<T>& ctx) {
        std::string filename = "";
        // Find filename property
        for (auto child : node.children("string")) {
            if (std::string(child.attribute("name").value()) == "filename") {
                filename = child.attribute("value").value();
            }
        }
        if (filename.empty())
            return;

        std::string abs_path = ctx.resolve_path(filename);

        auto transform_node = node.child("transform");
        geometry::Transform<T> obj_to_world = geometry::Transform<T>::identity();
        if (transform_node) {
            obj_to_world = load_transform<T>(transform_node);
        }

        auto to_render = ctx.render_transform.object_to_render_from_object_to_world(obj_to_world);

        // Generate name from filename (e.g. "meshes/cbox_floor.obj" -> "cbox_floor")
        std::filesystem::path mesh_path(filename);
        std::string name = mesh_path.stem().string();

        ctx.resources.mesh_library.add_item(name, shape::TriangleMesh<T>(to_render, abs_path, false));

        // Ref material
        auto ref_node = node.child("ref");
        if (ref_node) {
            std::string mat_id = ref_node.attribute("id").value();
            auto& lib = ctx.resources.any_material_library;
            if (mat_id.empty()) {
                throw std::runtime_error("shape mesh reference has empty material id: " + name);
            }
            if (!lib.name_to_id().contains(mat_id)) {
                throw std::runtime_error("Unknown material reference: " + mat_id + " for mesh " + name);
            }
            ctx.resources.mesh_material_map[name] = lib.name_to_id().at(mat_id);
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
            ctx.resources.reflectance_spectrum_library.add_item(
                spec_name, radiometry::PiecewiseLinearSpectrumDistribution<T>::from_string(radiance_str));

            const auto& emission_spec = ctx.resources.reflectance_spectrum_library.get(spec_name);
            auto light_spectrum_dist =
                radiometry::StandardEmissionSpectrum<T>(emission_spec, radiometry::constant::CIE_D65_ilum<T>);

            const auto& mesh = ctx.resources.mesh_library.get(name);
            for (int i = 0; i < mesh.triangle_count(); ++i) {
                std::string light_name = std::format("{}_{}", name, i);
                auto al = light::AreaLight<T, shape::Triangle<T>, decltype(light_spectrum_dist)>(
                    shape::Triangle<T>(mesh, i), light_spectrum_dist, light::AreaLightSamplingDomain::Shape);
                int lid = ctx.resources.any_light_library.add_item(light_name, std::move(al));
                ctx.resources.mesh_light_map[scene::make_mesh_triangle_key(name, i)] = lid;
            }
        }
    });
}

}  // namespace pbpt::loader
