#pragma once

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>

#include <pugixml.hpp>

#include "pbpt/loader/writer_context.hpp"

namespace pbpt::loader {

namespace detail {

template <typename T>
void write_mesh_obj(const shape::TriangleMesh<T>& mesh, const std::filesystem::path& mesh_path) {
    std::filesystem::create_directories(mesh_path.parent_path());

    std::ofstream out(mesh_path);
    if (!out) {
        throw std::runtime_error("Failed to open OBJ output path: " + mesh_path.string());
    }
    out << std::setprecision(9);

    const auto render_to_object = mesh.render_to_object_transform();
    for (const auto& position : mesh.positions()) {
        const auto obj_p = render_to_object.transform_point(position);
        out << "v " << obj_p.x() << ' ' << obj_p.y() << ' ' << obj_p.z() << '\n';
    }

    if (mesh.has_uvs()) {
        for (const auto& uv : mesh.uvs()) {
            out << "vt " << uv.x() << ' ' << uv.y() << '\n';
        }
    }

    if (mesh.has_normals()) {
        for (const auto& normal : mesh.normals()) {
            const auto obj_n = render_to_object.transform_normal(normal).normalized();
            out << "vn " << obj_n.x() << ' ' << obj_n.y() << ' ' << obj_n.z() << '\n';
        }
    }

    const bool write_uv = mesh.has_uvs();
    const bool write_normal = mesh.has_normals();
    for (int i = 0; i < mesh.triangle_count(); ++i) {
        const auto triangle = mesh.triangle_indices(i);
        out << "f ";
        for (int corner = 0; corner < 3; ++corner) {
            const int idx = triangle[corner] + 1;
            out << idx;
            if (write_uv || write_normal) {
                out << '/';
                if (write_uv) {
                    out << idx;
                }
                if (write_normal) {
                    out << '/' << idx;
                }
            }
            if (corner < 2) {
                out << ' ';
            }
        }
        out << '\n';
    }
}

}  // namespace detail

template <typename T>
void write_shape_nodes(pugi::xml_node& root, WriterContext<T>& ctx) {
    for (const auto& [mesh_name, mesh_id] : ctx.resources.mesh_library.name_to_id()) {
        const auto& mesh = ctx.resources.mesh_library.get(mesh_id);
        const auto obj_path = ctx.mesh_dir / (mesh_name + ".obj");
        detail::write_mesh_obj(mesh, obj_path);

        auto shape = root.append_child("shape");
        shape.append_attribute("type") = "obj";
        shape.append_attribute("id") = mesh_name.c_str();

        auto filename_node = shape.append_child("string");
        filename_node.append_attribute("name") = "filename";
        filename_node.append_attribute("value") = ctx.relative_mesh_path(mesh_name).c_str();

        auto shape_transform = shape.append_child("transform");
        shape_transform.append_attribute("name") = "toWorld";
        auto shape_matrix = shape_transform.append_child("matrix");
        const auto object_to_world_text = detail::serialize_transform_row_major(mesh.object_to_world_transform());
        shape_matrix.append_attribute("value") = object_to_world_text.c_str();

        if (!ctx.resources.mesh_material_map.contains(mesh_name)) {
            throw std::runtime_error("Mesh has no material assignment: " + mesh_name);
        }
        const int material_id = ctx.resources.mesh_material_map.at(mesh_name);
        if (!ctx.material_name_by_id.contains(material_id)) {
            throw std::runtime_error("Mesh references unknown material id.");
        }

        auto ref = shape.append_child("ref");
        ref.append_attribute("id") = ctx.material_name_by_id.at(material_id).c_str();

        bool has_light = false;
        for (const auto& [light_key, light_id] : ctx.resources.mesh_light_map) {
            if (light_key.mesh_name != mesh_name) {
                continue;
            }
            if (!ctx.resources.any_light_library.id_to_name().contains(light_id)) {
                throw std::runtime_error("Mesh light id is invalid.");
            }
            has_light = true;
            break;
        }

        if (has_light) {
            const auto emission_name = mesh_name + "_emission";
            if (!ctx.resources.reflectance_spectrum_library.name_to_id().contains(emission_name)) {
                throw std::runtime_error("Missing emission spectrum in reflectance_spectrum_library: " + emission_name);
            }

            auto emitter = shape.append_child("emitter");
            emitter.append_attribute("type") = "area";
            auto radiance = emitter.append_child("spectrum");
            radiance.append_attribute("name") = "radiance";
            const auto spectrum_text =
                serialize_piecewise_spectrum(ctx.resources.reflectance_spectrum_library.get(emission_name));
            radiance.append_attribute("value") = spectrum_text.c_str();
        }
    }
}

}  // namespace pbpt::loader
