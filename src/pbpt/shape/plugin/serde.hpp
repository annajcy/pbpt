#pragma once

#include <string_view>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <pugixml.hpp>

#include "pbpt/serde/domain/trait_contracts.hpp"
#include "pbpt/serde/value/value_codec_dispatch.hpp"
#include "pbpt/geometry/codec/transform_value_codec.hpp"
#include "pbpt/radiometry/codec/piecewise_spectrum_value_codec.hpp"
#include "pbpt/shape/plugin/shape/triangle.hpp"
#include "pbpt/light/plugin/light/area_light.hpp"
#include "pbpt/scene/scene.hpp"

namespace pbpt::serde {

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

template <typename T>
struct ObjShapeSerde {
    static constexpr std::string_view domain = "shape";
    static constexpr std::string_view xml_type = "obj";

    static void load(const pugi::xml_node& node, LoadContext<T>& ctx) {
        const ValueCodecReadEnv<T> read_env{ctx.resources, ctx.base_path};

        const std::string filename = find_child_value(node, "string", "filename").value_or("");
        if (filename.empty())
            return;

        std::string abs_path = ctx.resolve_path(filename);

        auto transform_node = node.child("transform");
        geometry::Transform<T> obj_to_world = geometry::Transform<T>::identity();
        if (transform_node) {
            obj_to_world = ValueCodec<T, geometry::Transform<T>>::parse_node(transform_node, read_env);
        }

        std::filesystem::path mesh_path(filename);
        std::string name = mesh_path.stem().string();
        std::string shape_id = node.attribute("id").value();
        if (shape_id.empty())
            shape_id = name;

        ctx.resources.mesh_library.add_item(
            name, shape::TriangleMesh<T>(ctx.render_transform, abs_path, false, obj_to_world));

        scene::ShapeInstanceRecord<T> record;
        record.shape_id = shape_id;
        record.shape_type = "obj";
        record.mesh_name = name;
        record.object_to_world = obj_to_world;

        auto ref_node = node.child("ref");
        if (ref_node) {
            std::string mat_id = find_first_reference_id(node).value_or("");
            auto& lib = ctx.resources.any_material_library;
            if (mat_id.empty()) {
                throw std::runtime_error("shape mesh reference has empty material id: " + shape_id);
            }
            if (!lib.name_to_id().contains(mat_id)) {
                throw std::runtime_error("Unknown material reference: " + mat_id + " for shape " + shape_id);
            }
            ctx.resources.mesh_material_map[name] = lib.name_to_id().at(mat_id);
            record.material_ref_name = mat_id;
        }

        auto emitter_node = node.child("emitter");
        if (emitter_node) {
            const auto radiance_str = find_child_value(emitter_node, "spectrum", "radiance").value_or("");

            std::string spec_name = name + "_emission";
            ctx.resources.reflectance_spectrum_library.add_item(
                spec_name, ValueCodec<T, radiometry::PiecewiseLinearSpectrumDistribution<T>>::parse_text(
                               radiance_str, read_env));
            record.emission_spectrum_name = spec_name;

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

        ctx.resources.shape_instances.push_back(std::move(record));
    }

    static void write(const scene::ShapeInstanceRecord<T>& record, pugi::xml_node& node, WriteContext<T>& ctx) {
        const ValueCodecWriteEnv<T> write_env{ctx.resources, ctx.scene_dir, ctx.mesh_dir, ctx.texture_dir};

        node.append_attribute("type") = xml_type.data();
        node.append_attribute("id") = record.shape_id.c_str();

        const auto& mesh = ctx.resources.mesh_library.get(record.mesh_name);
        const auto obj_path = ctx.mesh_dir / (record.mesh_name + ".obj");
        write_mesh_obj(mesh, obj_path);

        auto filename_node = node.append_child("string");
        filename_node.append_attribute("name") = "filename";
        filename_node.append_attribute("value") = ctx.relative_mesh_path(record.mesh_name).c_str();

        auto shape_transform = node.append_child("transform");
        shape_transform.append_attribute("name") = "toWorld";
        auto shape_matrix = shape_transform.append_child("matrix");
        const auto object_to_world_text = ValueCodec<T, geometry::Transform<T>>::write_text(
            record.object_to_world, write_env);
        shape_matrix.append_attribute("value") = object_to_world_text.c_str();

        auto ref = node.append_child("ref");
        ref.append_attribute("id") = record.material_ref_name.c_str();

        if (record.emission_spectrum_name.has_value()) {
            auto emitter = node.append_child("emitter");
            emitter.append_attribute("type") = "area";
            auto radiance = emitter.append_child("spectrum");
            radiance.append_attribute("name") = "radiance";
            const auto spectrum_text = ValueCodec<T, radiometry::PiecewiseLinearSpectrumDistribution<T>>::write_text(
                ctx.resources.reflectance_spectrum_library.get(record.emission_spectrum_name.value()), write_env);
            radiance.append_attribute("value") = spectrum_text.c_str();
        }
    }
};

static_assert(ShapeSerdeConcept<float, ObjShapeSerde<float>>);

}  // namespace pbpt::serde
