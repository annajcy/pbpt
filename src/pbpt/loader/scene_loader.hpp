#pragma once

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <format>
#include <iomanip>
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

namespace detail {

template <typename T>
std::string serialize_transform_row_major(const geometry::Transform<T>& transform) {
    std::ostringstream oss;
    oss << std::setprecision(9);
    const auto& matrix = transform.matrix();
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            if (row != 0 || col != 0) {
                oss << ", ";
            }
            oss << matrix.at(row, col);
        }
    }
    return oss.str();
}

template <typename T>
std::string serialize_rgb_reflectance(const material::LambertianMaterial<T>& material) {
    const auto& source = material.reflectance_source();
    return std::visit(
        [](const auto& reflectance) -> std::string {
            using SourceT = std::decay_t<decltype(reflectance)>;
            if constexpr (std::is_same_v<SourceT, radiometry::PiecewiseLinearSpectrumDistribution<T>>) {
                const T r = std::clamp(reflectance.at(T(610.0)), T(0.0), T(1.0));
                const T g = std::clamp(reflectance.at(T(550.0)), T(0.0), T(1.0));
                const T b = std::clamp(reflectance.at(T(460.0)), T(0.0), T(1.0));
                std::ostringstream oss;
                oss << std::setprecision(6) << r << ' ' << g << ' ' << b;
                return oss.str();
            }
            throw std::runtime_error("write_scene currently does not support texture-backed Lambertian materials.");
        },
        source
    );
}

template <typename T>
std::string serialize_spectrum_400_700(const light::AnyLight<T>& any_light) {
    return std::visit(
        [](const auto& light_variant) {
            const auto& power = light_variant.power_spectrum();
            std::ostringstream oss;
            oss << std::setprecision(6)
                << "400:" << std::max(T(0), power.at(T(400))) << ", "
                << "500:" << std::max(T(0), power.at(T(500))) << ", "
                << "600:" << std::max(T(0), power.at(T(600))) << ", "
                << "700:" << std::max(T(0), power.at(T(700)));
            return oss.str();
        },
        any_light
    );
}

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
void write_scene(const scene::Scene<T>& scene, const std::string& filename) {
    if (filename.empty()) {
        throw std::invalid_argument("write_scene filename must not be empty.");
    }

    const auto xml_path = std::filesystem::path(filename);
    const auto scene_dir = xml_path.parent_path();
    const auto mesh_dir = scene_dir / "meshes";
    std::filesystem::create_directories(mesh_dir);

    pugi::xml_document doc;
    auto root = doc.append_child("scene");
    root.append_attribute("version") = "0.5.0";

    auto integrator = root.append_child("integrator");
    integrator.append_attribute("type") = "path";

    auto sensor = root.append_child("sensor");
    sensor.append_attribute("type") = "perspective";
    sensor.append_child("float").append_attribute("name") = "fov";
    sensor.child("float").append_attribute("value") = "45";
    sensor.append_child("string").append_attribute("name") = "fovAxis";
    sensor.child("string").append_attribute("value") = "smaller";
    auto near_clip = sensor.append_child("float");
    near_clip.append_attribute("name") = "nearClip";
    near_clip.append_attribute("value") = "0.1";

    auto sensor_transform = sensor.append_child("transform");
    sensor_transform.append_attribute("name") = "toWorld";
    auto sensor_matrix = sensor_transform.append_child("matrix");
    const auto camera_to_world = scene.render_transform.camera_to_world();
    const auto camera_to_world_text = detail::serialize_transform_row_major(camera_to_world);
    sensor_matrix.append_attribute("value") = camera_to_world_text.c_str();

    const auto camera_resolution = std::visit(
        [](const auto& camera) {
            return camera.film_resolution();
        },
        scene.camera
    );

    auto sampler = sensor.append_child("sampler");
    sampler.append_attribute("type") = "independent";
    sampler.append_child("integer").append_attribute("name") = "sampleCount";
    sampler.child("integer").append_attribute("value") = "4";

    auto film = sensor.append_child("film");
    film.append_attribute("type") = "hdrfilm";
    film.append_child("integer").append_attribute("name") = "width";
    film.child("integer").append_attribute("value") = camera_resolution.x();
    auto film_height = film.append_child("integer");
    film_height.append_attribute("name") = "height";
    film_height.append_attribute("value") = camera_resolution.y();

    std::unordered_map<int, std::string> bsdf_name_by_id{};
    for (const auto& [material_name, material_id] : scene.resources.any_material_library.name_to_id()) {
        const auto& any_material = scene.resources.any_material_library.get(material_id);
        if (!std::holds_alternative<material::LambertianMaterial<T>>(any_material)) {
            throw std::runtime_error("write_scene only supports Lambertian material export.");
        }

        const auto& lambertian = std::get<material::LambertianMaterial<T>>(any_material);
        auto bsdf = root.append_child("bsdf");
        bsdf.append_attribute("type") = "diffuse";
        bsdf.append_attribute("id") = material_name.c_str();
        auto reflectance = bsdf.append_child("rgb");
        reflectance.append_attribute("name") = "reflectance";
        const auto rgb_value = detail::serialize_rgb_reflectance(lambertian);
        reflectance.append_attribute("value") = rgb_value.c_str();
        bsdf_name_by_id[material_id] = material_name;
    }

    for (const auto& [mesh_name, mesh_id] : scene.resources.mesh_library.name_to_id()) {
        const auto& mesh = scene.resources.mesh_library.get(mesh_id);
        const auto obj_filename = mesh_name + ".obj";
        const auto obj_path = mesh_dir / obj_filename;
        detail::write_mesh_obj(mesh, obj_path);

        auto shape = root.append_child("shape");
        shape.append_attribute("type") = "obj";
        shape.append_attribute("id") = mesh_name.c_str();

        auto filename_node = shape.append_child("string");
        filename_node.append_attribute("name") = "filename";
        filename_node.append_attribute("value") = (std::string("meshes/") + obj_filename).c_str();

        auto shape_transform = shape.append_child("transform");
        shape_transform.append_attribute("name") = "toWorld";
        auto shape_matrix = shape_transform.append_child("matrix");
        const auto object_to_world_text = detail::serialize_transform_row_major(mesh.object_to_world_transform());
        shape_matrix.append_attribute("value") = object_to_world_text.c_str();

        if (!scene.resources.mesh_material_map.contains(mesh_name)) {
            throw std::runtime_error("Mesh has no material assignment: " + mesh_name);
        }
        const int material_id = scene.resources.mesh_material_map.at(mesh_name);
        if (!bsdf_name_by_id.contains(material_id)) {
            throw std::runtime_error("Mesh references unknown material id.");
        }

        auto ref = shape.append_child("ref");
        ref.append_attribute("id") = bsdf_name_by_id.at(material_id).c_str();

        int first_light_id = -1;
        for (const auto& [light_key, light_id] : scene.resources.mesh_light_map) {
            if (light_key.mesh_name == mesh_name) {
                first_light_id = light_id;
                break;
            }
        }
        if (first_light_id >= 0) {
            if (!scene.resources.any_light_library.id_to_name().contains(first_light_id)) {
                throw std::runtime_error("Mesh light id is invalid.");
            }

            auto emitter = shape.append_child("emitter");
            emitter.append_attribute("type") = "area";
            auto radiance = emitter.append_child("spectrum");
            radiance.append_attribute("name") = "radiance";
            const auto spectrum_text = detail::serialize_spectrum_400_700(
                scene.resources.any_light_library.get(first_light_id)
            );
            radiance.append_attribute("value") = spectrum_text.c_str();
        }
    }

    std::filesystem::create_directories(scene_dir);
    if (!doc.save_file(xml_path.string().c_str(), "  ")) {
        throw std::runtime_error("Failed to save scene XML to: " + xml_path.string());
    }
}

}  // namespace pbpt::loader
